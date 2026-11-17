// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Userspace access to a native-mode SD card host (`hil::sdmmc::Sdmmc`).
//!
//! Read-only unless the board calls `enable_writes`, which hands it the
//! host's separate `SdmmcWrite` interface; until then commands 9-11 fail with
//! `NOSUPPORT`. Unlike the SPI `sdcard` capsule, this one exposes the card's
//! identification registers (CID, CSD, SCR, OCR), and reads several blocks
//! per request.
//!
//! One process at a time owns the card: the first to issue a command keeps it
//! until it exits.
//!
//! ### Commands
//!
//! | # | Arguments       | Effect |
//! |---|-----------------|--------|
//! | 0 |                 | driver exists |
//! | 1 |                 | card-detect switch: returns 1 present, 0 absent |
//! | 2 |                 | initialize; completes with upcall kind 0 |
//! | 3 | `lba`, `count`  | read `count` blocks into RW allow 0; upcall kind 1 |
//! | 4 |                 | copy `CardInfo` into RW allow 1; returns its length |
//! | 5 |                 | maximum blocks per read (kernel buffer size / 512) |
//! | 6 | `which`         | failure diagnostics: 0 last failure, 1 SD Status read at init. Returns `cmd \| stage << 8 \| card_state << 16`, controller SR, R1 |
//! | 7 | `width`, `hz`   | set bus width (1/4) and clock; returns the clock set |
//! | 8 | `ms`            | data timeout, 1-1000 ms (default 100, the SD specification's) |
//! | 9 | `lba`, `count`  | write `count` blocks from RO allow 0; upcall kind 2 |
//! | 10 |                | raw command described in RW allow 2; upcall kind 3 |
//! | 11 | `width`, `hz`  | set the host's bus width and clock only, not the card's; returns the clock set |
//!
//! Commands 9-11 need `enable_writes`.
//!
//! ### Upcall 0: `(kind, status, value)`
//!
//! `status` is 0 on success, otherwise an `ErrorCode` value.
//! * kind 0, initialize: `value` = capacity in 512-byte blocks.
//! * kind 1, read: `value` = bytes copied into the application's buffer.
//! * kind 2, write: `value` = bytes written.
//! * kind 3, raw command: `value` = data bytes moved. The response is in RW
//!   allow 2 even when `status` reports a failure.
//!
//! ### Raw command (RW allow 2, little-endian, at least 40 bytes)
//!
//! Filled in by the application:
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0  | 4 | argument |
//! | 4  | 1 | command index, 0-63 |
//! | 5  | 1 | response: 0 none, 1 R1/R6/R7, 2 R1b, 3 R2, 4 R3/R4 (no CRC) |
//! | 6  | 1 | data: 0 none, 1 read into RW allow 0, 2 write from RO allow 0 |
//! | 7  | 1 | flags: bit0 multiple-block transfer ended by CMD12 (CMD18/CMD25), bit1 send the 74 initialization clocks instead of a command |
//! | 8  | 2 | block length, bytes (1-2048) |
//! | 10 | 2 | block count |
//!
//! Filled in by the kernel when the command completes:
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 12 | 16 | response words: 48-bit responses in the first (bits 39:8), R2 as bits 127:0, most significant word first |
//! | 28 | 4  | host controller status register |
//! | 32 | 4  | CMD12's response, if flag bit0 sent one |
//! | 36 | 4  | data bytes moved |
//!
//! ### `CardInfo` layout (RW allow 1, little-endian integers)
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0  | 16 | CID, MSB first |
//! | 16 | 16 | CSD, MSB first |
//! | 32 | 8  | SCR, MSB first (zero if unread) |
//! | 40 | 4  | OCR |
//! | 44 | 2  | RCA |
//! | 46 | 1  | flags: bit0 high capacity, bit1 physical layer v2+, bit2 SD Status valid, bit3 CMD6 status valid, bit4 fell back to a 1-bit bus |
//! | 47 | 1  | bus width |
//! | 48 | 4  | capacity in 512-byte blocks |
//! | 52 | 4  | card clock, Hz |
//! | 56 | 64 | SD Status (ACMD13), MSB first |
//! | 120 | 64 | CMD6 check-mode status, MSB first |
//!
//! The first 56 bytes are the original layout; an application that allows a
//! 56-byte buffer still gets exactly that.

use core::cell::Cell;
use core::cmp;

use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
use kernel::hil::sdmmc::{
    BLOCK_SIZE, CardInfo, RawCommand, RawData, RawResponse, RawResult, Sdmmc, SdmmcClient,
    SdmmcWrite, SdmmcWriteClient,
};
use kernel::processbuffer::{ReadableProcessBuffer, WriteableProcessBuffer};
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::{ErrorCode, ProcessId};

use capsules_core::driver;
pub const DRIVER_NUM: usize = driver::NUM::Sdmmc as usize;

mod rw_allow {
    pub const READ: usize = 0;
    pub const INFO: usize = 1;
    pub const RAW: usize = 2;
    pub const COUNT: u8 = 3;
}

mod ro_allow {
    pub const WRITE: usize = 0;
    pub const COUNT: u8 = 1;
}

/// Serialized size of `CardInfo`.
pub const CARD_INFO_LEN: usize = 184;

/// Size of the raw command descriptor and result.
pub const RAW_LEN: usize = 40;

const KIND_INIT: usize = 0;
const KIND_READ: usize = 1;
const KIND_WRITE: usize = 2;
const KIND_RAW: usize = 3;

type Grants = Grant<
    App,
    UpcallCount<1>,
    AllowRoCount<{ ro_allow::COUNT }>,
    AllowRwCount<{ rw_allow::COUNT }>,
>;

#[derive(Default)]
pub struct App;

pub struct SdmmcDriver<'a, S: Sdmmc<'a>> {
    host: &'a S,
    /// The host's write interface, if the board enabled writes.
    writer: OptionalCell<&'a dyn SdmmcWrite<'a>>,
    /// The raw command outstanding reads data, to be copied out.
    raw_read: Cell<bool>,
    kernel_buf: TakeCell<'static, [u8]>,
    max_blocks: usize,
    grants: Grants,
    owner: OptionalCell<ProcessId>,
}

impl<'a, S: Sdmmc<'a>> SdmmcDriver<'a, S> {
    /// `kernel_buf` bounds a single read; its length should be a multiple of
    /// `BLOCK_SIZE`.
    pub fn new(host: &'a S, kernel_buf: &'static mut [u8], grants: Grants) -> Self {
        SdmmcDriver {
            host,
            writer: OptionalCell::empty(),
            raw_read: Cell::new(false),
            max_blocks: kernel_buf.len() / BLOCK_SIZE,
            kernel_buf: TakeCell::new(kernel_buf),
            grants,
            owner: OptionalCell::empty(),
        }
    }

    /// Let applications write to the card and issue raw commands, through
    /// `writer` -- normally the same host object as `host`. The board must
    /// also make this capsule `writer`'s `SdmmcWriteClient`.
    pub fn enable_writes(&self, writer: &'a dyn SdmmcWrite<'a>) {
        self.writer.set(writer);
    }

    /// Start a write of `count` blocks from the owner's RO allow 0.
    fn write(
        &self,
        writer: &'a dyn SdmmcWrite<'a>,
        lba: u32,
        count: usize,
        pid: ProcessId,
    ) -> CommandReturn {
        if count == 0 || count > self.max_blocks {
            return CommandReturn::failure(ErrorCode::INVAL);
        }
        let Some(buf) = self.kernel_buf.take() else {
            return CommandReturn::failure(ErrorCode::BUSY);
        };
        let len = count * BLOCK_SIZE;
        let copied = self
            .grants
            .enter(pid, |_, kd| {
                kd.get_readonly_processbuffer(ro_allow::WRITE)
                    .and_then(|b| {
                        b.enter(|app_buf| {
                            if app_buf.len() < len {
                                return false;
                            }
                            app_buf[..len].copy_to_slice(&mut buf[..len]);
                            true
                        })
                    })
                    .unwrap_or(false)
            })
            .unwrap_or(false);
        if !copied {
            self.kernel_buf.replace(buf);
            return CommandReturn::failure(ErrorCode::SIZE);
        }
        match writer.write_blocks(buf, lba, count as u32) {
            Ok(()) => CommandReturn::success(),
            Err((e, buf)) => {
                self.kernel_buf.replace(buf);
                CommandReturn::failure(e)
            }
        }
    }

    /// Start the raw command described in the owner's RW allow 2.
    fn raw(&self, writer: &'a dyn SdmmcWrite<'a>, pid: ProcessId) -> CommandReturn {
        let mut desc = [0u8; 12];
        let got = self
            .grants
            .enter(pid, |_, kd| {
                kd.get_readwrite_processbuffer(rw_allow::RAW)
                    .and_then(|b| {
                        b.enter(|app_buf| {
                            if app_buf.len() < RAW_LEN {
                                return false;
                            }
                            app_buf[..12].copy_to_slice(&mut desc);
                            true
                        })
                    })
                    .unwrap_or(false)
            })
            .unwrap_or(false);
        if !got {
            return CommandReturn::failure(ErrorCode::SIZE);
        }
        let command = RawCommand {
            arg: u32::from_le_bytes([desc[0], desc[1], desc[2], desc[3]]),
            index: desc[4],
            response: match desc[5] {
                0 => RawResponse::None,
                1 => RawResponse::R1,
                2 => RawResponse::R1b,
                3 => RawResponse::R2,
                4 => RawResponse::R3,
                _ => return CommandReturn::failure(ErrorCode::INVAL),
            },
            data: match desc[6] {
                0 => RawData::None,
                1 => RawData::Read,
                2 => RawData::Write,
                _ => return CommandReturn::failure(ErrorCode::INVAL),
            },
            auto_stop: desc[7] & 1 != 0,
            init_clocks: desc[7] & 2 != 0,
            block_len: u16::from_le_bytes([desc[8], desc[9]]),
            block_count: u16::from_le_bytes([desc[10], desc[11]]),
        };
        let Some(buf) = self.kernel_buf.take() else {
            return CommandReturn::failure(ErrorCode::BUSY);
        };
        if command.data == RawData::Write {
            let len = command.block_len as usize * command.block_count as usize;
            if len > buf.len() {
                self.kernel_buf.replace(buf);
                return CommandReturn::failure(ErrorCode::SIZE);
            }
            let copied = self
                .grants
                .enter(pid, |_, kd| {
                    kd.get_readonly_processbuffer(ro_allow::WRITE)
                        .and_then(|b| {
                            b.enter(|app_buf| {
                                if app_buf.len() < len {
                                    return false;
                                }
                                app_buf[..len].copy_to_slice(&mut buf[..len]);
                                true
                            })
                        })
                        .unwrap_or(false)
                })
                .unwrap_or(false);
            if !copied {
                self.kernel_buf.replace(buf);
                return CommandReturn::failure(ErrorCode::SIZE);
            }
        }
        self.raw_read.set(command.data == RawData::Read);
        match writer.raw_command(command, buf) {
            Ok(()) => CommandReturn::success(),
            Err((e, buf)) => {
                self.kernel_buf.replace(buf);
                CommandReturn::failure(e)
            }
        }
    }

    fn upcall(&self, kind: usize, status: Result<(), ErrorCode>, value: usize) {
        let status = match status {
            Ok(()) => 0,
            Err(e) => usize::from(e),
        };
        self.owner.map(|pid| {
            let _ = self.grants.enter(pid, |_, kd| {
                let _ = kd.schedule_upcall(0, (kind, status, value));
            });
        });
    }
}

fn serialize(info: &CardInfo, out: &mut [u8; CARD_INFO_LEN]) {
    out[0..16].copy_from_slice(&info.cid);
    out[16..32].copy_from_slice(&info.csd);
    out[32..40].copy_from_slice(&info.scr);
    out[40..44].copy_from_slice(&info.ocr.to_le_bytes());
    out[44..46].copy_from_slice(&info.rca.to_le_bytes());
    out[46] = (info.high_capacity as u8)
        | ((info.version2 as u8) << 1)
        | ((info.ssr_valid as u8) << 2)
        | ((info.switch_valid as u8) << 3)
        | ((info.bus_fallback as u8) << 4);
    out[47] = info.bus_width;
    out[48..52].copy_from_slice(&info.block_count.to_le_bytes());
    out[52..56].copy_from_slice(&info.clock_hz.to_le_bytes());
    out[56..120].copy_from_slice(&info.ssr);
    out[120..184].copy_from_slice(&info.switch_status);
}

impl<'a, S: Sdmmc<'a>> SdmmcClient for SdmmcDriver<'a, S> {
    fn init_done(&self, result: Result<CardInfo, ErrorCode>) {
        let blocks = result.as_ref().map_or(0, |i| i.block_count as usize);
        self.upcall(KIND_INIT, result.map(|_| ()), blocks);
    }

    fn read_done(&self, buffer: &'static mut [u8], count: u32, result: Result<(), ErrorCode>) {
        let mut copied = 0;
        if result.is_ok() {
            let len = count as usize * BLOCK_SIZE;
            self.owner.map(|pid| {
                let _ = self.grants.enter(pid, |_, kd| {
                    copied = kd
                        .get_readwrite_processbuffer(rw_allow::READ)
                        .and_then(|rb| {
                            rb.mut_enter(|app_buf| {
                                let n = cmp::min(app_buf.len(), len);
                                app_buf[..n].copy_from_slice(&buffer[..n]);
                                n
                            })
                        })
                        .unwrap_or(0);
                });
            });
        }
        self.kernel_buf.replace(buffer);
        self.upcall(KIND_READ, result, copied);
    }
}

impl<'a, S: Sdmmc<'a>> SdmmcWriteClient for SdmmcDriver<'a, S> {
    fn write_done(&self, buffer: &'static mut [u8], count: u32, result: Result<(), ErrorCode>) {
        self.kernel_buf.replace(buffer);
        let written = if result.is_ok() {
            count as usize * BLOCK_SIZE
        } else {
            0
        };
        self.upcall(KIND_WRITE, result, written);
    }

    fn raw_done(&self, buffer: &'static mut [u8], raw: RawResult, status: Result<(), ErrorCode>) {
        let mut out = [0u8; RAW_LEN - 12];
        for (i, w) in raw.resp.iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        out[16..20].copy_from_slice(&raw.sr.to_le_bytes());
        out[20..24].copy_from_slice(&raw.stop_resp.to_le_bytes());
        out[24..28].copy_from_slice(&raw.bytes.to_le_bytes());
        let len = raw.bytes as usize;
        self.owner.map(|pid| {
            let _ = self.grants.enter(pid, |_, kd| {
                let _ = kd.get_readwrite_processbuffer(rw_allow::RAW).and_then(|b| {
                    b.mut_enter(|app_buf| {
                        if app_buf.len() >= RAW_LEN {
                            app_buf[12..RAW_LEN].copy_from_slice(&out);
                        }
                    })
                });
                // Read data (a write's data is not copied back: the buffer
                // still holds what was sent).
                let _ = kd
                    .get_readwrite_processbuffer(rw_allow::READ)
                    .and_then(|b| {
                        b.mut_enter(|app_buf| {
                            let n = cmp::min(app_buf.len(), len);
                            if self.raw_read.get() {
                                app_buf[..n].copy_from_slice(&buffer[..n]);
                            }
                        })
                    });
            });
        });
        self.kernel_buf.replace(buffer);
        self.upcall(KIND_RAW, status, len);
    }
}

impl<'a, S: Sdmmc<'a>> SyscallDriver for SdmmcDriver<'a, S> {
    fn command(
        &self,
        command_num: usize,
        arg1: usize,
        arg2: usize,
        pid: ProcessId,
    ) -> CommandReturn {
        if command_num == 0 {
            return CommandReturn::success();
        }

        // Claim the card for this process unless a live process holds it.
        let available = self.owner.map_or(true, |owner| {
            owner == pid || self.grants.enter(owner, |_, _| ()).is_err()
        });
        if !available {
            return CommandReturn::failure(ErrorCode::RESERVE);
        }
        self.owner.set(pid);

        match command_num {
            1 => CommandReturn::success_u32(self.host.is_card_present() as u32),

            2 => CommandReturn::from(self.host.initialize()),

            3 => {
                let count = arg2;
                if count == 0 || count > self.max_blocks {
                    return CommandReturn::failure(ErrorCode::INVAL);
                }
                match self.kernel_buf.take() {
                    None => CommandReturn::failure(ErrorCode::BUSY),
                    Some(buf) => match self.host.read_blocks(buf, arg1 as u32, count as u32) {
                        Ok(()) => CommandReturn::success(),
                        Err((e, buf)) => {
                            self.kernel_buf.replace(buf);
                            CommandReturn::failure(e)
                        }
                    },
                }
            }

            4 => match self.host.card_info() {
                None => CommandReturn::failure(ErrorCode::OFF),
                Some(info) => {
                    let mut bytes = [0u8; CARD_INFO_LEN];
                    serialize(&info, &mut bytes);
                    let res = self
                        .grants
                        .enter(pid, |_, kd| {
                            kd.get_readwrite_processbuffer(rw_allow::INFO)
                                .and_then(|b| {
                                    b.mut_enter(|app_buf| {
                                        let n = cmp::min(app_buf.len(), CARD_INFO_LEN);
                                        app_buf[..n].copy_from_slice(&bytes[..n]);
                                        n
                                    })
                                })
                                .map_err(ErrorCode::from)
                        })
                        .map_err(ErrorCode::from)
                        .and_then(|r| r);
                    match res {
                        Ok(n) => CommandReturn::success_u32(n as u32),
                        Err(e) => CommandReturn::failure(e),
                    }
                }
            },

            5 => CommandReturn::success_u32(self.max_blocks as u32),

            // Failure diagnostics: arg1 0 = last failed operation, 1 = the SD
            // Status read during initialization. Returns
            // (cmd | stage << 8 | card_state << 16, controller SR, R1).
            6 => {
                let e = if arg1 == 1 {
                    self.host.init_error()
                } else {
                    self.host.last_error()
                };
                CommandReturn::success_u32_u32_u32(
                    (e.cmd as u32) | ((e.stage as u32) << 8) | ((e.card_state as u32) << 16),
                    e.sr,
                    e.r1,
                )
            }

            // Set bus width (arg1: 1 or 4) and clock (arg2, Hz). Returns the
            // clock actually set.
            7 => match self.host.set_bus(arg1 as u8, arg2 as u32) {
                Ok(hz) => CommandReturn::success_u32(hz),
                Err(e) => CommandReturn::failure(e),
            },

            // Data timeout in ms (arg1).
            8 => CommandReturn::from(self.host.set_data_timeout(arg1 as u32)),

            9 => match self.writer.get() {
                None => CommandReturn::failure(ErrorCode::NOSUPPORT),
                Some(w) => self.write(w, arg1 as u32, arg2, pid),
            },

            10 => match self.writer.get() {
                None => CommandReturn::failure(ErrorCode::NOSUPPORT),
                Some(w) => self.raw(w, pid),
            },

            11 => match self.writer.get() {
                None => CommandReturn::failure(ErrorCode::NOSUPPORT),
                Some(w) => match w.set_host_bus(arg1 as u8, arg2 as u32) {
                    Ok(hz) => CommandReturn::success_u32(hz),
                    Err(e) => CommandReturn::failure(e),
                },
            },

            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, pid: ProcessId) -> Result<(), kernel::process::Error> {
        self.grants.enter(pid, |_, _| {})
    }
}
