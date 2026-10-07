// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Userspace access to a native-mode SD card host (`hil::sdmmc::Sdmmc`).
//!
//! Read-only, like the HIL beneath it. Unlike the SPI `sdcard` capsule, this
//! one exposes the card's identification registers (CID, CSD, SCR, OCR), and
//! reads several blocks per request.
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
//!
//! ### Upcall 0: `(kind, status, value)`
//!
//! `status` is 0 on success, otherwise an `ErrorCode` value.
//! * kind 0, initialize: `value` = capacity in 512-byte blocks.
//! * kind 1, read: `value` = bytes copied into the application's buffer.
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
//! | 46 | 1  | flags: bit0 high capacity, bit1 physical layer v2+, bit2 SD Status valid, bit3 CMD6 status valid |
//! | 47 | 1  | bus width |
//! | 48 | 4  | capacity in 512-byte blocks |
//! | 52 | 4  | card clock, Hz |
//! | 56 | 64 | SD Status (ACMD13), MSB first |
//! | 120 | 64 | CMD6 check-mode status, MSB first |
//!
//! The first 56 bytes are the original layout; an application that allows a
//! 56-byte buffer still gets exactly that.

use core::cmp;

use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
use kernel::hil::sdmmc::{CardInfo, Sdmmc, SdmmcClient, BLOCK_SIZE};
use kernel::processbuffer::WriteableProcessBuffer;
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::{ErrorCode, ProcessId};

use capsules_core::driver;
pub const DRIVER_NUM: usize = driver::NUM::Sdmmc as usize;

mod rw_allow {
    pub const READ: usize = 0;
    pub const INFO: usize = 1;
    pub const COUNT: u8 = 2;
}

/// Serialized size of `CardInfo`.
pub const CARD_INFO_LEN: usize = 184;

const KIND_INIT: usize = 0;
const KIND_READ: usize = 1;

#[derive(Default)]
pub struct App;

pub struct SdmmcDriver<'a, S: Sdmmc<'a>> {
    host: &'a S,
    kernel_buf: TakeCell<'static, [u8]>,
    max_blocks: usize,
    grants: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<{ rw_allow::COUNT }>>,
    owner: OptionalCell<ProcessId>,
}

impl<'a, S: Sdmmc<'a>> SdmmcDriver<'a, S> {
    /// `kernel_buf` bounds a single read; its length should be a multiple of
    /// `BLOCK_SIZE`.
    pub fn new(
        host: &'a S,
        kernel_buf: &'static mut [u8],
        grants: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<{ rw_allow::COUNT }>>,
    ) -> Self {
        SdmmcDriver {
            host,
            max_blocks: kernel_buf.len() / BLOCK_SIZE,
            kernel_buf: TakeCell::new(kernel_buf),
            grants,
            owner: OptionalCell::empty(),
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
        | ((info.switch_valid as u8) << 3);
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

impl<'a, S: Sdmmc<'a>> SyscallDriver for SdmmcDriver<'a, S> {
    fn command(&self, command_num: usize, arg1: usize, arg2: usize, pid: ProcessId) -> CommandReturn {
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

            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, pid: ProcessId) -> Result<(), kernel::process::Error> {
        self.grants.enter(pid, |_, _| {})
    }
}
