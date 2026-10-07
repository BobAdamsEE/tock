// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! HSMCI (High Speed MultiMedia Card Interface) SD memory card host for the
//! SAMV71, implementing the read-only `kernel::hil::sdmmc::Sdmmc` interface.
//!
//! Native SD bus mode, 4 bits wide, 25 MHz default speed. Register map:
//! SAMV71 datasheet, "High Speed Multimedia Card Interface (HSMCI)".
//!
//! Base address 0x4000_0000, peripheral ID 18.
//!
//! SAMV71 Xplained Ultra wiring (slot A):
//!   PA25 MCCK  (Peripheral D)     PA28 MCCDA (Peripheral C)
//!   PA30 MCDA0 (Peripheral C)     PA31 MCDA1 (Peripheral C)
//!   PA26 MCDA2 (Peripheral C)     PA27 MCDA3 (Peripheral C)
//!   PD18 card detect, active low (GPIO)
//! The board does the pin muxing; this driver only touches HSMCI registers
//! and reads the card-detect pin.
//!
//! ## Design
//!
//! * **Read-only.** No command that writes, erases or locks the card is ever
//!   issued. The card this was written for holds deleted data that must
//!   survive inspection, and the strongest guarantee of that is no write path.
//! * **Polled data transfer, no DMA.** The CPU drains `RDR` one word at a
//!   time with `RDPROOF` set, so the controller stops the card clock rather
//!   than overrunning if the CPU falls behind. That rules out the D-cache
//!   coherence problem XDMAC would bring (cf. the MCAN message RAM). Large
//!   reads are drained `READ_CHUNK_BLOCKS` at a time from successive deferred
//!   calls, so the kernel is never busy for more than ~0.35 ms at a stretch.
//! * **Asynchronous at the HIL.** Completions are delivered from a deferred
//!   call, never from inside the request, so callers see a normal split-phase
//!   interface. The one genuinely slow step, waiting for the card to leave
//!   its power-up busy state under ACMD41 (typically tens to hundreds of ms),
//!   is paced by an alarm so the kernel keeps running processes meanwhile.

use core::cell::Cell;

use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::sdmmc::{CardInfo, Sdmmc, SdmmcClient, BLOCK_SIZE};
use kernel::hil::time::{Alarm, AlarmClient, ConvertTicks};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use crate::gpio::GPIOPin;

/// HSMCI peripheral identifier, for the PMC.
pub const HSMCI_PID: u32 = 18;

/// Peripheral (master) clock feeding HSMCI, from `pmc::setup_clocks`.
const MCK_HZ: u32 = 150_000_000;

/// Identification-mode clock. The SD specification caps it at 400 kHz.
const INIT_CLOCK_HZ: u32 = 400_000;

/// Data-transfer clock, SD default speed.
const DATA_CLOCK_HZ: u32 = 25_000_000;

/// Interval between ACMD41 attempts while the card reports busy.
const ACMD41_RETRY_MS: u32 = 10;

/// ACMD41 attempts before giving up. The specification allows the card one
/// second to power up; this allows 1.5 s.
const ACMD41_MAX_TRIES: u16 = 150;

/// Blocks drained per deferred call. A larger read is one CMD18 spread over
/// several deferred calls: `RDPROOF` stops the card clock while the CPU is
/// away, so the card simply waits. This bounds how long the kernel spends
/// polling at a stretch -- about 0.35 ms at 25 MHz x 4 bits -- however large
/// the read, which matters because one CMD18 costs this card ~2 ms of access
/// latency and so large reads are much faster than many small ones.
const READ_CHUNK_BLOCKS: u32 = 8;

/// Bound on any single status-polling loop. Hardware timeouts (RTOE, DTOE,
/// CSTOE) are what normally end a stuck operation; this only guards against
/// a controller that never raises them. One SR read is tens of ns, so this is
/// well under a second.
const POLL_LIMIT: u32 = 10_000_000;

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

register_structs! {
    HsmciRegisters {
        (0x00 => cr:    WriteOnly<u32>),
        (0x04 => mr:    ReadWrite<u32>),
        (0x08 => dtor:  ReadWrite<u32>),
        (0x0C => sdcr:  ReadWrite<u32>),
        (0x10 => argr:  ReadWrite<u32>),
        (0x14 => cmdr:  WriteOnly<u32>),
        (0x18 => blkr:  ReadWrite<u32>),
        (0x1C => cstor: ReadWrite<u32>),
        // Four addresses, but successive reads of the first return successive
        // words of a 136-bit response. The other three are not used.
        (0x20 => rspr:  [ReadOnly<u32>; 4]),
        (0x30 => rdr:   ReadOnly<u32>),
        (0x34 => tdr:   WriteOnly<u32>),
        (0x38 => _reserved0),
        (0x40 => sr:    ReadOnly<u32>),
        (0x44 => ier:   WriteOnly<u32>),
        (0x48 => idr:   WriteOnly<u32>),
        (0x4C => imr:   ReadOnly<u32>),
        (0x50 => dma:   ReadWrite<u32>),
        (0x54 => cfg:   ReadWrite<u32>),
        (0x58 => _reserved1),
        (0xE4 => wpmr:  ReadWrite<u32>),
        (0xE8 => wpsr:  ReadOnly<u32>),
        (0xEC => @END),
    }
}

const HSMCI_BASE: StaticRef<HsmciRegisters> =
    unsafe { StaticRef::new(0x4000_0000 as *const HsmciRegisters) };

// CR
const CR_MCIEN: u32 = 1 << 0;
const CR_MCIDIS: u32 = 1 << 1;
const CR_PWSDIS: u32 = 1 << 3;
const CR_SWRST: u32 = 1 << 7;

// MR. The card clock is MCK / ({CLKDIV, CLKODD} + 2), a 9-bit divisor whose
// low bit is CLKODD.
const MR_PWSDIV_MAX: u32 = 7 << 8;
const MR_RDPROOF: u32 = 1 << 11;
const MR_WRPROOF: u32 = 1 << 12;
const MR_CLKODD: u32 = 1 << 16;

// SDCR: slot A, SDCBUS in [7:6].
const SDCR_BUS_1BIT: u32 = 0 << 6;
const SDCR_BUS_4BIT: u32 = 2 << 6;

// DTOR / CSTOR: largest timeout, 15 x 1048576 card clocks.
const TIMEOUT_MAX: u32 = 0x7F;

// CFG
const CFG_FIFOMODE: u32 = 1 << 0;
const CFG_FERRCTRL: u32 = 1 << 4;

// CMDR fields
const RSP_NONE: u32 = 0 << 6;
const RSP_48: u32 = 1 << 6;
const RSP_136: u32 = 2 << 6;
const RSP_R1B: u32 = 3 << 6;
const SPCMD_INIT: u32 = 1 << 8;
const MAXLAT_64: u32 = 1 << 12;
const TRCMD_START: u32 = 1 << 16;
const TRCMD_STOP: u32 = 2 << 16;
const TRDIR_READ: u32 = 1 << 18;
const TRTYP_SINGLE: u32 = 0 << 19;
const TRTYP_MULTIPLE: u32 = 1 << 19;

// SR
const SR_CMDRDY: u32 = 1 << 0;
const SR_RXRDY: u32 = 1 << 1;
const SR_NOTBUSY: u32 = 1 << 5;
const SR_DTIP: u32 = 1 << 4;
const SR_RINDE: u32 = 1 << 16;
const SR_RDIRE: u32 = 1 << 17;
const SR_RCRCE: u32 = 1 << 18;
const SR_RENDE: u32 = 1 << 19;
const SR_RTOE: u32 = 1 << 20;
const SR_DCRCE: u32 = 1 << 21;
const SR_DTOE: u32 = 1 << 22;
const SR_CSTOE: u32 = 1 << 23;
const SR_XFRDONE: u32 = 1 << 27;
const SR_OVRE: u32 = 1 << 30;
const SR_UNRE: u32 = 1 << 31;

const SR_RESPONSE_ERRORS: u32 =
    SR_RINDE | SR_RDIRE | SR_RCRCE | SR_RENDE | SR_RTOE | SR_CSTOE;
const SR_DATA_ERRORS: u32 = SR_DCRCE | SR_DTOE | SR_OVRE | SR_UNRE;

// ---------------------------------------------------------------------------
// Commands (SD Physical Layer Simplified Specification, section 4.7.4)
// ---------------------------------------------------------------------------

/// Response format, which decides both CMDR.RSPTYP and which errors count.
#[derive(Clone, Copy, PartialEq)]
enum Resp {
    None,
    /// R1, R6, R7: 48 bits, CRC protected.
    R1,
    /// R1 followed by busy signalling on DAT0.
    R1b,
    /// R2: 136 bits (CID, CSD).
    R2,
    /// R3: 48 bits, OCR, **no CRC** -- the CRC field is all ones, so RCRCE
    /// is always raised and must be ignored.
    R3,
}

impl Resp {
    fn rsptyp(self) -> u32 {
        match self {
            Resp::None => RSP_NONE,
            Resp::R1 | Resp::R3 => RSP_48,
            Resp::R1b => RSP_R1B,
            Resp::R2 => RSP_136,
        }
    }

    fn error_mask(self) -> u32 {
        match self {
            Resp::None => 0,
            Resp::R3 => SR_RESPONSE_ERRORS & !SR_RCRCE,
            _ => SR_RESPONSE_ERRORS,
        }
    }
}

const CMD0_GO_IDLE: u32 = 0;
const CMD2_ALL_SEND_CID: u32 = 2;
const CMD3_SEND_RELATIVE_ADDR: u32 = 3;
const ACMD6_SET_BUS_WIDTH: u32 = 6;
const CMD6_SWITCH_FUNC: u32 = 6;
const ACMD13_SD_STATUS: u32 = 13;
const CMD7_SELECT_CARD: u32 = 7;
const CMD8_SEND_IF_COND: u32 = 8;
const CMD9_SEND_CSD: u32 = 9;
const CMD12_STOP_TRANSMISSION: u32 = 12;
const CMD16_SET_BLOCKLEN: u32 = 16;
const CMD17_READ_SINGLE: u32 = 17;
const CMD18_READ_MULTIPLE: u32 = 18;
const ACMD41_SD_SEND_OP_COND: u32 = 41;
const ACMD51_SEND_SCR: u32 = 51;
const CMD55_APP_CMD: u32 = 55;

/// CMD8 argument: 2.7-3.6 V, check pattern 0xAA.
const CMD8_ARG: u32 = 0x1AA;
/// ACMD41 argument: 3.2-3.4 V window, plus HCS for a v2 card.
const OCR_VOLTAGE_WINDOW: u32 = 0x00FF_8000;
const OCR_HCS: u32 = 1 << 30;
const OCR_READY: u32 = 1 << 31;
/// CMD6 argument: mode 0 (check, bit 31 clear) with every function group set
/// to 0xF, "keep the current function". Queries without switching anything.
const CMD6_CHECK_ALL_UNCHANGED: u32 = 0x00FF_FFFF;

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
enum State {
    /// Not initialized, or initialization failed.
    Off,
    /// Waiting on the alarm between ACMD41 attempts.
    WaitReady,
    /// Initialization finished; the result waits for the deferred call.
    InitDone,
    /// Initialized and idle.
    Ready,
    /// A read command is outstanding; the deferred call drains it a chunk
    /// at a time.
    Reading,
}

/// SAMV71 HSMCI SD host.
pub struct Hsmci<'a, A: Alarm<'a>> {
    regs: StaticRef<HsmciRegisters>,
    alarm: &'a A,
    card_detect: Option<&'a GPIOPin<'a>>,
    client: OptionalCell<&'a dyn SdmmcClient>,
    deferred_call: DeferredCall,
    state: Cell<State>,
    tries: Cell<u16>,
    /// Card details gathered so far during initialization.
    pending: Cell<CardInfo>,
    info: OptionalCell<CardInfo>,
    init_result: Cell<Result<(), ErrorCode>>,
    buffer: TakeCell<'static, [u8]>,
    read_count: Cell<u32>,
    /// Blocks of the current read already drained into `buffer`.
    read_done: Cell<u32>,
}

impl<'a, A: Alarm<'a>> Hsmci<'a, A> {
    /// `card_detect` is the active-low card-detect input, already configured
    /// as a GPIO input, or `None` for a holder without a switch.
    pub fn new(alarm: &'a A, card_detect: Option<&'a GPIOPin<'a>>) -> Self {
        Hsmci {
            regs: HSMCI_BASE,
            alarm,
            card_detect,
            client: OptionalCell::empty(),
            deferred_call: DeferredCall::new(),
            state: Cell::new(State::Off),
            tries: Cell::new(0),
            pending: Cell::new(CardInfo::default()),
            info: OptionalCell::empty(),
            init_result: Cell::new(Ok(())),
            buffer: TakeCell::empty(),
            read_count: Cell::new(0),
            read_done: Cell::new(0),
        }
    }

    // ----- low-level helpers ---------------------------------------------

    /// Card clock divisor field for `hz`, rounded so the clock never exceeds
    /// it. Returns (MR bits, actual frequency).
    fn clock_bits(hz: u32) -> (u32, u32) {
        // MCK / (div + 2) <= hz  =>  div >= MCK / hz - 2, rounded up.
        let div = MCK_HZ.div_ceil(hz).saturating_sub(2).min(0x1FF);
        let bits = ((div >> 1) & 0xFF) | if div & 1 != 0 { MR_CLKODD } else { 0 };
        (bits, MCK_HZ / (div + 2))
    }

    fn set_clock(&self, hz: u32) -> u32 {
        let (bits, actual) = Self::clock_bits(hz);
        let mr = self.regs.mr.get() & !(0xFF | MR_CLKODD);
        self.regs.mr.set(mr | bits);
        actual
    }

    /// Software reset that keeps the configuration. Needed after a command or
    /// data error, which otherwise leaves the controller wedged.
    fn soft_reset(&self) {
        let mr = self.regs.mr.get();
        let dtor = self.regs.dtor.get();
        let sdcr = self.regs.sdcr.get();
        let cstor = self.regs.cstor.get();
        let cfg = self.regs.cfg.get();
        self.regs.cr.set(CR_SWRST);
        self.regs.mr.set(mr);
        self.regs.dtor.set(dtor);
        self.regs.sdcr.set(sdcr);
        self.regs.cstor.set(cstor);
        self.regs.cfg.set(cfg);
        self.regs.cr.set(CR_MCIEN | CR_PWSDIS);
    }

    fn hw_init(&self) {
        self.regs.cr.set(CR_SWRST);
        self.regs.cr.set(CR_MCIDIS | CR_PWSDIS);
        self.regs.idr.set(0xFFFF_FFFF);
        self.regs.dtor.set(TIMEOUT_MAX);
        self.regs.cstor.set(TIMEOUT_MAX);
        self.regs.cfg.set(CFG_FIFOMODE | CFG_FERRCTRL);
        self.regs.mr.set(MR_PWSDIV_MAX);
        self.set_clock(INIT_CLOCK_HZ);
        self.regs.sdcr.set(SDCR_BUS_1BIT);
        self.regs.dma.set(0); // PIO transfers only
        self.regs.cr.set(CR_MCIEN | CR_PWSDIS);
    }

    /// Issue one command and wait for its response. For R1b, also waits for
    /// the card to release DAT0.
    fn command(&self, cmdr: u32, arg: u32, resp: Resp) -> Result<(), ErrorCode> {
        let data = cmdr & TRCMD_START != 0;
        let mr = self.regs.mr.get() & !(MR_RDPROOF | MR_WRPROOF);
        self.regs
            .mr
            .set(if data { mr | MR_RDPROOF | MR_WRPROOF } else { mr });
        if !data {
            self.regs.blkr.set(0);
        }

        self.regs.argr.set(arg);
        self.regs.cmdr.set(cmdr | resp.rsptyp() | MAXLAT_64);

        let mask = resp.error_mask();
        let mut n = 0;
        loop {
            let sr = self.regs.sr.get();
            if sr & mask != 0 {
                self.soft_reset();
                return Err(if sr & (SR_RTOE | SR_CSTOE) != 0 {
                    ErrorCode::NOACK
                } else {
                    ErrorCode::FAIL
                });
            }
            if sr & SR_CMDRDY != 0 {
                break;
            }
            n += 1;
            if n > POLL_LIMIT {
                self.soft_reset();
                return Err(ErrorCode::FAIL);
            }
        }

        if resp == Resp::R1b {
            self.wait_not_busy()?;
        }
        Ok(())
    }

    fn wait_not_busy(&self) -> Result<(), ErrorCode> {
        let mut n = 0;
        loop {
            let sr = self.regs.sr.get();
            if sr & SR_NOTBUSY != 0 && sr & SR_DTIP == 0 {
                return Ok(());
            }
            n += 1;
            if n > POLL_LIMIT {
                self.soft_reset();
                return Err(ErrorCode::FAIL);
            }
        }
    }

    fn response(&self) -> u32 {
        self.regs.rspr[0].get()
    }

    /// The four words of a 136-bit response, as 16 bytes, MSB first.
    fn response_136(&self) -> [u8; 16] {
        let mut out = [0u8; 16];
        for chunk in out.chunks_exact_mut(4) {
            chunk.copy_from_slice(&self.regs.rspr[0].get().to_be_bytes());
        }
        out
    }

    fn app_command(&self, cmd: u32, arg: u32, resp: Resp, rca: u16) -> Result<(), ErrorCode> {
        self.command(CMD55_APP_CMD, (rca as u32) << 16, Resp::R1)?;
        self.command(cmd, arg, resp)
    }

    /// Drain `out.len()` bytes from RDR. Bytes arrive in bus order, the first
    /// in RDR[7:0].
    fn read_data(&self, out: &mut [u8]) -> Result<(), ErrorCode> {
        for chunk in out.chunks_exact_mut(4) {
            let mut n = 0;
            loop {
                let sr = self.regs.sr.get();
                if sr & SR_RXRDY != 0 {
                    break;
                }
                if sr & SR_DATA_ERRORS != 0 {
                    self.soft_reset();
                    return Err(ErrorCode::FAIL);
                }
                n += 1;
                if n > POLL_LIMIT {
                    self.soft_reset();
                    return Err(ErrorCode::FAIL);
                }
            }
            chunk.copy_from_slice(&self.regs.rdr.get().to_le_bytes());
        }
        Ok(())
    }

    fn wait_xfrdone(&self) -> Result<(), ErrorCode> {
        let mut n = 0;
        loop {
            let sr = self.regs.sr.get();
            if sr & SR_DATA_ERRORS != 0 {
                self.soft_reset();
                return Err(ErrorCode::FAIL);
            }
            if sr & SR_XFRDONE != 0 {
                return Ok(());
            }
            n += 1;
            if n > POLL_LIMIT {
                self.soft_reset();
                return Err(ErrorCode::FAIL);
            }
        }
    }

    // ----- initialization ------------------------------------------------

    /// Power-up through CMD8. Leaves the card ready for ACMD41 polling.
    fn init_start(&self) -> Result<(), ErrorCode> {
        self.hw_init();

        // 74+ clocks with CMD high before the first command.
        self.command(SPCMD_INIT, 0, Resp::None)?;
        self.command(CMD0_GO_IDLE, 0, Resp::None)?;

        let mut info = CardInfo::default();
        // A v1 card ignores CMD8, which surfaces as a response timeout.
        match self.command(CMD8_SEND_IF_COND, CMD8_ARG, Resp::R1) {
            Ok(()) => {
                if self.response() & 0xFFF != CMD8_ARG {
                    return Err(ErrorCode::NOSUPPORT); // voltage not accepted
                }
                info.version2 = true;
            }
            Err(ErrorCode::NOACK) => info.version2 = false,
            Err(e) => return Err(e),
        }
        self.pending.set(info);
        Ok(())
    }

    /// One ACMD41 attempt. `Ok(true)` once the card has finished powering up.
    fn init_poll_ready(&self) -> Result<bool, ErrorCode> {
        let mut info = self.pending.get();
        let arg = OCR_VOLTAGE_WINDOW | if info.version2 { OCR_HCS } else { 0 };
        self.app_command(ACMD41_SD_SEND_OP_COND, arg, Resp::R3, 0)?;
        let ocr = self.response();
        if ocr & OCR_READY == 0 {
            return Ok(false);
        }
        info.ocr = ocr;
        info.high_capacity = ocr & OCR_HCS != 0;
        self.pending.set(info);
        Ok(true)
    }

    /// Identification and setup after the card is ready: CID, RCA, CSD,
    /// select, SCR, 4-bit bus, full-speed clock.
    fn init_finish(&self) -> Result<CardInfo, ErrorCode> {
        let mut info = self.pending.get();

        self.command(CMD2_ALL_SEND_CID, 0, Resp::R2)?;
        info.cid = self.response_136();

        self.command(CMD3_SEND_RELATIVE_ADDR, 0, Resp::R1)?;
        info.rca = (self.response() >> 16) as u16;
        let rca_arg = (info.rca as u32) << 16;

        self.command(CMD9_SEND_CSD, rca_arg, Resp::R2)?;
        info.csd = self.response_136();
        info.block_count = csd_block_count(&info.csd).ok_or(ErrorCode::NOSUPPORT)?;

        self.command(CMD7_SELECT_CARD, rca_arg, Resp::R1b)?;

        if !info.high_capacity {
            // SDSC defaults to its READ_BL_LEN, which may be 1024 or 2048.
            self.command(CMD16_SET_BLOCKLEN, BLOCK_SIZE as u32, Resp::R1)?;
        }

        // SCR. Informational, so a failure is not fatal.
        let scr_result = self.read_short_block(ACMD51_SEND_SCR, 0, Some(info.rca), &mut info.scr);
        if scr_result.is_err() {
            info.scr = [0; 8];
        }

        // 4-bit bus, if the SCR says the card supports it (every SD memory
        // card must, but SCR.SD_BUS_WIDTHS is the authoritative answer).
        let four_bit = scr_result.is_err() || info.scr[1] & 0x04 != 0;
        if four_bit {
            self.app_command(ACMD6_SET_BUS_WIDTH, 2, Resp::R1, info.rca)?;
            self.regs.sdcr.set(SDCR_BUS_4BIT);
            info.bus_width = 4;
        } else {
            info.bus_width = 1;
        }

        info.clock_hz = self.set_clock(DATA_CLOCK_HZ);

        // SD Status (ACMD13): speed class, UHS and video speed grades,
        // application performance class, AU and erase parameters. Sent on the
        // full data bus, so it comes after the switch to 4 bits.
        info.ssr_valid = self
            .read_short_block(ACMD13_SD_STATUS, 0, Some(info.rca), &mut info.ssr)
            .is_ok();

        // CMD6 in mode 0 ("check"): which bus speed modes, command systems,
        // driver strengths and power limits the card supports. Mode 0 only
        // reports -- it switches nothing -- and every group argument is 0xF,
        // "no influence". CMD6 exists only on cards implementing command
        // class 10 (Physical Layer 1.10 and later).
        let ccc = ((info.csd[4] as u32) << 4) | ((info.csd[5] as u32) >> 4);
        info.switch_valid = ccc & (1 << 10) != 0
            && self
                .read_short_block(CMD6_SWITCH_FUNC, CMD6_CHECK_ALL_UNCHANGED, None, &mut info.switch_status)
                .is_ok();

        Ok(info)
    }

    /// A one-block data read shorter than `BLOCK_SIZE` (SCR, SD Status, CMD6
    /// status), optionally as an application command. BLKR has to be set
    /// after the CMD55 prefix, because a command without data clears it.
    fn read_short_block(
        &self,
        cmd: u32,
        arg: u32,
        app_rca: Option<u16>,
        out: &mut [u8],
    ) -> Result<(), ErrorCode> {
        if let Some(rca) = app_rca {
            self.command(CMD55_APP_CMD, (rca as u32) << 16, Resp::R1)?;
        }
        self.regs.blkr.set(((out.len() as u32) << 16) | 1);
        self.command(cmd | TRCMD_START | TRDIR_READ | TRTYP_SINGLE, arg, Resp::R1)?;
        self.read_data(out)?;
        self.wait_xfrdone()
    }

    fn fail_init(&self, e: ErrorCode) {
        self.regs.cr.set(CR_MCIDIS);
        self.init_result.set(Err(e));
        self.state.set(State::InitDone);
        self.deferred_call.set();
    }

    fn poll_ready_step(&self) {
        match self.init_poll_ready() {
            Ok(true) => match self.init_finish() {
                Ok(info) => {
                    self.pending.set(info);
                    self.init_result.set(Ok(()));
                    self.state.set(State::InitDone);
                    self.deferred_call.set();
                }
                Err(e) => self.fail_init(e),
            },
            Ok(false) => {
                let tries = self.tries.get() + 1;
                self.tries.set(tries);
                if tries >= ACMD41_MAX_TRIES {
                    self.fail_init(ErrorCode::BUSY);
                } else {
                    self.state.set(State::WaitReady);
                    self.alarm
                        .set_alarm(self.alarm.now(), self.alarm.ticks_from_ms(ACMD41_RETRY_MS));
                }
            }
            Err(e) => self.fail_init(e),
        }
    }

    // ----- reads ---------------------------------------------------------

    /// Issue CMD17/CMD18 for `count` blocks at `lba`. The data is left for
    /// `read_chunk` to drain.
    fn start_read(&self, lba: u32, count: u32, high_capacity: bool) -> Result<(), ErrorCode> {
        let addr = if high_capacity { lba } else { lba * BLOCK_SIZE as u32 };
        self.regs.blkr.set(((BLOCK_SIZE as u32) << 16) | count);
        let cmd = if count == 1 {
            CMD17_READ_SINGLE | TRTYP_SINGLE
        } else {
            CMD18_READ_MULTIPLE | TRTYP_MULTIPLE
        };
        self.command(cmd | TRCMD_START | TRDIR_READ, addr, Resp::R1)
    }

    /// Drain up to `READ_CHUNK_BLOCKS` more blocks of the outstanding read.
    /// Returns `Ok(true)` once the whole transfer has finished.
    fn read_chunk(&self, buf: &mut [u8]) -> Result<bool, ErrorCode> {
        let (done, total) = (self.read_done.get(), self.read_count.get());
        let n = (total - done).min(READ_CHUNK_BLOCKS);
        let from = done as usize * BLOCK_SIZE;
        self.read_data(&mut buf[from..from + n as usize * BLOCK_SIZE])?;
        self.read_done.set(done + n);
        if done + n < total {
            return Ok(false);
        }
        if total > 1 {
            self.command(CMD12_STOP_TRANSMISSION | TRCMD_STOP, 0, Resp::R1b)?;
        }
        self.wait_xfrdone()?;
        Ok(true)
    }

    /// Abandon a multiple-block read after an error. The card stays in data
    /// state, ignoring every later command, until it sees CMD12.
    fn abort_read(&self) {
        if self.read_count.get() > 1 {
            let _ = self.command(CMD12_STOP_TRANSMISSION | TRCMD_STOP, 0, Resp::R1b);
        }
    }

    fn finish_read(&self, result: Result<(), ErrorCode>) {
        self.state.set(State::Ready);
        let count = self.read_count.get();
        if let Some(buf) = self.buffer.take() {
            self.client.map(move |c| c.read_done(buf, count, result));
        }
    }
}

/// Card capacity in 512-byte blocks, from a CSD image (MSB first).
fn csd_block_count(csd: &[u8; 16]) -> Option<u32> {
    // Bits [msb:lsb] of the 128-bit register.
    let bits = |msb: u32, lsb: u32| -> u32 {
        let mut v = 0u32;
        for bit in (lsb..=msb).rev() {
            let byte = csd[15 - (bit / 8) as usize];
            v = (v << 1) | ((byte >> (bit % 8)) & 1) as u32;
        }
        v
    };
    match bits(127, 126) {
        // CSD 1.0 (SDSC): (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^READ_BL_LEN bytes
        0 => {
            let read_bl_len = bits(83, 80);
            let c_size = bits(73, 62);
            let c_size_mult = bits(49, 47);
            let bytes = ((c_size as u64 + 1) << (c_size_mult + 2)) << read_bl_len;
            Some((bytes / BLOCK_SIZE as u64) as u32)
        }
        // CSD 2.0 (SDHC/SDXC): (C_SIZE+1) * 512 KiB
        1 => Some((bits(69, 48) + 1).checked_mul(1024)?),
        _ => None,
    }
}

impl<'a, A: Alarm<'a>> Sdmmc<'a> for Hsmci<'a, A> {
    fn set_client(&self, client: &'a dyn SdmmcClient) {
        self.client.set(client);
    }

    fn is_card_present(&self) -> bool {
        self.card_detect.map_or(true, |pin| !pin.read())
    }

    fn initialize(&self) -> Result<(), ErrorCode> {
        match self.state.get() {
            State::Off | State::Ready => {}
            _ => return Err(ErrorCode::BUSY),
        }
        self.info.clear();
        self.tries.set(0);
        match self.init_start() {
            Ok(()) => self.poll_ready_step(),
            Err(e) => self.fail_init(e),
        }
        Ok(())
    }

    fn card_info(&self) -> Option<CardInfo> {
        self.info.get()
    }

    fn read_blocks(
        &self,
        buffer: &'static mut [u8],
        lba: u32,
        count: u32,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        if self.state.get() != State::Ready {
            let e = if self.state.get() == State::Off { ErrorCode::OFF } else { ErrorCode::BUSY };
            return Err((e, buffer));
        }
        let info = match self.info.get() {
            Some(i) => i,
            None => return Err((ErrorCode::OFF, buffer)),
        };
        let in_range = lba
            .checked_add(count)
            .is_some_and(|end| end <= info.block_count);
        if count == 0 || count > 0xFFFF || !in_range || buffer.len() < count as usize * BLOCK_SIZE {
            return Err((ErrorCode::INVAL, buffer));
        }

        if let Err(e) = self.start_read(lba, count, info.high_capacity) {
            return Err((e, buffer));
        }
        self.read_count.set(count);
        self.read_done.set(0);
        self.buffer.replace(buffer);
        self.state.set(State::Reading);
        self.deferred_call.set();
        Ok(())
    }
}

impl<'a, A: Alarm<'a>> AlarmClient for Hsmci<'a, A> {
    fn alarm(&self) {
        if self.state.get() == State::WaitReady {
            self.poll_ready_step();
        }
    }
}

impl<'a, A: Alarm<'a>> DeferredCallClient for Hsmci<'a, A> {
    fn register(&'static self) {
        self.deferred_call.register(self);
    }

    fn handle_deferred_call(&self) {
        match self.state.get() {
            State::InitDone => {
                let result = self.init_result.get().map(|()| self.pending.get());
                match result {
                    Ok(info) => {
                        self.info.set(info);
                        self.state.set(State::Ready);
                    }
                    Err(_) => self.state.set(State::Off),
                }
                self.client.map(|c| c.init_done(result));
            }
            State::Reading => {
                let step = self
                    .buffer
                    .map_or(Err(ErrorCode::FAIL), |buf| self.read_chunk(buf));
                match step {
                    Ok(false) => self.deferred_call.set(),
                    Ok(true) => self.finish_read(Ok(())),
                    Err(e) => {
                        self.abort_read();
                        self.finish_read(Err(e));
                    }
                }
            }
            _ => {}
        }
    }
}
