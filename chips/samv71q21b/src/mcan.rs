// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2025.

//! MCAN (Bosch M_CAN) driver for SAMV71Q21B — Classic CAN.
//!
//! Implements the Tock `hil::can` traits for the on-chip MCAN controller.
//!
//! # SAMV71-Specific Quirks
//!
//! 1. **CAN core clock = PCK5** — The SAMV71 feeds the MCAN protocol
//!    engine from Programmable Clock 5, NOT from a generic clock (GCLK).
//!    Configure via `pmc::PMC.configure_pck(5, css, pres)`.
//!
//! 2. **Message RAM DMA base (CCFG_CAN0)** — The MCAN controller forms
//!    message RAM addresses as `{CCFG_CAN0.CAN0DMABA[15:0], field[13:0], 2'b00}`.
//!    The board must write `0x2040_0000` to CCFG_CAN0 (Matrix offset 0x110)
//!    so the upper 16 address bits point at SRAM.
//!
//! 3. **D-Cache coherence** — On Cortex-M7, the MCAN's DMA reads/writes
//!    bypass the CPU D-Cache.  The driver cleans cache lines before TX
//!    and invalidates before RX to keep CPU and DMA views consistent.
//!    For best results, place the message RAM buffer in a 32-byte-aligned
//!    static to avoid false-sharing with adjacent kernel data.
//!
//! # Bit Timing (500 kbps, 20 MHz CAN clock)
//!
//! PCK5 = PLLA (300 MHz) / 15 = 20 MHz.
//! 40 TQ per bit → 500 kbps.  NTSEG1=34, NTSEG2=5,
//! sync_seg=1 TQ.  Sample point = 35/40 = 87.5%.
//!
//! # Message RAM Layout (allocated in SRAM, 32-byte aligned)
//!
//! | Section             | Elements | Bytes each | Total |
//! |---------------------|----------|------------|-------|
//! | Std ID Filters      | 4        | 4          | 16    |
//! | Ext ID Filters      | 4        | 8          | 32    |
//! | Rx FIFO 0           | 8        | 16         | 128   |
//! | Tx Buffers          | 4        | 16         | 64    |
//! | **Total**           |          |            | 240   |
//!
//! # Pin Assignments (SAM V71 Xplained Ultra)
//!
//! MCAN1 is connected to the on-board ATA6561 CAN transceiver:
//! - MCAN1_TX: PC14 (Peripheral C)
//! - MCAN1_RX: PC12 (Peripheral C)

use core::cell::Cell;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::can::{self, StandardBitTiming};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, register_structs, ReadOnly, ReadWrite,
};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

// ---------------------------------------------------------------------------
// MCAN base addresses and peripheral IDs
// ---------------------------------------------------------------------------

const MCAN0_BASE: u32 = 0x4003_0000;
const MCAN1_BASE: u32 = 0x4003_4000;

pub const MCAN0_PID: u32 = 35;
pub const MCAN1_PID: u32 = 37;

// CAN core clock: PCK5 = PLLA (300 MHz) / 15 = 20 MHz.
const CAN_CLK_HZ: u32 = 20_000_000;

// ---------------------------------------------------------------------------
// Message RAM sizing
// ---------------------------------------------------------------------------

const STD_FILTER_COUNT: usize = 4;
const EXT_FILTER_COUNT: usize = 4;
// At 500 kbit/s a consecutive frame arrives roughly every 260 us. Eight
// elements is only ~2 ms of headroom, which is not enough to cover a flash
// write or a slow client callback once several clients share the peripheral.
const RX_FIFO0_COUNT: usize = 32;
const TX_BUF_COUNT: usize = 4;

// Classic CAN element: 2 header words + 2 data words = 16 bytes
const CAN_ELEMENT_WORDS: usize = 4;
const STD_FILTER_WORDS: usize = STD_FILTER_COUNT; // 1 word each
const EXT_FILTER_WORDS: usize = EXT_FILTER_COUNT * 2; // 2 words each
const FILTER_WORDS: usize = STD_FILTER_WORDS + EXT_FILTER_WORDS;

// Filter element encodings (SAMV71 datasheet, "Standard/Extended Message ID
// Filter Element"). Only the classic "identifier + mask" filter type is used.
//
//   standard, 1 word:  [31:30] SFT | [29:27] SFEC | [26:16] SFID1 | [10:0] SFID2
//   extended, 2 words: w0 = [31:29] EFEC | [28:0] EFID1
//                      w1 = [31:30] EFT  | [28:0] EFID2
//
// SFID1/EFID1 hold the identifier and SFID2/EFID2 hold the mask.
const FILTER_TYPE_CLASSIC: u32 = 0b10; // SFT / EFT: identifier + mask
const FILTER_CONFIG_FIFO0: u32 = 0b001; // SFEC / EFEC: store in RX FIFO 0
const FILTER_CONFIG_DISABLED: u32 = 0b000; // SFEC / EFEC: element disabled

const STD_ID_MASK: u32 = 0x7FF;
const EXT_ID_MASK: u32 = 0x1FFF_FFFF;
const RX_FIFO0_WORDS: usize = RX_FIFO0_COUNT * CAN_ELEMENT_WORDS;
const TX_BUF_WORDS: usize = TX_BUF_COUNT * CAN_ELEMENT_WORDS;

const MSG_RAM_WORDS: usize =
    STD_FILTER_WORDS + EXT_FILTER_WORDS + RX_FIFO0_WORDS + TX_BUF_WORDS;

// Word offsets within the message RAM array
const STD_FILTER_OFFSET: usize = 0;
const EXT_FILTER_OFFSET: usize = STD_FILTER_OFFSET + STD_FILTER_WORDS;
const RX_FIFO0_OFFSET: usize = EXT_FILTER_OFFSET + EXT_FILTER_WORDS;
const TX_BUF_OFFSET: usize = RX_FIFO0_OFFSET + RX_FIFO0_WORDS;

// ---------------------------------------------------------------------------
// Register map (Bosch M_CAN, SAMV71 instantiation)
// ---------------------------------------------------------------------------

register_structs! {
    McanRegisters {
        (0x00 => crel: ReadOnly<u32>),
        (0x04 => endn: ReadOnly<u32>),
        (0x08 => cust: ReadWrite<u32>),
        (0x0C => dbtp: ReadWrite<u32, DBTP::Register>),
        (0x10 => test: ReadWrite<u32, TEST::Register>),
        (0x14 => rwd: ReadWrite<u32>),
        (0x18 => cccr: ReadWrite<u32, CCCR::Register>),
        (0x1C => nbtp: ReadWrite<u32, NBTP::Register>),
        (0x20 => tscc: ReadWrite<u32>),
        (0x24 => tscv: ReadOnly<u32>),
        (0x28 => tocc: ReadWrite<u32>),
        (0x2C => tocv: ReadOnly<u32>),
        (0x30 => _reserved0),
        (0x40 => ecr: ReadOnly<u32, ECR::Register>),
        (0x44 => psr: ReadOnly<u32, PSR::Register>),
        (0x48 => tdcr: ReadWrite<u32>),
        (0x4C => _reserved1),
        (0x50 => ir: ReadWrite<u32, IR::Register>),
        (0x54 => ie: ReadWrite<u32, IE::Register>),
        (0x58 => ils: ReadWrite<u32>),
        (0x5C => ile: ReadWrite<u32, ILE::Register>),
        (0x60 => _reserved2),
        (0x80 => gfc: ReadWrite<u32, GFC::Register>),
        (0x84 => sidfc: ReadWrite<u32, SIDFC::Register>),
        (0x88 => xidfc: ReadWrite<u32, XIDFC::Register>),
        (0x8C => _reserved3),
        (0x90 => xidam: ReadWrite<u32>),
        (0x94 => hpms: ReadOnly<u32>),
        (0x98 => ndat1: ReadWrite<u32>),
        (0x9C => ndat2: ReadWrite<u32>),
        (0xA0 => rxf0c: ReadWrite<u32, RXF0C::Register>),
        (0xA4 => rxf0s: ReadOnly<u32, RXF0S::Register>),
        (0xA8 => rxf0a: ReadWrite<u32, RXF0A::Register>),
        (0xAC => rxbc: ReadWrite<u32>),
        (0xB0 => rxf1c: ReadWrite<u32>),
        (0xB4 => rxf1s: ReadOnly<u32>),
        (0xB8 => rxf1a: ReadWrite<u32>),
        (0xBC => rxesc: ReadWrite<u32>),
        (0xC0 => txbc: ReadWrite<u32, TXBC::Register>),
        (0xC4 => txfqs: ReadOnly<u32, TXFQS::Register>),
        (0xC8 => txesc: ReadWrite<u32>),
        (0xCC => txbrp: ReadOnly<u32>),
        (0xD0 => txbar: ReadWrite<u32>),
        (0xD4 => txbcr: ReadWrite<u32>),
        (0xD8 => txbto: ReadOnly<u32>),
        (0xDC => txbcf: ReadOnly<u32>),
        (0xE0 => txbtie: ReadWrite<u32>),
        (0xE4 => txbcie: ReadWrite<u32>),
        (0xE8 => _reserved4),
        (0xFC => _reserved_end),
        (0x100 => @END),
    }
}

// ---------------------------------------------------------------------------
// Bitfield definitions
// ---------------------------------------------------------------------------

register_bitfields![u32,
    DBTP [
        TDC   OFFSET(23) NUMBITS(1) [],
        DBRP  OFFSET(16) NUMBITS(5) [],
        DTSEG1 OFFSET(8) NUMBITS(5) [],
        DTSEG2 OFFSET(4) NUMBITS(4) [],
        DSJW  OFFSET(0)  NUMBITS(4) []
    ],

    TEST [
        SVAL  OFFSET(21) NUMBITS(1) [],
        TXBNS OFFSET(20) NUMBITS(1) [],
        TXBNP OFFSET(19) NUMBITS(1) [],
        TX    OFFSET(5)  NUMBITS(2) [],
        RX    OFFSET(7)  NUMBITS(1) [],
        LBCK  OFFSET(4)  NUMBITS(1) []
    ],

    CCCR [
        TXP   OFFSET(14) NUMBITS(1) [],
        EFBI  OFFSET(13) NUMBITS(1) [],
        PXHD  OFFSET(12) NUMBITS(1) [],
        BRSE  OFFSET(9)  NUMBITS(1) [],
        FDOE  OFFSET(8)  NUMBITS(1) [],
        TEST  OFFSET(7)  NUMBITS(1) [],
        DAR   OFFSET(6)  NUMBITS(1) [],
        MON   OFFSET(5)  NUMBITS(1) [],
        CSR   OFFSET(4)  NUMBITS(1) [],
        CSA   OFFSET(3)  NUMBITS(1) [],
        ASM   OFFSET(2)  NUMBITS(1) [],
        CCE   OFFSET(1)  NUMBITS(1) [],
        INIT  OFFSET(0)  NUMBITS(1) []
    ],

    NBTP [
        NSJW   OFFSET(25) NUMBITS(7) [],
        NBRP   OFFSET(16) NUMBITS(9) [],
        NTSEG1 OFFSET(8)  NUMBITS(8) [],
        NTSEG2 OFFSET(0)  NUMBITS(7) []
    ],

    ECR [
        CEL OFFSET(16) NUMBITS(8) [],
        RP  OFFSET(15) NUMBITS(1) [],
        REC OFFSET(8)  NUMBITS(7) [],
        TEC OFFSET(0)  NUMBITS(8) []
    ],

    PSR [
        TDCV OFFSET(16) NUMBITS(7) [],
        PXE  OFFSET(14) NUMBITS(1) [],
        RFDF OFFSET(13) NUMBITS(1) [],
        RBRS OFFSET(12) NUMBITS(1) [],
        RESI OFFSET(11) NUMBITS(1) [],
        DLEC OFFSET(8)  NUMBITS(3) [],
        BO   OFFSET(7)  NUMBITS(1) [],
        EW   OFFSET(6)  NUMBITS(1) [],
        EP   OFFSET(5)  NUMBITS(1) [],
        ACT  OFFSET(3)  NUMBITS(2) [],
        LEC  OFFSET(0)  NUMBITS(3) []
    ],

    IR [
        ARA   OFFSET(28) NUMBITS(1) [],
        PED   OFFSET(27) NUMBITS(1) [],
        PEA   OFFSET(26) NUMBITS(1) [],
        WDI   OFFSET(25) NUMBITS(1) [],
        BO_   OFFSET(24) NUMBITS(1) [],
        EW_   OFFSET(23) NUMBITS(1) [],
        EP_   OFFSET(22) NUMBITS(1) [],
        ELO   OFFSET(21) NUMBITS(1) [],
        BEU   OFFSET(20) NUMBITS(1) [],
        BEC   OFFSET(19) NUMBITS(1) [],
        DRX   OFFSET(18) NUMBITS(1) [],
        TOO   OFFSET(17) NUMBITS(1) [],
        MRAF  OFFSET(16) NUMBITS(1) [],
        TSW   OFFSET(15) NUMBITS(1) [],
        TEFL  OFFSET(14) NUMBITS(1) [],
        TEFF  OFFSET(13) NUMBITS(1) [],
        TEFN  OFFSET(12) NUMBITS(1) [],
        TFE   OFFSET(11) NUMBITS(1) [],
        TCF   OFFSET(10) NUMBITS(1) [],
        TC    OFFSET(9)  NUMBITS(1) [],
        HPM   OFFSET(8)  NUMBITS(1) [],
        RF1L  OFFSET(7)  NUMBITS(1) [],
        RF1F  OFFSET(6)  NUMBITS(1) [],
        RF1N  OFFSET(5)  NUMBITS(1) [],
        RF0L  OFFSET(3)  NUMBITS(1) [],
        RF0F  OFFSET(2)  NUMBITS(1) [],
        RF0W  OFFSET(1)  NUMBITS(1) [],
        RF0N  OFFSET(0)  NUMBITS(1) []
    ],

    IE [
        ARAE  OFFSET(28) NUMBITS(1) [],
        PEDE  OFFSET(27) NUMBITS(1) [],
        PEAE  OFFSET(26) NUMBITS(1) [],
        WDIE  OFFSET(25) NUMBITS(1) [],
        BOE   OFFSET(24) NUMBITS(1) [],
        EWE   OFFSET(23) NUMBITS(1) [],
        EPE   OFFSET(22) NUMBITS(1) [],
        ELOE  OFFSET(21) NUMBITS(1) [],
        BEUE  OFFSET(20) NUMBITS(1) [],
        BECE  OFFSET(19) NUMBITS(1) [],
        DRXE  OFFSET(18) NUMBITS(1) [],
        TOOE  OFFSET(17) NUMBITS(1) [],
        MRAFE OFFSET(16) NUMBITS(1) [],
        TSWE  OFFSET(15) NUMBITS(1) [],
        TEFLE OFFSET(14) NUMBITS(1) [],
        TEFFE OFFSET(13) NUMBITS(1) [],
        TEFNE OFFSET(12) NUMBITS(1) [],
        TFEE  OFFSET(11) NUMBITS(1) [],
        TCFE  OFFSET(10) NUMBITS(1) [],
        TCE   OFFSET(9)  NUMBITS(1) [],
        HPME  OFFSET(8)  NUMBITS(1) [],
        RF1LE OFFSET(7)  NUMBITS(1) [],
        RF1FE OFFSET(6)  NUMBITS(1) [],
        RF1NE OFFSET(5)  NUMBITS(1) [],
        RF0LE OFFSET(3)  NUMBITS(1) [],
        RF0FE OFFSET(2)  NUMBITS(1) [],
        RF0WE OFFSET(1)  NUMBITS(1) [],
        RF0NE OFFSET(0)  NUMBITS(1) []
    ],

    ILE [
        EINT1 OFFSET(1) NUMBITS(1) [],
        EINT0 OFFSET(0) NUMBITS(1) []
    ],

    GFC [
        ANFS OFFSET(4) NUMBITS(2) [],
        ANFE OFFSET(2) NUMBITS(2) [],
        RRFS OFFSET(1) NUMBITS(1) [],
        RRFE OFFSET(0) NUMBITS(1) []
    ],

    SIDFC [
        LSS    OFFSET(16) NUMBITS(8) [],
        FLSSA  OFFSET(2)  NUMBITS(14) []
    ],

    XIDFC [
        LSE    OFFSET(16) NUMBITS(7) [],
        FLESA  OFFSET(2)  NUMBITS(14) []
    ],

    RXF0C [
        F0OM  OFFSET(31) NUMBITS(1) [],
        F0WM  OFFSET(24) NUMBITS(7) [],
        F0S   OFFSET(16) NUMBITS(7) [],
        F0SA  OFFSET(2)  NUMBITS(14) []
    ],

    RXF0S [
        RF0L  OFFSET(25) NUMBITS(1) [],
        F0F   OFFSET(24) NUMBITS(1) [],
        F0PI  OFFSET(16) NUMBITS(6) [],
        F0GI  OFFSET(8)  NUMBITS(6) [],
        F0FL  OFFSET(0)  NUMBITS(7) []
    ],

    RXF0A [
        F0AI OFFSET(0) NUMBITS(6) []
    ],

    TXBC [
        TFQS  OFFSET(24) NUMBITS(6) [],
        NDTB  OFFSET(16) NUMBITS(6) [],
        TBSA  OFFSET(2)  NUMBITS(14) []
    ],

    TXFQS [
        TFQF  OFFSET(21) NUMBITS(1) [],
        TFQPI OFFSET(16) NUMBITS(5) [],
        TFGI  OFFSET(8)  NUMBITS(5) [],
        TFFL  OFFSET(0)  NUMBITS(6) []
    ]
];

// ---------------------------------------------------------------------------
// Driver state
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq)]
enum McanState {
    Disabled,
    Running,
    Error(can::Error),
}

impl From<McanState> for can::State {
    fn from(s: McanState) -> can::State {
        match s {
            McanState::Running => can::State::Running,
            McanState::Disabled => can::State::Disabled,
            McanState::Error(e) => can::State::Error(e),
        }
    }
}

/// Does this error describe the controller, or just one frame that went wrong?
///
/// Bus-off, error-passive and error-warning are *states* the controller is in,
/// reported by their own interrupts and cleared by their own recovery. The
/// rest come from `PSR.LEC`, which records what went wrong with the last
/// frame: they say nothing about whether the next one will succeed, and must
/// not be left latched.
fn is_controller_state(e: can::Error) -> bool {
    matches!(
        e,
        can::Error::BusOff | can::Error::Passive | can::Error::Warning
    )
}

#[derive(Clone, Copy)]
enum AsyncAction {
    Enable,
    EnableError(ErrorCode),
    Disable,
    AbortReceive,
}

// ---------------------------------------------------------------------------
// Message RAM buffer type (statically allocated by the board)
// ---------------------------------------------------------------------------

// 32-byte alignment matches the Cortex-M7 D-Cache line size,
// preventing false-sharing with adjacent kernel data.
#[repr(C, align(32))]
pub struct MessageRam {
    words: [u32; MSG_RAM_WORDS],
}

impl MessageRam {
    pub const fn new() -> Self {
        MessageRam {
            words: [0u32; MSG_RAM_WORDS],
        }
    }
}

const CACHE_LINE: usize = 32;

// ---------------------------------------------------------------------------
// MCAN driver struct
// ---------------------------------------------------------------------------

pub struct Mcan {
    regs: StaticRef<McanRegisters>,
    msg_ram: TakeCell<'static, MessageRam>,
    msg_ram_base: Cell<u32>,

    state: Cell<McanState>,

    bit_timing: OptionalCell<can::BitTiming>,
    operating_mode: OptionalCell<can::OperationMode>,
    automatic_retransmission: Cell<bool>,
    automatic_wake_up: Cell<bool>,

    controller_client: OptionalCell<&'static dyn can::ControllerClient>,
    transmit_client:
        OptionalCell<&'static dyn can::TransmitClient<{ can::STANDARD_CAN_PACKET_SIZE }>>,
    receive_client:
        OptionalCell<&'static dyn can::ReceiveClient<{ can::STANDARD_CAN_PACKET_SIZE }>>,

    tx_buffer: TakeCell<'static, [u8; can::STANDARD_CAN_PACKET_SIZE]>,
    rx_buffer: TakeCell<'static, [u8; can::STANDARD_CAN_PACKET_SIZE]>,

    /// Shadow copy of the filter elements, laid out exactly as the filter
    /// region of message RAM.
    ///
    /// `setup_message_ram` zeroes all of message RAM (see the D-Cache note
    /// there) and then re-applies this shadow, so filters installed while the
    /// peripheral is disabled survive `enable()`. That in turn lets callers
    /// configure filters in any order relative to `enable`, which the HIL
    /// leaves unspecified.
    filters: [Cell<u32>; FILTER_WORDS],

    deferred_call: DeferredCall,
    deferred_action: OptionalCell<AsyncAction>,
}

impl Mcan {
    pub fn new_mcan1(msg_ram: &'static mut MessageRam) -> Self {
        let base_addr = msg_ram.words.as_ptr() as u32;
        Self {
            regs: unsafe { StaticRef::new(MCAN1_BASE as *const McanRegisters) },
            msg_ram: TakeCell::new(msg_ram),
            msg_ram_base: Cell::new(base_addr),
            state: Cell::new(McanState::Disabled),
            bit_timing: OptionalCell::empty(),
            operating_mode: OptionalCell::empty(),
            automatic_retransmission: Cell::new(false),
            automatic_wake_up: Cell::new(false),
            controller_client: OptionalCell::empty(),
            transmit_client: OptionalCell::empty(),
            receive_client: OptionalCell::empty(),
            tx_buffer: TakeCell::empty(),
            rx_buffer: TakeCell::empty(),
            filters: [const { Cell::new(0) }; FILTER_WORDS],
            deferred_call: DeferredCall::new(),
            deferred_action: OptionalCell::empty(),
        }
    }

    pub fn new_mcan0(msg_ram: &'static mut MessageRam) -> Self {
        let base_addr = msg_ram.words.as_ptr() as u32;
        Self {
            regs: unsafe { StaticRef::new(MCAN0_BASE as *const McanRegisters) },
            msg_ram: TakeCell::new(msg_ram),
            msg_ram_base: Cell::new(base_addr),
            state: Cell::new(McanState::Disabled),
            bit_timing: OptionalCell::empty(),
            operating_mode: OptionalCell::empty(),
            automatic_retransmission: Cell::new(false),
            automatic_wake_up: Cell::new(false),
            controller_client: OptionalCell::empty(),
            transmit_client: OptionalCell::empty(),
            receive_client: OptionalCell::empty(),
            tx_buffer: TakeCell::empty(),
            rx_buffer: TakeCell::empty(),
            filters: [const { Cell::new(0) }; FILTER_WORDS],
            deferred_call: DeferredCall::new(),
            deferred_action: OptionalCell::empty(),
        }
    }

    fn wait_for(times: usize, f: impl Fn() -> bool) -> bool {
        for _ in 0..times {
            if f() {
                return true;
            }
        }
        false
    }

    fn ram_byte_offset(word_index: usize) -> u32 {
        (word_index * 4) as u32
    }

    // True if the Cortex-M7 D-Cache is currently enabled (SCB CCR bit 16).
    // Writing to DCCMVAC / DCIMVAC when the cache is disabled is CONSTRAINED
    // UNPREDICTABLE and can generate IMPRECISERR.  When the cache is off the
    // CPU and MCAN DMA both hit SRAM directly — no coherency maintenance is
    // needed at all.
    fn is_dcache_enabled() -> bool {
        unsafe { core::ptr::read_volatile(0xE000_ED14 as *const u32) & (1 << 16) != 0 }
    }

    // Cortex-M7 D-Cache: clean (flush dirty lines to SRAM).
    fn dcache_clean(addr: usize, len: usize) {
        if !Self::is_dcache_enabled() {
            // Cache is off: writes go straight to SRAM, no flush needed.
            unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }
            return;
        }
        const DCCMVAC: *mut u32 = 0xE000_EF68 as *mut u32;
        // Mask interrupts for the DCCMVAC → DSB window.  An interrupt (e.g.
        // ECC_WARNING) firing between DCCMVAC writes and the final DSB causes
        // exception entry to drain the AXI write buffer while cache write-backs
        // are still in flight; that drain can fail with IMPRECISERR.  PRIMASK
        // prevents preemption until the DSB confirms all write-backs landed.
        unsafe {
            core::arch::asm!("cpsid i", options(nostack, preserves_flags));
            let mut a = addr & !(CACHE_LINE - 1);
            while a < addr + len {
                core::ptr::write_volatile(DCCMVAC, a as u32);
                a += CACHE_LINE;
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("cpsie i", options(nostack, preserves_flags));
        }
    }

    // Cortex-M7 D-Cache: invalidate (discard cached copies).
    fn dcache_invalidate(addr: usize, len: usize) {
        if !Self::is_dcache_enabled() {
            // Cache is off: MCAN DMA writes go straight to SRAM, CPU reads
            // come straight from SRAM — no invalidation needed.
            unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }
            return;
        }
        const DCIMVAC: *mut u32 = 0xE000_EF5C as *mut u32;
        unsafe {
            core::arch::asm!("cpsid i", options(nostack, preserves_flags));
            let mut a = addr & !(CACHE_LINE - 1);
            while a < addr + len {
                core::ptr::write_volatile(DCIMVAC, a as u32);
                a += CACHE_LINE;
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("cpsie i", options(nostack, preserves_flags));
        }
    }

    /// Enter INIT + CCE mode so registers can be configured.
    fn enter_config_mode(&self) -> Result<(), ErrorCode> {
        self.regs.cccr.modify(CCCR::INIT::SET);
        if !Self::wait_for(20_000, || self.regs.cccr.is_set(CCCR::INIT)) {
            return Err(ErrorCode::FAIL);
        }
        self.regs.cccr.modify(CCCR::CCE::SET);
        Ok(())
    }

    /// Leave INIT mode → start normal operation.
    fn leave_config_mode(&self) -> Result<(), ErrorCode> {
        self.regs.cccr.modify(CCCR::CCE::CLEAR);
        self.regs.cccr.modify(CCCR::INIT::CLEAR);
        if !Self::wait_for(20_000, || !self.regs.cccr.is_set(CCCR::INIT)) {
            return Err(ErrorCode::FAIL);
        }
        Ok(())
    }

    /// Full hardware enable sequence: config mode → set timing/mode →
    /// set up message RAM → leave config mode.
    fn hw_enable(&self) -> Result<(), ErrorCode> {
        // Disable the Cortex-M7 default write buffer so any imprecise bus
        // fault becomes precise: the exact faulting PC and BFAR are captured
        // instead of a deferred IMPRECISERR with no address.  Leave this in
        // until the CAN bring-up is stable; it slightly reduces store
        // throughput but is invaluable for diagnosing cache/DMA coherence
        // bugs.  Clear bit 1 of ACTLR (0xE000_E008) to re-enable the buffer.
        unsafe {
            const ACTLR: *mut u32 = 0xE000_E008 as *mut u32;
            let v = core::ptr::read_volatile(ACTLR);
            core::ptr::write_volatile(ACTLR, v | (1 << 1)); // DISDEFWBUF
            // DSB+ISB required after ACTLR write: without ISB the pipeline
            // may not see the new setting before the next instruction executes.
            core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
        }

        self.enter_config_mode()?;

        let timing = self.bit_timing.get().ok_or(ErrorCode::INVAL)?;

        // Nominal bit timing
        self.regs.nbtp.write(
            NBTP::NSJW.val(timing.sync_jump_width)
                + NBTP::NBRP.val(timing.baud_rate_prescaler)
                + NBTP::NTSEG1.val(timing.segment1 as u32)
                + NBTP::NTSEG2.val(timing.segment2 as u32),
        );

        // Classic CAN only — no FD
        self.regs.cccr.modify(CCCR::FDOE::CLEAR + CCCR::BRSE::CLEAR);

        // Automatic retransmission: DAR=1 disables auto-retransmit
        if self.automatic_retransmission.get() {
            self.regs.cccr.modify(CCCR::DAR::CLEAR);
        } else {
            self.regs.cccr.modify(CCCR::DAR::SET);
        }

        // Operation mode
        if let Some(mode) = self.operating_mode.get() {
            match mode {
                can::OperationMode::Loopback => {
                    self.regs.cccr.modify(CCCR::TEST::SET + CCCR::MON::CLEAR);
                    self.regs.test.modify(TEST::LBCK::SET);
                }
                can::OperationMode::Monitoring => {
                    self.regs.cccr.modify(CCCR::MON::SET + CCCR::TEST::CLEAR);
                }
                can::OperationMode::Freeze => {
                    self.regs.cccr.modify(CCCR::CSR::SET);
                }
                can::OperationMode::Normal => {
                    self.regs
                        .cccr
                        .modify(CCCR::MON::CLEAR + CCCR::TEST::CLEAR + CCCR::CSR::CLEAR);
                }
            }
        }

        // Configure message RAM pointers
        self.setup_message_ram();

        // Global filter: reject every frame that no filter element matched,
        // standard and extended alike. Reception is therefore entirely
        // determined by the filters installed through `hil::can::Filter`, and
        // a peripheral with no filters receives nothing.
        //
        // ANFE used to be 0 (accept unmatched extended frames into FIFO 0),
        // which meant 29-bit reception worked by accident: every extended
        // frame on the bus was delivered and clients discarded the unwanted
        // ones in software. That does not scale to several clients sharing the
        // peripheral, and on a busy bus it spends an interrupt per frame.
        self.regs.gfc.write(
            GFC::ANFS.val(2) // Reject non-matching standard frames
                + GFC::ANFE.val(2) // Reject non-matching extended frames
                + GFC::RRFS::CLEAR
                + GFC::RRFE::CLEAR,
        );

        // Enable TX/error interrupts only.  RF0NE/RF0LE (RX FIFO 0) are
        // intentionally omitted here: the MCAN controller goes live on the
        // bus as soon as INIT is cleared, so frames from other nodes arrive
        // immediately — before the app calls CMD_START_RECEIVE and registers
        // a receive buffer.  Enabling RF0NE here would fire handle_rx_fifo0()
        // into an unregistered capsule buffer, causing a write to an invalid
        // address through the write buffer → imprecise bus error → HardFault.
        // RF0NE/RF0LE are enabled in start_receive_process() and disabled in
        // stop_receive() / hw_disable().
        //
        // PEAE fires on ACK errors with DAR=1 (no auto-retransmit): the
        // M_CAN clears TXBRP after one failed attempt but fires neither TC
        // nor TCF.  PEAE is the only way to detect the failed TX and
        // unblock the app's yield_for.  TCFE is kept for software-requested
        // cancellations via TXBCR.
        // BOE/EPE/EWE are NOT enabled — bus error state changes are handled
        // inside handle_interrupt whenever TC or PEAE brings us in.
        self.regs.ie.write(
            IE::TCE::SET
                + IE::TCFE::SET
                + IE::PEAE::SET,
        );

        // All interrupts → line 0
        self.regs.ils.set(0);
        self.regs.ile.write(ILE::EINT0::SET);

        // Per-buffer TX interrupt enables.
        //
        // IR::TC and IR::TCF are each gated by a *per-buffer* enable register;
        // IE::TCE / IE::TCFE above only route an already-set flag to the
        // interrupt line, they do not cause the flag to be set. So both of
        // these are required, not just TXBTIE:
        //
        //   TXBTIE -> IR::TC   (transmission completed successfully)
        //   TXBCIE -> IR::TCF  (transmission cancelled / abandoned)
        //
        // TXBCIE matters because with CCCR.DAR = 1 the M_CAN abandons a frame
        // after a single failed attempt -- including a lost arbitration, which
        // is not a protocol error and so raises neither PEA nor PED. It clears
        // TXBRP and sets TXBCF. Without TXBCIE that produces no interrupt at
        // all, the transmit client is never called back, and the caller just
        // blocks until its own timeout expires.
        self.regs.txbtie.set((1u32 << TX_BUF_COUNT) - 1);
        self.regs.txbcie.set((1u32 << TX_BUF_COUNT) - 1);

        self.leave_config_mode()?;

        self.state.set(McanState::Running);
        Ok(())
    }

    fn setup_message_ram(&self) {
        let base = self.msg_ram_base.get();

        // Standard ID filter configuration
        self.regs.sidfc.write(
            SIDFC::FLSSA.val((base + Self::ram_byte_offset(STD_FILTER_OFFSET)) >> 2)
                + SIDFC::LSS.val(STD_FILTER_COUNT as u32),
        );

        // Extended ID filter configuration
        self.regs.xidfc.write(
            XIDFC::FLESA.val((base + Self::ram_byte_offset(EXT_FILTER_OFFSET)) >> 2)
                + XIDFC::LSE.val(EXT_FILTER_COUNT as u32),
        );

        // RX FIFO 0
        self.regs.rxf0c.write(
            RXF0C::F0SA.val((base + Self::ram_byte_offset(RX_FIFO0_OFFSET)) >> 2)
                + RXF0C::F0S.val(RX_FIFO0_COUNT as u32)
                + RXF0C::F0WM.val(0)
                + RXF0C::F0OM::CLEAR,
        );

        // TX buffers (dedicated, not FIFO/queue)
        self.regs.txbc.write(
            TXBC::TBSA.val((base + Self::ram_byte_offset(TX_BUF_OFFSET)) >> 2)
                + TXBC::NDTB.val(TX_BUF_COUNT as u32)
                + TXBC::TFQS.val(0),
        );

        // Element sizes: 8-byte data field for classic CAN (RBDS/FEDS = 0)
        self.regs.rxesc.set(0);
        self.regs.txesc.set(0);

        // Extended ID AND Mask, applied to every received extended identifier
        // before it is compared against the extended filter elements. All-ones
        // makes it a no-op, which is what the classic identifier + mask
        // filters written by `enable_filter` assume. This is the reset value,
        // but `enable()` may run after a soft reset that left it modified.
        self.regs.xidam.set(EXT_ID_MASK);

        // Zero out the message RAM and immediately flush to physical SRAM.
        // Without the dcache_clean, the D-Cache holds dirty zero-lines over
        // every RX FIFO element.  Later, when handle_rx_fifo0() calls
        // DCIMVAC on one of those elements, Cortex-M7 triggers an internal
        // write-back for the dirty line before discarding it; that write-back
        // drains through the write buffer after the ISR returns to Thread mode
        // and faults as IMPRECISERR → HardFault.  Cleaning here eliminates
        // all dirty lines before MCAN DMA starts, so DCIMVAC only ever sees
        // clean lines (simple invalidation, no write-back required).
        self.msg_ram.map(|ram| {
            for w in ram.words.iter_mut() {
                *w = 0;
            }
            // Re-apply the configured filter elements. The loop above disabled
            // every one of them (SFEC/EFEC = 0), so any filter installed while
            // the peripheral was disabled has to be written back here. The
            // standard and extended filter lists are contiguous by
            // construction, so one pass covers both.
            for (i, filter) in self.filters.iter().enumerate() {
                ram.words[STD_FILTER_OFFSET + i] = filter.get();
            }
        });
        Self::dcache_clean(self.msg_ram_base.get() as usize, MSG_RAM_WORDS * 4);
    }

    fn hw_disable(&self) {
        // Disable all interrupts
        self.regs.ie.set(0);
        self.regs.ile.set(0);

        // Request INIT mode
        self.regs.cccr.modify(CCCR::INIT::SET);
        let _ = Self::wait_for(20_000, || self.regs.cccr.is_set(CCCR::INIT));

        self.state.set(McanState::Disabled);
    }

    /// Find a free TX buffer slot, returns the index.
    fn find_free_tx_buf(&self) -> Option<usize> {
        let pending = self.regs.txbrp.get();
        for i in 0..TX_BUF_COUNT {
            if pending & (1 << i) == 0 {
                return Some(i);
            }
        }
        None
    }

    /// Write a frame into a TX buffer and request transmission.
    fn send_frame(
        &self,
        id: can::Id,
        data: &[u8; can::STANDARD_CAN_PACKET_SIZE],
        len: usize,
    ) -> Result<(), ErrorCode> {
        let buf_idx = self.find_free_tx_buf().ok_or(ErrorCode::BUSY)?;

        let dlc = if len > 8 { 8 } else { len };

        // Build TX element header words
        let (w0, w1);
        match id {
            can::Id::Standard(sid) => {
                // Bits 28:18 = STID, bit 30 = XTD (0), bit 29 = RTR (0)
                w0 = ((sid as u32) & 0x7FF) << 18;
            }
            can::Id::Extended(eid) => {
                // Bits 28:0 = EXTID, bit 30 = XTD (1)
                w0 = (eid & 0x1FFF_FFFF) | (1 << 30);
            }
        }
        // W1: bits 19:16 = DLC, bit 21 = FDF (0 for classic), bit 20 = BRS (0)
        w1 = (dlc as u32) << 16;

        let elem_base = TX_BUF_OFFSET + buf_idx * CAN_ELEMENT_WORDS;

        self.msg_ram.map(|ram| {
            ram.words[elem_base] = w0;
            ram.words[elem_base + 1] = w1;

            // Pack data bytes into words 2 and 3.
            // SAMV71 MCAN uses little-endian message RAM: data[0] at
            // bits [7:0] of the first data word.
            let mut d0: u32 = 0;
            let mut d1: u32 = 0;
            for i in 0..dlc {
                if i < 4 {
                    d0 |= (data[i] as u32) << (i * 8);
                } else {
                    d1 |= (data[i] as u32) << ((i - 4) * 8);
                }
            }
            ram.words[elem_base + 2] = d0;
            ram.words[elem_base + 3] = d1;
        });

        // Clean D-Cache for the TX element so the MCAN DMA reads
        // actual data from SRAM, not stale cache lines.
        Self::dcache_clean(
            self.msg_ram_base.get() as usize + elem_base * 4,
            CAN_ELEMENT_WORDS * 4,
        );

        // Request transmission
        self.regs.txbar.set(1 << buf_idx);

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Interrupt handlers (called from chip.rs InterruptService)
    // -----------------------------------------------------------------------

    pub fn handle_interrupt(&self) {
        let ir = self.regs.ir.extract();

        // TX Complete
        if ir.is_set(IR::TC) {
            self.regs.ir.write(IR::TC::SET);
            self.handle_tx_complete();
        }

        // TX Cancellation Finished: the frame was abandoned rather than sent.
        // Reached either via a software cancellation (TXBCR) or, when
        // CCCR.DAR = 1, after a single failed transmission attempt -- most
        // often a lost arbitration, which raises no protocol error. Requires
        // TXBCIE to be set in hw_enable(), otherwise IR::TCF never sets and
        // the failure is silent.
        if ir.is_set(IR::TCF) {
            self.regs.ir.write(IR::TCF::SET);
            self.transmit_client.map(|client| {
                if let Some(buf) = self.tx_buffer.take() {
                    client.transmit_complete(Err(can::Error::SetBySoftware), buf);
                }
            });
        }

        // RX FIFO 0 new message
        if ir.is_set(IR::RF0N) {
            self.regs.ir.write(IR::RF0N::SET);

            // Drain every queued element, not just one.
            //
            // RF0N latches when a message is written to the FIFO, so handling
            // a single element per interrupt strands anything else already
            // queued until the *next* frame happens to arrive. That went
            // unnoticed while GFC accepted all unmatched extended frames,
            // because bus traffic kept re-triggering the interrupt and
            // flushing the backlog within microseconds. Now that the filters
            // reject everything unwanted, a stranded element can sit in the
            // FIFO indefinitely -- and back-to-back segmented traffic
            // (ISO-TP consecutive frames arrive roughly every 260 us at
            // 500 kbit/s) is exactly the case that queues several at once.
            //
            // RF0N is cleared *before* draining, so a frame arriving during
            // the loop re-latches it and we get a fresh interrupt rather than
            // losing it. The bound guarantees termination: the FIFO holds at
            // most RX_FIFO0_COUNT elements, so anything still queued
            // afterwards necessarily arrived after the clear above.
            for _ in 0..RX_FIFO0_COUNT {
                if !self.handle_rx_fifo0() {
                    break;
                }
            }
        }

        // RX FIFO 0 message lost
        if ir.is_set(IR::RF0L) {
            self.regs.ir.write(IR::RF0L::SET);
        }

        // Bus-off
        if ir.is_set(IR::BO_) {
            self.regs.ir.write(IR::BO_::SET);
            self.state.set(McanState::Error(can::Error::BusOff));
            self.controller_client.map(|c| {
                c.state_changed(can::State::Error(can::Error::BusOff));
            });
        }

        // Error passive
        if ir.is_set(IR::EP_) {
            self.regs.ir.write(IR::EP_::SET);
            self.state.set(McanState::Error(can::Error::Passive));
            self.controller_client.map(|c| {
                c.state_changed(can::State::Error(can::Error::Passive));
            });
        }

        // Error warning
        if ir.is_set(IR::EW_) {
            self.regs.ir.write(IR::EW_::SET);
            self.state.set(McanState::Error(can::Error::Warning));
            self.controller_client.map(|c| {
                c.state_changed(can::State::Error(can::Error::Warning));
            });
        }

        // Protocol error (arbitration / data phase)
        // PEAE is enabled so that ACK errors with DAR=1 reach us: the M_CAN
        // clears TXBRP after one failed attempt without setting TC or TCF,
        // so this is the only interrupt that fires for a no-ACK TX.
        if ir.is_set(IR::PEA) || ir.is_set(IR::PED) {
            self.regs.ir.write(IR::PEA::SET + IR::PED::SET);
            let lec = self.regs.psr.read(PSR::LEC);
            let err = match lec {
                1 => can::Error::Stuff,
                2 => can::Error::Form,
                3 => can::Error::Ack,
                4 => can::Error::BitRecessive,
                5 => can::Error::BitDominant,
                6 => can::Error::Crc,
                _ => can::Error::SetBySoftware,
            };
            self.state.set(McanState::Error(err));
            // Unblock any pending TX: DAR=1 already cleared TXBRP so there
            // will be no TC interrupt.  Return the buffer now so the app's
            // yield_for can complete.
            self.transmit_client.map(|client| {
                if let Some(buf) = self.tx_buffer.take() {
                    client.transmit_complete(Err(err), buf);
                }
            });
        }
    }

    fn handle_tx_complete(&self) {
        // `IR::TC` is the controller saying *this* frame was transmitted and
        // acknowledged. It succeeded, whatever went wrong before it.
        //
        // This used to report `self.state` instead, which is latched: one
        // unacknowledged frame -- transmitting onto a bus with nothing else on
        // it, which is the normal bench case -- left `Error(Ack)` set forever,
        // and every later transmission was reported as failed even though it
        // went out fine. With one application that is invisible, because the
        // application that poisoned the state is the one that gives up. With
        // two it is not: a second process inherits a transmit path that can
        // never report success again, and `enable()` (which refuses in the
        // error state) can never succeed again either.
        //
        // The latched protocol error is dropped here for the same reason. Real
        // controller states are left alone: they have their own interrupts,
        // and one good frame does not mean the controller has left them.
        if let McanState::Error(e) = self.state.get() {
            if !is_controller_state(e) {
                self.state.set(McanState::Running);
            }
        }

        self.transmit_client.map(|client| {
            if let Some(buf) = self.tx_buffer.take() {
                client.transmit_complete(Ok(()), buf);
            }
        });
    }

    /// Process a single element from RX FIFO 0.
    ///
    /// Returns `true` if an element was consumed, so the caller can keep
    /// draining until the FIFO reports empty.
    fn handle_rx_fifo0(&self) -> bool {
        let f0s = self.regs.rxf0s.extract();
        let fill = f0s.read(RXF0S::F0FL);
        if fill == 0 {
            return false;
        }

        let get_idx = f0s.read(RXF0S::F0GI) as usize;
        let elem_base = RX_FIFO0_OFFSET + get_idx * CAN_ELEMENT_WORDS;
        let inv_addr = self.msg_ram_base.get() as usize + elem_base * 4;

        // Invalidate D-Cache for the RX element so the CPU reads
        // fresh data written by the MCAN DMA, not stale cache lines.
        // Precondition: setup_message_ram() must have called dcache_clean
        // after zeroing so no dirty lines exist here (dirty lines + DCIMVAC
        // triggers an internal write-back through the write buffer that can
        // fault as IMPRECISERR when it drains after ISR return).
        Self::dcache_invalidate(inv_addr, CAN_ELEMENT_WORDS * 4);

        let mut frame_id = can::Id::Standard(0);
        let mut frame_data = [0u8; can::STANDARD_CAN_PACKET_SIZE];
        let mut frame_len: usize = 0;

        self.msg_ram.map(|ram| {
            let w0 = ram.words[elem_base];
            let w1 = ram.words[elem_base + 1];
            let d0 = ram.words[elem_base + 2];
            let d1 = ram.words[elem_base + 3];

            let xtd = (w0 >> 30) & 1;
            if xtd == 0 {
                frame_id = can::Id::Standard(((w0 >> 18) & 0x7FF) as u16);
            } else {
                frame_id = can::Id::Extended(w0 & 0x1FFF_FFFF);
            }

            let dlc = ((w1 >> 16) & 0xF) as usize;
            frame_len = if dlc > 8 { 8 } else { dlc };

            // Unpack data (little-endian: byte 0 at bits [7:0])
            for i in 0..frame_len {
                if i < 4 {
                    frame_data[i] = ((d0 >> (i * 8)) & 0xFF) as u8;
                } else {
                    frame_data[i] = ((d1 >> ((i - 4) * 8)) & 0xFF) as u8;
                }
            }
        });

        // Acknowledge the element so the FIFO advances
        self.regs
            .rxf0a
            .write(RXF0A::F0AI.val(get_idx as u32));

        self.receive_client.map(|client| {
            client.message_received(frame_id, &mut frame_data, frame_len, Ok(()));
        });

        true
    }
}

// ---------------------------------------------------------------------------
// DeferredCallClient — async enable/disable/abort callbacks
// ---------------------------------------------------------------------------

impl DeferredCallClient for Mcan {
    fn register(&'static self) {
        self.deferred_call.register(self);
    }

    fn handle_deferred_call(&self) {
        if let Some(action) = self.deferred_action.take() {
            match action {
                AsyncAction::Enable => {
                    self.controller_client.map(|c| {
                        c.state_changed(can::State::Running);
                        c.enabled(Ok(()));
                    });
                }
                AsyncAction::EnableError(err) => {
                    self.controller_client.map(|c| {
                        c.state_changed(self.state.get().into());
                        c.enabled(Err(err));
                    });
                }
                AsyncAction::Disable => {
                    self.controller_client.map(|c| {
                        c.state_changed(can::State::Disabled);
                        c.disabled(Ok(()));
                    });
                }
                AsyncAction::AbortReceive => {
                    if let Some(rx) = self.rx_buffer.take() {
                        self.receive_client.map(|c| c.stopped(rx));
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// hil::can::Configure
// ---------------------------------------------------------------------------

impl can::Configure for Mcan {
    // MCAN NBTP field ranges (register values, 0-indexed)
    const MIN_BIT_TIMINGS: can::BitTiming = can::BitTiming {
        segment1: 1,
        segment2: 1,
        propagation: 0,
        sync_jump_width: 1,
        baud_rate_prescaler: 1,
    };

    const MAX_BIT_TIMINGS: can::BitTiming = can::BitTiming {
        segment1: 255,
        segment2: 127,
        propagation: 0,
        sync_jump_width: 127,
        baud_rate_prescaler: 511,
    };

    const SYNC_SEG: u8 = 1;

    fn set_bitrate(&self, bitrate: u32) -> Result<(), ErrorCode> {
        let bt = Self::bit_timing_for_bitrate(CAN_CLK_HZ, bitrate)?;
        self.set_bit_timing(bt)
    }

    fn set_bit_timing(&self, bit_timing: can::BitTiming) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Disabled => {
                self.bit_timing.set(bit_timing);
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    fn set_operation_mode(&self, mode: can::OperationMode) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Disabled => {
                self.operating_mode.set(mode);
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    fn get_bit_timing(&self) -> Result<can::BitTiming, ErrorCode> {
        self.bit_timing.get().ok_or(ErrorCode::INVAL)
    }

    fn get_operation_mode(&self) -> Result<can::OperationMode, ErrorCode> {
        self.operating_mode.get().ok_or(ErrorCode::INVAL)
    }

    fn set_automatic_retransmission(&self, automatic: bool) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Disabled => {
                self.automatic_retransmission.set(automatic);
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    fn set_wake_up(&self, wake_up: bool) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Disabled => {
                self.automatic_wake_up.set(wake_up);
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    fn get_automatic_retransmission(&self) -> Result<bool, ErrorCode> {
        Ok(self.automatic_retransmission.get())
    }

    fn get_wake_up(&self) -> Result<bool, ErrorCode> {
        Ok(self.automatic_wake_up.get())
    }

    fn receive_fifo_count(&self) -> usize {
        1
    }
}

// ---------------------------------------------------------------------------
// hil::can::Controller
// ---------------------------------------------------------------------------

impl can::Controller for Mcan {
    fn set_client(&self, client: Option<&'static dyn can::ControllerClient>) {
        if let Some(c) = client {
            self.controller_client.replace(c);
        } else {
            self.controller_client.clear();
        }
    }

    fn enable(&self) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Disabled => {
                if self.bit_timing.is_none() || self.operating_mode.is_none() {
                    return Err(ErrorCode::INVAL);
                }
                if self.deferred_action.is_some() {
                    return Err(ErrorCode::BUSY);
                }

                match self.hw_enable() {
                    Ok(()) => {
                        self.deferred_action.set(AsyncAction::Enable);
                    }
                    Err(e) => {
                        self.deferred_action.set(AsyncAction::EnableError(e));
                    }
                }
                self.deferred_call.set();
                Ok(())
            }
            McanState::Running => Err(ErrorCode::ALREADY),
            McanState::Error(_) => Err(ErrorCode::FAIL),
        }
    }

    fn disable(&self) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Running | McanState::Error(_) => {
                self.hw_disable();
                if self.deferred_action.is_some() {
                    return Err(ErrorCode::BUSY);
                }
                self.deferred_action.set(AsyncAction::Disable);
                self.deferred_call.set();
                Ok(())
            }
            McanState::Disabled => Err(ErrorCode::OFF),
        }
    }

    fn get_state(&self) -> Result<can::State, ErrorCode> {
        Ok(self.state.get().into())
    }
}

// ---------------------------------------------------------------------------
// hil::can::Transmit<8>
// ---------------------------------------------------------------------------

impl can::Transmit<{ can::STANDARD_CAN_PACKET_SIZE }> for Mcan {
    fn set_client(
        &self,
        client: Option<&'static dyn can::TransmitClient<{ can::STANDARD_CAN_PACKET_SIZE }>>,
    ) {
        if let Some(c) = client {
            self.transmit_client.set(c);
        } else {
            self.transmit_client.clear();
        }
    }

    fn send(
        &self,
        id: can::Id,
        buffer: &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE],
        len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE])> {
        match self.state.get() {
            McanState::Running | McanState::Error(_) => {
                // Copy data from the buffer before storing ownership
                match self.send_frame(id, buffer, len) {
                    Ok(()) => {
                        // Store buffer — returned via transmit_complete callback
                        self.tx_buffer.replace(buffer);
                        Ok(())
                    }
                    Err(e) => Err((e, buffer)),
                }
            }
            McanState::Disabled => Err((ErrorCode::OFF, buffer)),
        }
    }
}

// ---------------------------------------------------------------------------
// hil::can::Receive<8>
// ---------------------------------------------------------------------------

impl can::Receive<{ can::STANDARD_CAN_PACKET_SIZE }> for Mcan {
    fn set_client(
        &self,
        client: Option<&'static dyn can::ReceiveClient<{ can::STANDARD_CAN_PACKET_SIZE }>>,
    ) {
        if let Some(c) = client {
            self.receive_client.set(c);
        } else {
            self.receive_client.clear();
        }
    }

    fn start_receive_process(
        &self,
        buffer: &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE],
    ) -> Result<(), (ErrorCode, &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE])> {
        match self.state.get() {
            McanState::Running | McanState::Error(_) => {
                self.rx_buffer.put(Some(buffer));
                // Enable FIFO 0 interrupts now that a buffer is registered.
                // Any frames that arrived between hw_enable() and here will
                // have set IR::RF0N already; enabling RF0NE causes an
                // immediate interrupt that drains the accumulated frames.
                self.regs.ie.modify(IE::RF0NE::SET + IE::RF0LE::SET);
                Ok(())
            }
            McanState::Disabled => Err((ErrorCode::OFF, buffer)),
        }
    }

    fn stop_receive(&self) -> Result<(), ErrorCode> {
        match self.state.get() {
            McanState::Running | McanState::Error(_) => {
                if self.deferred_action.is_some() {
                    return Err(ErrorCode::BUSY);
                }
                if self.rx_buffer.is_none() {
                    return Err(ErrorCode::ALREADY);
                }
                // Disable FIFO 0 interrupts before returning the buffer so
                // no in-flight RF0N can race with the deferred callback.
                self.regs.ie.modify(IE::RF0NE::CLEAR + IE::RF0LE::CLEAR);
                self.deferred_action.set(AsyncAction::AbortReceive);
                self.deferred_call.set();
                Ok(())
            }
            McanState::Disabled => Err(ErrorCode::OFF),
        }
    }
}

// ---------------------------------------------------------------------------
// hil::can::Filter
// ---------------------------------------------------------------------------

impl Mcan {
    /// Flush the filter region of message RAM out of the D-Cache so the MCAN's
    /// AHB master sees the current filter elements.
    fn clean_filter_region(&self) {
        Self::dcache_clean(
            self.msg_ram_base.get() as usize + Self::ram_byte_offset(STD_FILTER_OFFSET) as usize,
            FILTER_WORDS * 4,
        );
    }

    /// Install a standard (11-bit) filter element.
    ///
    /// A standard element is a single word, so the update is atomic from the
    /// MCAN's point of view and needs no disable/re-enable sequence.
    fn write_std_filter(&self, index: usize, word: u32) {
        self.filters[index].set(word);
        self.msg_ram.map(|ram| {
            ram.words[STD_FILTER_OFFSET + index] = word;
        });
        self.clean_filter_region();
    }

    /// Install an extended (29-bit) filter element.
    ///
    /// An extended element spans two words and the MCAN re-reads it from
    /// message RAM for every received frame, so the element is disabled
    /// (EFEC = 0) and flushed before EFID2 changes, then re-enabled once both
    /// words are in place. A frame arriving mid-update therefore matches
    /// either the old filter or the new one, never a mix of the two.
    fn write_ext_filter(&self, index: usize, w0: u32, w1: u32) {
        let shadow = STD_FILTER_WORDS + index * 2;
        let word = EXT_FILTER_OFFSET + index * 2;

        self.filters[shadow].set(w0);
        self.filters[shadow + 1].set(w1);

        self.msg_ram.map(|ram| {
            ram.words[word] = FILTER_CONFIG_DISABLED;
        });
        self.clean_filter_region();

        self.msg_ram.map(|ram| {
            ram.words[word + 1] = w1;
            ram.words[word] = w0;
        });
        self.clean_filter_region();
    }
}

/// Filter numbering is a single flat space across both hardware filter lists:
/// `0..STD_FILTER_COUNT` address the standard (11-bit) elements and the
/// following `EXT_FILTER_COUNT` numbers address the extended (29-bit) ones.
/// `enable_filter` rejects a number whose list does not match the variant of
/// [`can::Id`] supplied with it.
///
/// Unlike the HIL's note on [`can::Receive::start_receive_process`], filters
/// may be installed at any time, before or after `enable()`. Elements are
/// shadowed in the driver and re-applied whenever message RAM is set up.
impl can::Filter for Mcan {
    fn enable_filter(&self, filter: can::FilterParameters) -> Result<(), ErrorCode> {
        // Only RX FIFO 0 is configured; see `setup_message_ram`.
        if filter.fifo_number != 0 {
            return Err(ErrorCode::INVAL);
        }

        let number = filter.number as usize;

        match filter.id {
            can::Id::Standard(id) => {
                if number >= STD_FILTER_COUNT {
                    return Err(ErrorCode::INVAL);
                }
                let mask = match filter.identifier_mode {
                    can::IdentifierMode::List => STD_ID_MASK,
                    can::IdentifierMode::Mask => filter.mask & STD_ID_MASK,
                };
                self.write_std_filter(
                    number,
                    (FILTER_TYPE_CLASSIC << 30)
                        | (FILTER_CONFIG_FIFO0 << 27)
                        | ((u32::from(id) & STD_ID_MASK) << 16)
                        | mask,
                );
                Ok(())
            }
            can::Id::Extended(id) => {
                if number < STD_FILTER_COUNT || number >= STD_FILTER_COUNT + EXT_FILTER_COUNT {
                    return Err(ErrorCode::INVAL);
                }
                let mask = match filter.identifier_mode {
                    can::IdentifierMode::List => EXT_ID_MASK,
                    can::IdentifierMode::Mask => filter.mask & EXT_ID_MASK,
                };
                self.write_ext_filter(
                    number - STD_FILTER_COUNT,
                    (FILTER_CONFIG_FIFO0 << 29) | (id & EXT_ID_MASK),
                    (FILTER_TYPE_CLASSIC << 30) | mask,
                );
                Ok(())
            }
        }
    }

    fn disable_filter(&self, number: u32) -> Result<(), ErrorCode> {
        let number = number as usize;
        if number < STD_FILTER_COUNT {
            self.write_std_filter(number, FILTER_CONFIG_DISABLED);
            Ok(())
        } else if number < STD_FILTER_COUNT + EXT_FILTER_COUNT {
            self.write_ext_filter(
                number - STD_FILTER_COUNT,
                FILTER_CONFIG_DISABLED,
                FILTER_CONFIG_DISABLED,
            );
            Ok(())
        } else {
            Err(ErrorCode::INVAL)
        }
    }

    fn filter_count(&self) -> usize {
        STD_FILTER_COUNT + EXT_FILTER_COUNT
    }
}
