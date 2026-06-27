// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Power Management Controller (PMC) for SAMV71Q21B.
//!
//! Configures clocks for the SAMV71 Xplained Ultra evaluation board:
//!   - Attempts external 12 MHz crystal → PLLA ×25 = 300 MHz (PCK)
//!   - Falls back to internal RC if crystal does not start
//!   - MCK (master/peripheral clock) = PCK / 2 = 150 MHz  (MDIV = PCK_DIV2)
//!
//! UART baud rate at MCK = 150 MHz:
//!   CD = 150_000_000 / (16 × 115_200) = 81  →  actual 115_740 baud (0.47% error)
//!
//! Register map: SAMV71Q21B datasheet Table 31-3, base address 0x400E_0600.

use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly,
};
use kernel::utilities::StaticRef;

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

register_structs! {
    PmcRegisters {
        (0x000 => pmc_scer:  WriteOnly<u32, PmcScer::Register>),
        (0x004 => pmc_scdr:  WriteOnly<u32, PmcScdr::Register>),
        (0x008 => pmc_scsr:  ReadOnly<u32>),
        (0x00C => _reserved0),
        (0x010 => pmc_pcer0: WriteOnly<u32, PmcPcx::Register>),
        (0x014 => pmc_pcdr0: WriteOnly<u32, PmcPcx::Register>),
        (0x018 => pmc_pcsr0: ReadOnly<u32,  PmcPcx::Register>),
        (0x01C => ckgr_uckr: ReadWrite<u32, CkgrUckr::Register>),
        (0x020 => ckgr_mor:  ReadWrite<u32, CkgrMor::Register>),
        (0x024 => ckgr_mcfr: ReadOnly<u32,  CkgrMcfr::Register>),
        (0x028 => ckgr_pllar:ReadWrite<u32, CkgrPllar::Register>),
        (0x02C => _reserved1),
        (0x030 => pmc_mckr:  ReadWrite<u32, PmcMckr::Register>),
        (0x034 => _reserved2),
        (0x038 => pmc_usb:   ReadWrite<u32, PmcUsb::Register>),
        (0x03C => _reserved3),
        (0x040 => pmc_pck:   [ReadWrite<u32, PmcPck::Register>; 8]),
        (0x060 => pmc_ier:   WriteOnly<u32, PmcSr::Register>),
        (0x064 => pmc_idr:   WriteOnly<u32, PmcSr::Register>),
        (0x068 => pmc_sr:    ReadOnly<u32,  PmcSr::Register>),
        (0x06C => pmc_imr:   ReadOnly<u32,  PmcSr::Register>),
        (0x070 => pmc_fsmr:  ReadWrite<u32>),
        (0x074 => pmc_fspr:  ReadWrite<u32>),
        (0x078 => pmc_focr:  WriteOnly<u32>),
        (0x07C => _reserved4),
        (0x0E4 => pmc_wpmr:  ReadWrite<u32, PmcWpmr::Register>),
        (0x0E8 => pmc_wpsr:  ReadOnly<u32>),
        (0x0EC => _reserved5),
        (0x100 => pmc_pcer1: WriteOnly<u32, PmcPcx::Register>),
        (0x104 => pmc_pcdr1: WriteOnly<u32, PmcPcx::Register>),
        (0x108 => pmc_pcsr1: ReadOnly<u32,  PmcPcx::Register>),
        (0x10C => pmc_pcr:   ReadWrite<u32>),
        (0x110 => pmc_ocr:   ReadWrite<u32>),
        (0x114 => pmc_slpwk_er:  WriteOnly<u32>),
        (0x118 => pmc_slpwk_dr:  WriteOnly<u32>),
        (0x11C => pmc_slpwk_sr:  ReadOnly<u32>),
        (0x120 => pmc_slpwk_asr: ReadOnly<u32>),
        (0x124 => _reserved6),
        (0x128 => pmc_pmmr:  ReadWrite<u32>),
        (0x12C => @END),
    }
}

// ---------------------------------------------------------------------------
// Bitfields
// ---------------------------------------------------------------------------

register_bitfields![u32,
    CkgrMor [
        /// 0 = internal RC, 1 = external crystal. Bit 24.
        MOSCSEL  OFFSET(24) NUMBITS(1) [],
        /// Must be 0x37 on every write to this register. Bits 23:16.
        KEY      OFFSET(16) NUMBITS(8) [],
        /// Crystal oscillator startup time: (MOSCXTST + 1) × 8 slow-clock cycles.
        MOSCXTST OFFSET(8)  NUMBITS(8) [],
        MOSCRCF  OFFSET(4)  NUMBITS(3) [
            Mhz4  = 0,
            Mhz8  = 1,
            Mhz12 = 2
        ],
        MOSCRCEN OFFSET(3) NUMBITS(1) [],
        /// Crystal oscillator bypass (use external clock input instead of crystal).
        MOSCXTBY OFFSET(1) NUMBITS(1) [],
        /// Main crystal oscillator enable. Bit 0.
        MOSCXTEN OFFSET(0) NUMBITS(1) [],
    ],

    CkgrMcfr [
        MAINFRDY OFFSET(16) NUMBITS(1) [],
        MAINF    OFFSET(0)  NUMBITS(16) []
    ],

    CkgrPllar [
        /// Must be 1.
        ONE       OFFSET(29) NUMBITS(1) [],
        /// Multiplier minus 1. Set to 24 for ×25 (300 MHz from 12 MHz).
        MULA      OFFSET(16) NUMBITS(11) [],
        /// Lock time in slow-clock cycles. Use 0x3F (maximum) for safety.
        PLLACOUNT OFFSET(8)  NUMBITS(6) [],
        /// Divider. Set to 1 (bypass divider).
        DIVA      OFFSET(0)  NUMBITS(8) []
    ],

    CkgrUckr [
        UPLLCOUNT OFFSET(24) NUMBITS(4) [],
        BIASEN    OFFSET(20) NUMBITS(1) [],
        UPLLEN    OFFSET(16) NUMBITS(1) []
    ],

    PmcMckr [
        UPLLDIV2 OFFSET(13) NUMBITS(1) [],
        /// Master clock divider (applied after PRES to produce MCK from PCK).
        MDIV OFFSET(8) NUMBITS(2) [
            Div1 = 0,   // MCK = PCK
            Div2 = 1,   // MCK = PCK/2  → 150 MHz when PCK = 300 MHz
            Div4 = 2,   // MCK = PCK/4
            Div3 = 3    // MCK = PCK/3
        ],
        /// Processor clock prescaler (applied to the selected source).
        PRES OFFSET(4) NUMBITS(3) [
            Div1  = 0,
            Div2  = 1,
            Div4  = 2,
            Div8  = 3,
            Div16 = 4,
            Div32 = 5,
            Div64 = 6,
            Div3  = 7
        ],
        CSS OFFSET(0) NUMBITS(2) [
            Slow = 0,
            Main = 1,
            Plla = 2,
            Upll = 3
        ]
    ],

    PmcUsb [
        USBDIV OFFSET(8) NUMBITS(4) [],
        USBS   OFFSET(0) NUMBITS(1) [
            Plla = 0,
            Upll = 1
        ]
    ],

    PmcScer [
        PCK7   OFFSET(15) NUMBITS(1) [],
        PCK6   OFFSET(14) NUMBITS(1) [],
        PCK5   OFFSET(13) NUMBITS(1) [],
        PCK4   OFFSET(12) NUMBITS(1) [],
        PCK3   OFFSET(11) NUMBITS(1) [],
        PCK2   OFFSET(10) NUMBITS(1) [],
        PCK1   OFFSET(9)  NUMBITS(1) [],
        PCK0   OFFSET(8)  NUMBITS(1) [],
        USBCLK OFFSET(5)  NUMBITS(1) []
    ],

    PmcScdr [
        PCK7   OFFSET(15) NUMBITS(1) [],
        PCK6   OFFSET(14) NUMBITS(1) [],
        PCK5   OFFSET(13) NUMBITS(1) [],
        PCK4   OFFSET(12) NUMBITS(1) [],
        PCK3   OFFSET(11) NUMBITS(1) [],
        PCK2   OFFSET(10) NUMBITS(1) [],
        PCK1   OFFSET(9)  NUMBITS(1) [],
        PCK0   OFFSET(8)  NUMBITS(1) [],
        USBCLK OFFSET(5)  NUMBITS(1) []
    ],

    PmcPck [
        PRES OFFSET(4) NUMBITS(8) [],
        CSS  OFFSET(0) NUMBITS(3) []
    ],

    PmcSr [
        XT32KERR OFFSET(21) NUMBITS(1) [],
        CFDS     OFFSET(19) NUMBITS(1) [],
        CFDEV    OFFSET(18) NUMBITS(1) [],
        MOSCRCS  OFFSET(17) NUMBITS(1) [],
        MOSCSELS OFFSET(16) NUMBITS(1) [],
        PCKRDY7  OFFSET(15) NUMBITS(1) [],
        PCKRDY6  OFFSET(14) NUMBITS(1) [],
        PCKRDY5  OFFSET(13) NUMBITS(1) [],
        PCKRDY4  OFFSET(12) NUMBITS(1) [],
        PCKRDY3  OFFSET(11) NUMBITS(1) [],
        PCKRDY2  OFFSET(10) NUMBITS(1) [],
        PCKRDY1  OFFSET(9)  NUMBITS(1) [],
        PCKRDY0  OFFSET(8)  NUMBITS(1) [],
        LOCKU    OFFSET(6)  NUMBITS(1) [],
        MCKRDY   OFFSET(3)  NUMBITS(1) [],
        LOCKA    OFFSET(1)  NUMBITS(1) [],
        MOSCXTS  OFFSET(0)  NUMBITS(1) []
    ],

    PmcWpmr [
        /// Write "PMC" (0x504D43) to unlock; any other value re-locks.
        WPKEY OFFSET(8) NUMBITS(24) [],
        WPEN  OFFSET(0) NUMBITS(1)  []
    ],

    /// Peripheral clock enable/disable/status: one bit per peripheral ID.
    PmcPcx [
        PID OFFSET(0) NUMBITS(32) []
    ]
];

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

const PMC_BASE: StaticRef<PmcRegisters> =
    unsafe { StaticRef::new(0x400E_0600 as *const PmcRegisters) };

const WP_KEY: u32 = 0x504D43; // ASCII "PMC"
const MOR_KEY: u32 = 0x37;

pub static mut PMC: Pmc = Pmc::new();

pub struct Pmc {
    regs: StaticRef<PmcRegisters>,
}

impl Pmc {
    pub const fn new() -> Pmc {
        Pmc { regs: PMC_BASE }
    }

    fn disable_write_protection(&self) {
        self.regs
            .pmc_wpmr
            .write(PmcWpmr::WPKEY.val(WP_KEY) + PmcWpmr::WPEN::CLEAR);
    }

    fn enable_write_protection(&self) {
        self.regs
            .pmc_wpmr
            .write(PmcWpmr::WPKEY.val(WP_KEY) + PmcWpmr::WPEN::SET);
    }

    /// Configure clocks for the SAMV71 Xplained Ultra board.
    ///
    /// Tries the external 12 MHz crystal first. If MOSCXTS does not
    /// go high within the timeout, falls back to the internal RC
    /// oscillator and adapts the PLLA multiplier to whatever MOSCRCF
    /// frequency is currently selected.
    ///
    /// After this call:
    ///   - PCK (Cortex-M7 core) = 300 MHz  (296 MHz in 8 MHz RC fallback)
    ///   - MCK (peripheral bus)  = 150 MHz  (148 MHz in 8 MHz RC fallback)
    ///
    /// UART baud rate at MCK ≈ 150 MHz: CD = 81.
    pub fn setup_clocks(&self) {
        self.disable_write_protection();

        // Park MCK on MAINCK (the internal RC) while reconfiguring PLLA.
        // MAINCK runs at MHz speeds, keeping the bus fast enough for
        // reliable register access (unlike the 32 kHz slow clock).
        self.regs.pmc_mckr.modify(PmcMckr::CSS::Main);
        while !self.regs.pmc_sr.is_set(PmcSr::MCKRDY) {}

        // ---------------------------------------------------------------
        // Attempt external 12 MHz crystal startup.
        //
        // Enable the crystal oscillator with maximum startup time:
        // MOSCXTST=255 → (255+1) × 8 slow-clock cycles ≈ 62 ms.
        // Keep MOSCRCEN=1 so the internal RC continues running as
        // the active MAINCK source while the crystal stabilises.
        // ---------------------------------------------------------------
        self.regs.ckgr_mor.modify(
            CkgrMor::KEY.val(MOR_KEY)
                + CkgrMor::MOSCXTEN::SET
                + CkgrMor::MOSCXTST.val(255),
        );

        // Poll MOSCXTS with a timeout. At the slowest RC (4 MHz) and
        // ~10 cycles per iteration, 500 000 iterations ≈ 1.25 s — well
        // beyond the 62 ms hardware startup window.
        let mut crystal_ok = false;
        for _ in 0..500_000u32 {
            if self.regs.pmc_sr.is_set(PmcSr::MOSCXTS) {
                crystal_ok = true;
                break;
            }
        }

        let mula;
        if crystal_ok {
            // Switch MAINCK to the external crystal.
            self.regs.ckgr_mor.modify(
                CkgrMor::KEY.val(MOR_KEY) + CkgrMor::MOSCSEL::SET,
            );
            while !self.regs.pmc_sr.is_set(PmcSr::MOSCSELS) {}

            // 12 MHz × 25 = 300 MHz — deterministic, no MOSCRCF ambiguity.
            mula = 24;
        } else {
            // Crystal did not start — fall back to internal RC.
            //
            // CKGR_MOR survives software resets (only POR clears it), so
            // the RC frequency may be 4, 8, or 12 MHz depending on what
            // a prior boot left behind. Pick a PLLA multiplier that
            // produces ~300 MHz from the current RC frequency.
            let moscrcf = self.regs.ckgr_mor.read(CkgrMor::MOSCRCF);
            mula = match moscrcf {
                0 => 74, // 4 MHz × 75 = 300 MHz
                1 => 36, // 8 MHz × 37 = 296 MHz
                _ => 24, // 12 MHz × 25 = 300 MHz
            };
        }

        self.regs.ckgr_pllar.write(
            CkgrPllar::ONE::SET
                + CkgrPllar::MULA.val(mula)
                + CkgrPllar::PLLACOUNT.val(0x3F)
                + CkgrPllar::DIVA.val(1),
        );
        while !self.regs.pmc_sr.is_set(PmcSr::LOCKA) {}

        // Set MDIV *before* CSS so MCK never briefly hits 300 MHz
        // (which exceeds the 150 MHz MCK maximum).
        self.regs.pmc_mckr.modify(PmcMckr::MDIV::Div2);
        while !self.regs.pmc_sr.is_set(PmcSr::MCKRDY) {}

        self.regs.pmc_mckr.modify(PmcMckr::CSS::Plla);
        while !self.regs.pmc_sr.is_set(PmcSr::MCKRDY) {}

        self.enable_write_protection();
    }

    /// Enable the peripheral clock for a peripheral identified by its ID.
    ///
    /// Peripheral IDs match the NVIC interrupt numbers in nvic.rs.
    /// IDs 0–31 are in PCER0; IDs 32–63 are in PCER1.
    pub fn enable_peripheral_clock(&self, pid: u32) {
        self.disable_write_protection();
        if pid < 32 {
            self.regs.pmc_pcer0.set(1 << pid);
        } else if pid < 64 {
            self.regs.pmc_pcer1.set(1 << (pid - 32));
        }
        self.enable_write_protection();
    }

    /// Disable the peripheral clock for a peripheral identified by its ID.
    pub fn disable_peripheral_clock(&self, pid: u32) {
        self.disable_write_protection();
        if pid < 32 {
            self.regs.pmc_pcdr0.set(1 << pid);
        } else if pid < 64 {
            self.regs.pmc_pcdr1.set(1 << (pid - 32));
        }
        self.enable_write_protection();
    }

    /// Returns true if the peripheral clock for the given ID is currently enabled.
    pub fn is_peripheral_clock_enabled(&self, pid: u32) -> bool {
        if pid < 32 {
            self.regs.pmc_pcsr0.get() & (1 << pid) != 0
        } else if pid < 64 {
            self.regs.pmc_pcsr1.get() & (1 << (pid - 32)) != 0
        } else {
            false
        }
    }

    /// Configure and enable a programmable clock output (PCK0–PCK7).
    ///
    /// `css`: clock source (0=Slow, 1=Main, 2=PLLA, 3=UPLL, 4=MCK)
    /// `pres`: prescaler (output = source / (pres + 1))
    pub fn configure_pck(&self, index: usize, css: u32, pres: u32) {
        if index > 7 {
            return;
        }
        self.disable_write_protection();
        self.regs.pmc_pck[index].write(PmcPck::CSS.val(css) + PmcPck::PRES.val(pres));
        self.regs.pmc_scer.set(1 << (index + 8));
        while self.regs.pmc_sr.get() & (1 << (index + 8)) == 0 {}
        self.enable_write_protection();
    }

    /// Configure and enable a peripheral's generic clock (GCLK).
    ///
    /// Used by MCAN, I2SC, and other peripherals that require a
    /// dedicated clock from the PMC generic clock generator.
    ///
    /// `pid`: peripheral ID (e.g., 35 for MCAN0, 37 for MCAN1)
    /// `css`: clock source (0=Slow, 1=Main, 2=PLLA, 3=UPLL, 4=MCK)
    /// `div`: division factor (output = source / (div + 1))
    pub fn configure_gclk(&self, pid: u32, css: u32, div: u32) {
        self.disable_write_protection();
        let pcr_val = (pid & 0x7F)
            | (1 << 12)           // CMD (write command)
            | (1 << 28)           // EN (peripheral clock enable)
            | (1 << 29)           // GCLKEN (generic clock enable)
            | ((css & 0x7) << 8)  // GCLKCSS
            | ((div & 0xFF) << 20); // GCLKDIV
        self.regs.pmc_pcr.set(pcr_val);
        self.enable_write_protection();
    }
}
