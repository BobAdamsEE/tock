// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Power Management Controller (PMC) for SAMV71Q21B.
//! Modified from SAM4L pm.rs to target SAMV71Q21B, removing sleep modes and non-basic functions.
//! Implements clock setup following the recommended programming sequence from datasheet section 31.17.
//! Default configuration uses external 12 MHz crystal and sets up clocks for USB use.
//! Assumes 300 MHz CPU clock via PLLA (12 MHz * 25).

use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;

const PMC_BASE: StaticRef<PmcRegisters> =
    unsafe { StaticRef::new(0x400E0600 as *const PmcRegisters) };

#[repr(C)]
struct PmcRegisters {
    pmc_scer: WriteOnly<u32, PmcScer::Register>,
    pmc_scdr: WriteOnly<u32, PmcScdr::Register>,
    _reserved0: [u32; 2],
    ckgr_uckr: ReadWrite<u32, CkgrUckr::Register>,
    ckgr_mor: ReadWrite<u32, CkgrMor::Register>,
    ckgr_mcfr: ReadOnly<u32, CkgrMcfr::Register>,
    ckgr_pllar: ReadWrite<u32, CkgrPllar::Register>,
    _reserved1: u32,
    pmc_mckr: ReadWrite<u32, PmcMckr::Register>,
    _reserved2: [u32; 3],
    pmc_pck: [ReadWrite<u32, PmcPck::Register>; 8],
    pmc_ier: WriteOnly<u32, PmcSr::Register>,
    pmc_idr: WriteOnly<u32, PmcSr::Register>,
    pmc_sr: ReadOnly<u32, PmcSr::Register>,
    pmc_imr: ReadOnly<u32, PmcSr::Register>,
    pmc_fsmr: ReadWrite<u32>,
    pmc_fspr: ReadWrite<u32>,
    pmc_focr: ReadWrite<u32>,
    _reserved3: [u32; 26],
    pmc_wpmr: ReadWrite<u32, PmcWpmr::Register>,
    pmc_wpsr: ReadOnly<u32>,
    _reserved4: [u32; 5],
    pmc_pcer0: WriteOnly<u32, PmcPcx::Register>,
    pmc_pcdr0: WriteOnly<u32, PmcPcx::Register>,
    pmc_pcsr0: ReadOnly<u32, PmcPcx::Register>,
    ckgr_uckr2: ReadWrite<u32>, // Not used in basic setup
    pmc_pcer1: WriteOnly<u32, PmcPcx::Register>,
    pmc_pcdr1: WriteOnly<u32, PmcPcx::Register>,
    pmc_pcsr1: ReadOnly<u32, PmcPcx::Register>,
    pmc_pcr: ReadWrite<u32>,
    pmc_ocr: ReadWrite<u32>,
    pmc_slpwk_er: WriteOnly<u32>,
    pmc_slpwk_dr: WriteOnly<u32>,
    pmc_slpwk_sr: ReadOnly<u32>,
    pmc_slpwk_asr: ReadOnly<u32>,
    pmc_pmmr: ReadWrite<u32>,
}

register_bitfields![u32,
    CkgrMor [
        KEY OFFSET(24) NUMBITS(8) [],
        MOSCXTST OFFSET(8) NUMBITS(8) [],
        MOSCRCF OFFSET(4) NUMBITS(3) [
            _4MHz = 0,
            _8MHz = 1,
            _12MHz = 2
        ],
        MOSCRCEN OFFSET(3) NUMBITS(1) [],
        MOSCXTBY OFFSET(2) NUMBITS(1) [],
        MOSCXTEN OFFSET(1) NUMBITS(1) [],
        MOSCSEL OFFSET(0) NUMBITS(1) []
    ],
    CkgrMcfr [
        MAINFRDY OFFSET(16) NUMBITS(1) [],
        MAINF OFFSET(0) NUMBITS(16) []
    ],
    CkgrPllar [
        ONE OFFSET(29) NUMBITS(1) [],
        MULA OFFSET(16) NUMBITS(11) [],
        PLLACOUNT OFFSET(8) NUMBITS(6) [],
        DIVA OFFSET(0) NUMBITS(8) []
    ],
    PmcMckr [
        UPLLDIV2 OFFSET(13) NUMBITS(1) [],
        MDIV OFFSET(8) NUMBITS(2) [],
        PRES OFFSET(4) NUMBITS(3) [],
        CSS OFFSET(0) NUMBITS(2) [
            Slow = 0,
            Main = 1,
            Plla = 2,
            Upll = 3
        ]
    ],
    CkgrUckr [
        UPLLCOUNT OFFSET(24) NUMBITS(4) [],
        BIASEN OFFSET(20) NUMBITS(1) [],
        UPLLEN OFFSET(16) NUMBITS(1) []
    ],
    PmcScer [
        PCK7 OFFSET(15) NUMBITS(1) [],
        PCK6 OFFSET(14) NUMBITS(1) [],
        PCK5 OFFSET(13) NUMBITS(1) [],
        PCK4 OFFSET(12) NUMBITS(1) [],
        PCK3 OFFSET(11) NUMBITS(1) [],
        PCK2 OFFSET(10) NUMBITS(1) [],
        PCK1 OFFSET(9) NUMBITS(1) [],
        PCK0 OFFSET(8) NUMBITS(1) [],
        USBCLK OFFSET(5) NUMBITS(1) []
    ],
    PmcScdr [
        PCK7 OFFSET(15) NUMBITS(1) [],
        PCK6 OFFSET(14) NUMBITS(1) [],
        PCK5 OFFSET(13) NUMBITS(1) [],
        PCK4 OFFSET(12) NUMBITS(1) [],
        PCK3 OFFSET(11) NUMBITS(1) [],
        PCK2 OFFSET(10) NUMBITS(1) [],
        PCK1 OFFSET(9) NUMBITS(1) [],
        PCK0 OFFSET(8) NUMBITS(1) [],
        USBCLK OFFSET(5) NUMBITS(1) []
    ],
    PmcPck [
        PRES OFFSET(4) NUMBITS(8) [],
        CSS OFFSET(0) NUMBITS(3) []
    ],
    PmcSr [
        XT32KERR OFFSET(21) NUMBITS(1) [],
        CFDS OFFSET(19) NUMBITS(1) [],
        CFDEV OFFSET(18) NUMBITS(1) [],
        MOSCRCS OFFSET(17) NUMBITS(1) [],
        MOSCSELS OFFSET(16) NUMBITS(1) [],
        PCKRDY7 OFFSET(15) NUMBITS(1) [],
        PCKRDY6 OFFSET(14) NUMBITS(1) [],
        PCKRDY5 OFFSET(13) NUMBITS(1) [],
        PCKRDY4 OFFSET(12) NUMBITS(1) [],
        PCKRDY3 OFFSET(11) NUMBITS(1) [],
        PCKRDY2 OFFSET(10) NUMBITS(1) [],
        PCKRDY1 OFFSET(9) NUMBITS(1) [],
        PCKRDY0 OFFSET(8) NUMBITS(1) [],
        LOCKU OFFSET(6) NUMBITS(1) [],
        MCKRDY OFFSET(3) NUMBITS(1) [],
        LOCKA OFFSET(1) NUMBITS(1) [],
        MOSCXTS OFFSET(0) NUMBITS(1) []
    ],
    PmcWpmr [
        WPKEY OFFSET(8) NUMBITS(24) [],
        WPEN OFFSET(0) NUMBITS(1) []
    ],
    PmcPcx [
        PID OFFSET(0) NUMBITS(32) []
    ]
];

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
            .modify(PmcWpmr::WPKEY.val(0x504D43) + PmcWpmr::WPEN.val(0));
    }

    fn enable_write_protection(&self) {
        self.regs
            .pmc_wpmr
            .modify(PmcWpmr::WPKEY.val(0x504D43) + PmcWpmr::WPEN.val(1));
    }

    pub fn setup_clocks(&self) {
        self.disable_write_protection();

        // Step 2: Enable and Stabilize the Main Oscillator (MAINCK) with external 12 MHz crystal
        // Startup time: 62 * 8 SLCK cycles (assuming SLCK ~32 kHz, ~15 ms)
        self.regs
            .ckgr_mor
            .modify(CkgrMor::KEY.val(0x37) + CkgrMor::MOSCXTEN.val(1) + CkgrMor::MOSCXTST.val(62));
        while !self.regs.pmc_sr.is_set(PmcSr::MOSCXTS) {}

        // Switch MAINCK to the main oscillator
        self.regs
            .ckgr_mor
            .modify(CkgrMor::KEY.val(0x37) + CkgrMor::MOSCSEL.val(1));
        while !self.regs.pmc_sr.is_set(PmcSr::MOSCSELS) {}

        // Step 3: Configure and Lock the PLLA (300 MHz: DIVA=1, MULA=24 -> 12 MHz * 25 = 300 MHz)
        self.regs.ckgr_pllar.write(
            CkgrPllar::ONE::SET
                + CkgrPllar::MULA.val(24)
                + CkgrPllar::PLLACOUNT.val(0x3F)
                + CkgrPllar::DIVA.val(1),
        );
        while !self.regs.pmc_sr.is_set(PmcSr::LOCKA) {}

        // Step 4: Select the Processor/Master Clock (MCK) to PLLA (no prescaler, no divider)
        // Follow safe sequence for switching to PLL
        self.regs.pmc_mckr.modify(PmcMckr::PRES.val(0));
        while !self.regs.pmc_sr.is_set(PmcSr::MCKRDY) {}

        self.regs.pmc_mckr.modify(PmcMckr::MDIV.val(0));
        while !self.regs.pmc_sr.is_set(PmcSr::MCKRDY) {}

        self.regs.pmc_mckr.modify(PmcMckr::CSS.val(2)); // PLLA
        while !self.regs.pmc_sr.is_set(PmcSr::MCKRDY) {}

        // Step 7: USB Full-Speed Clock (UPLL) Configuration
        self.regs
            .ckgr_uckr
            .modify(CkgrUckr::UPLLEN.val(1) + CkgrUckr::UPLLCOUNT.val(0xF));
        while !self.regs.pmc_sr.is_set(PmcSr::LOCKU) {}

        // Enable USB clock
        self.regs.pmc_scer.set(PmcScer::USBCLK::SET.into());

        self.enable_write_protection();
    }

    // Function to enable peripheral clocks (basic, as per sequence step 5)
    pub fn enable_peripheral_clock(&self, pid: u32) {
        self.disable_write_protection();
        if pid < 32 {
            self.regs.pmc_pcer0.set(1 << pid);
        } else {
            self.regs.pmc_pcer1.set(1 << (pid - 32));
        }
        self.enable_write_protection();
    }

    pub fn disable_peripheral_clock(&self, pid: u32) {
        self.disable_write_protection();
        if pid < 32 {
            self.regs.pmc_pcdr0.set(1 << pid);
        } else {
            self.regs.pmc_pcdr1.set(1 << (pid - 32));
        }
        self.enable_write_protection();
    }

    // Optional: Configure Programmable Clock (PCKx) as per step 6
    pub fn configure_pck(&self, index: usize, css: u32, pres: u32) {
        if index > 7 {
            return;
        }
        self.disable_write_protection();
        self.regs.pmc_pck[index].modify(PmcPck::CSS.val(css) + PmcPck::PRES.val(pres));
        self.regs.pmc_scer.set(1 << (index + 8));
        while !self.regs.pmc_sr.get() & (1 << (index + 8)) != 0 {}
        self.enable_write_protection();
    }
}
