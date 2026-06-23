// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Embedded Flash Controller (EFC/EEFC) driver for SAMV71Q21B.
//!
//! - Flash size: 2 MB, base address 0x00400000 (aliased at 0x00000000)
//! - Page size: 512 bytes  →  4096 pages total
//! - `read_page`:  direct memory copy from physical address (synchronous)
//! - `write_page`: EPA erase + CPU latch fill + IAP WP (synchronous)
//! - `erase_page`: no-op; write_page handles erasing
//!
//! Flash programming on the Cortex-M7 SAMV71 requires two workarounds:
//!
//! 1. **Latch fill via alias address** — The EEFC intercepts writes to
//!    the flash alias region (0x00000000+) for its page latch. The CPU
//!    must write through an MPU Strongly-Ordered region so the stores
//!    bypass the D-Code cache path and reach the EEFC.
//!
//! 2. **Commands via ROM IAP** — Flash commands must not execute from
//!    flash (read-while-write conflict). The ROM IAP function (entry
//!    point at 0x00800008) issues the command and returns FSR.
//!
//! 3. **WP instead of EWP** — The EWP (Erase-and-Write-Page) command
//!    returns FCMDE on this silicon. EPA (Erase Pages) + WP (Write
//!    Page) issued separately work correctly.

use kernel::hil::flash;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::Writeable;
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

// ---------------------------------------------------------------------------
// Register layout
// ---------------------------------------------------------------------------

register_structs! {
    EfcRegisters {
        (0x000 => fmr: ReadWrite<u32, Fmr::Register>),
        (0x004 => fcr: WriteOnly<u32, Fcr::Register>),
        (0x008 => fsr: ReadOnly<u32,  Fsr::Register>),
        (0x00C => frr: ReadOnly<u32>),
        (0x010 => @END),
    }
}

register_bitfields![u32,
    Fmr [
        FRDY  OFFSET(0)  NUMBITS(1) [],
        FWS   OFFSET(8)  NUMBITS(4) [],
        SCOD  OFFSET(16) NUMBITS(1) [],
        FAM   OFFSET(24) NUMBITS(1) [],
        CLOE  OFFSET(26) NUMBITS(1) [],
    ],
    Fcr [
        FCMD  OFFSET(0)  NUMBITS(8) [],
        FARG  OFFSET(8)  NUMBITS(16) [],
        FKEY  OFFSET(24) NUMBITS(8) [],
    ],
    Fsr [
        FRDY   OFFSET(0) NUMBITS(1) [],
        FCMDE  OFFSET(1) NUMBITS(1) [],
        FLOCKE OFFSET(2) NUMBITS(1) [],
        FLERR  OFFSET(3) NUMBITS(1) [],
    ],
];

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const EFC_BASE: StaticRef<EfcRegisters> =
    unsafe { StaticRef::new(0x400E_0C00 as *const EfcRegisters) };

/// EFC peripheral ID (for PMC clock gating / NVIC).
pub const EFC_PID: u32 = 6;

/// Flash memory base (physical, non-aliased).
const FLASH_BASE: usize = 0x0040_0000;

/// Flash page size in bytes.
pub const PAGE_SIZE: usize = 512;

/// Total number of 512-byte pages in 2 MB flash.
pub const PAGE_COUNT: usize = 4096;

const FKEY: u32 = 0x5A;

/// WP – Write Page (latch → flash, no erase).
const CMD_WP: u32 = 0x01;

/// EPA – Erase Pages.
const CMD_EPA: u32 = 0x07;

/// CLB – Clear Lock Bit.
const CMD_CLB: u32 = 0x09;

/// Minimum erase granularity on the 2 MB SAMV71Q21B is 32 pages
/// (16 KB). EPA FARG[1:0]=3 selects 32-page erase.
const EPA_ERASE_SIZE: u32 = 3; // FARG[1:0]: 0=4pp, 1=8pp, 2=16pp, 3=32pp
const EPA_PAGE_ALIGN: u32 = 32;

/// ROM IAP function pointer address (NMI vector in ROM).
const IAP_ENTRY_ADDR: usize = 0x0080_0008;

// MPU registers
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

// ---------------------------------------------------------------------------
// Page buffer type
// ---------------------------------------------------------------------------

pub struct Sam71Page(pub [u8; PAGE_SIZE]);

impl Default for Sam71Page {
    fn default() -> Self {
        Sam71Page([0; PAGE_SIZE])
    }
}

impl core::ops::Index<usize> for Sam71Page {
    type Output = u8;
    fn index(&self, idx: usize) -> &u8 {
        &self.0[idx]
    }
}

impl core::ops::IndexMut<usize> for Sam71Page {
    fn index_mut(&mut self, idx: usize) -> &mut u8 {
        &mut self.0[idx]
    }
}

impl AsMut<[u8]> for Sam71Page {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Efc {
    regs: StaticRef<EfcRegisters>,
    client: OptionalCell<&'static dyn flash::Client<Efc>>,
}

impl Efc {
    pub const fn new() -> Self {
        Efc {
            regs: EFC_BASE,
            client: OptionalCell::empty(),
        }
    }

    /// Configure flash wait states for 150 MHz MCK (≥6 wait states).
    pub fn init(&self) {
        self.regs.fmr.write(Fmr::FWS.val(6) + Fmr::CLOE::SET);
    }

    /// No-op — commands use synchronous IAP; no FRDY interrupt needed.
    pub fn handle_interrupt(&self) {}

    /// Issue an EFC command via the ROM IAP function. Returns FSR.
    fn call_iap(cmd: u32, arg: u32) -> u32 {
        let fcr = (FKEY << 24) | ((arg & 0xFFFF) << 8) | (cmd & 0xFF);
        let iap_ptr = unsafe { core::ptr::read_volatile(IAP_ENTRY_ADDR as *const u32) };
        let iap: extern "C" fn(u32, u32) -> u32 = unsafe { core::mem::transmute(iap_ptr) };
        iap(0, fcr)
    }

    /// Erase the 32-page block containing `page_number` via EPA + IAP.
    fn erase_block(page_number: usize) -> u32 {
        let first_page = (page_number as u32) & !(EPA_PAGE_ALIGN - 1);
        let epa_arg = first_page | EPA_ERASE_SIZE;
        Self::call_iap(CMD_EPA, epa_arg)
    }

    /// Fill the EEFC page latch by writing 128 words to the flash alias.
    ///
    /// This function is placed in SRAM (`.ramfunc` section) so that the
    /// entire sequence — cache cleanup, MPU setup, copy loop, MPU
    /// teardown — executes without any flash instruction fetches. On
    /// Cortex-M7 SAMV71, accumulated I/D-cache and bus-matrix state
    /// from prior flash accesses prevents the EEFC from capturing
    /// latch writes if any part of the sequence runs from flash.
    /// Erase, fill latch, and program a flash page — entirely from SRAM.
    ///
    /// The entire sequence (cache cleanup → CLB → EPA → MPU setup →
    /// latch fill → WP → restore) runs from `.ramfunc` so the CPU
    /// never fetches instructions from flash during EFC operations.
    /// Accumulated I/D-cache and bus-matrix state from prior flash
    /// accesses prevents latch writes from reaching the EEFC if any
    /// part of the sequence runs from flash.
    #[link_section = ".ramfunc"]
    #[inline(never)]
    fn load_latch(page_number: usize, buf: &Sam71Page) -> u32 {
        let page_addr = FLASH_BASE + page_number * PAGE_SIZE;

        const SCB_CCR: *mut u32 = 0xE000_ED14 as *mut u32;
        const SCB_ICIALLU: *mut u32 = 0xE000_EF50 as *mut u32;
        const SCB_DCISW: *mut u32 = 0xE000_EF60 as *mut u32;
        const SCB_CCSIDR: *const u32 = 0xE000_ED80 as *const u32;
        const SCB_CSSELR: *mut u32 = 0xE000_ED84 as *mut u32;
        const EFC_FMR: *mut u32 = 0x400E_0C00 as *mut u32;
        const EFC_FSR: *const u32 = 0x400E_0C08 as *const u32;

        // IAP function pointer (ROM NMI vector).
        let iap_ptr = unsafe { core::ptr::read_volatile(IAP_ENTRY_ADDR as *const u32) };
        let iap: extern "C" fn(u32, u32) -> u32 = unsafe { core::mem::transmute(iap_ptr) };

        unsafe {
            // --- 1. Disable and invalidate caches ---
            let ccr = core::ptr::read_volatile(SCB_CCR);
            if ccr & (1 << 17) != 0 {
                core::ptr::write_volatile(SCB_CCR, ccr & !(1 << 17));
                core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));
            }
            core::ptr::write_volatile(SCB_ICIALLU, 0);
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            if ccr & (1 << 16) != 0 {
                core::ptr::write_volatile(SCB_CCR, core::ptr::read_volatile(SCB_CCR) & !(1 << 16));
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
                core::ptr::write_volatile(SCB_CSSELR, 0);
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
                let ccsidr = core::ptr::read_volatile(SCB_CCSIDR);
                let sets = ((ccsidr >> 13) & 0x7FFF) as u32;
                let ways = ((ccsidr >> 3) & 0x3FF) as u32;
                for way in 0..=ways {
                    for set in 0..=sets {
                        core::ptr::write_volatile(SCB_DCISW, (way << 30) | (set << 5));
                    }
                }
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            }

            // --- 2. Re-init EEFC: FWS=6, CLOE OFF ---
            core::ptr::write_volatile(EFC_FMR, 0x0000_0600);
            core::ptr::read_volatile(EFC_FSR);
            core::ptr::read_volatile(EFC_FSR);
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            // --- 3. CLB unlock via IAP (from ROM) ---
            let lock_region = page_number as u32 / 32;
            let clb_fcr = (FKEY << 24) | ((lock_region & 0xFFFF) << 8) | (CMD_CLB & 0xFF);
            iap(0, clb_fcr);

            // --- 4. EPA erase via IAP (from ROM) ---
            if page_number as u32 % EPA_PAGE_ALIGN == 0 {
                let first_page = (page_number as u32) & !(EPA_PAGE_ALIGN - 1);
                let epa_arg = first_page | EPA_ERASE_SIZE;
                let epa_fcr = (FKEY << 24) | ((epa_arg & 0xFFFF) << 8) | (CMD_EPA & 0xFF);
                iap(0, epa_fcr);
            }
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            // --- 5. Force I-Code completely idle ---
            // Move vector table to SRAM so NO flash reads can occur
            // (not even for fault handlers or NMI vector).
            const SCB_VTOR: *mut u32 = 0xE000_ED08 as *mut u32;
            let vtor_prev = core::ptr::read_volatile(SCB_VTOR);
            core::ptr::write_volatile(SCB_VTOR, 0x2040_0000);
            // Disable all NVIC interrupts
            const NVIC_ICER0: *mut u32 = 0xE000_E180 as *mut u32;
            const NVIC_ICER1: *mut u32 = 0xE000_E184 as *mut u32;
            const NVIC_ICER2: *mut u32 = 0xE000_E188 as *mut u32;
            let icer0 = core::ptr::read_volatile(0xE000_E100 as *const u32); // ISER0
            let icer1 = core::ptr::read_volatile(0xE000_E104 as *const u32);
            let icer2 = core::ptr::read_volatile(0xE000_E108 as *const u32);
            core::ptr::write_volatile(NVIC_ICER0, 0xFFFF_FFFF);
            core::ptr::write_volatile(NVIC_ICER1, 0xFFFF_FFFF);
            core::ptr::write_volatile(NVIC_ICER2, 0xFFFF_FFFF);
            // Mask interrupts at CPU level
            core::arch::asm!("cpsid i", options(nostack, preserves_flags));
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            // --- 5b. Re-configure MATRIX flash slave default master ---
            // The flash slave (SCFG[2]) defaults to Fixed Default
            // Master = I-Code (master 0). The I-Code prefetch unit
            // holds flash access even when running from SRAM, blocking
            // D-Code/System bus writes to the EEFC. Switch to No
            // Default Master so our System bus writes can reach the
            // flash slave.
            const MATRIX_SCFG2: *mut u32 = 0x4008_8048 as *mut u32;
            let scfg2_prev = core::ptr::read_volatile(MATRIX_SCFG2);
            // DEFMSTR_TYPE=0 (No Default Master), SLOT_CYCLE=511
            core::ptr::write_volatile(MATRIX_SCFG2, 0x0000_01FF);
            // Also do SCFG[3] in case flash spans both slaves
            const MATRIX_SCFG3: *mut u32 = 0x4008_804C as *mut u32;
            let scfg3_prev = core::ptr::read_volatile(MATRIX_SCFG3);
            core::ptr::write_volatile(MATRIX_SCFG3, 0x0000_01FF);

            // --- 7. Copy 128 words to flash via inline asm ---
            let src = buf.0.as_ptr();
            let dst = page_addr as *mut u8;
            core::arch::asm!(
                "mov {cnt}, #128",
                "2:",
                "ldr {tmp}, [{src}], #4",
                "str {tmp}, [{dst}], #4",
                "subs {cnt}, {cnt}, #1",
                "bne 2b",
                "dsb sy",
                src = inout(reg) src => _,
                dst = inout(reg) dst => _,
                cnt = out(reg) _,
                tmp = out(reg) _,
                options(nostack),
            );

            // --- 8. WP via IAP (from ROM) ---
            let wp_fcr = (FKEY << 24) | ((page_number as u32 & 0xFFFF) << 8) | (CMD_WP & 0xFF);
            let fsr = iap(0, wp_fcr);

            // --- 9. Restore NVIC, VTOR, MATRIX, EEFC ---
            core::ptr::write_volatile(SCB_VTOR, vtor_prev);
            core::ptr::write_volatile(0xE000_E100 as *mut u32, icer0); // ISER0
            core::ptr::write_volatile(0xE000_E104 as *mut u32, icer1);
            core::ptr::write_volatile(0xE000_E108 as *mut u32, icer2);
            core::arch::asm!("cpsie i", options(nostack, preserves_flags));
            core::ptr::write_volatile(MATRIX_SCFG2, scfg2_prev);
            core::ptr::write_volatile(MATRIX_SCFG3, scfg3_prev);
            core::ptr::write_volatile(EFC_FMR, 0x0400_0600);
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            if ccr & (1 << 17) != 0 {
                core::ptr::write_volatile(SCB_ICIALLU, 0);
                core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));
                core::ptr::write_volatile(SCB_CCR, core::ptr::read_volatile(SCB_CCR) | (1 << 17));
                core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));
            }
            if ccr & (1 << 16) != 0 {
                core::ptr::write_volatile(SCB_CCR, core::ptr::read_volatile(SCB_CCR) | (1 << 16));
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            }

            fsr
        }
    }
}

// ---------------------------------------------------------------------------
// HIL: Flash
// ---------------------------------------------------------------------------

impl<'a, C: flash::Client<Self>> flash::HasClient<'a, C> for Efc {
    fn set_client(&'a self, client: &'a C) {
        let client_ref: &'static dyn flash::Client<Efc> =
            unsafe { core::mem::transmute(client as &dyn flash::Client<Efc>) };
        self.client.set(client_ref);
    }
}

impl flash::Flash for Efc {
    type Page = Sam71Page;

    fn read_page(
        &self,
        page_number: usize,
        buf: &'static mut Sam71Page,
    ) -> Result<(), (ErrorCode, &'static mut Sam71Page)> {
        if page_number >= PAGE_COUNT {
            return Err((ErrorCode::INVAL, buf));
        }
        let page_addr = FLASH_BASE + page_number * PAGE_SIZE;
        let flash_slice =
            unsafe { core::slice::from_raw_parts(page_addr as *const u8, PAGE_SIZE) };
        buf.0.copy_from_slice(flash_slice);
        self.client.map(|c| c.read_complete(buf, Ok(())));
        Ok(())
    }

    fn write_page(
        &self,
        page_number: usize,
        buf: &'static mut Sam71Page,
    ) -> Result<(), (ErrorCode, &'static mut Sam71Page)> {
        if page_number >= PAGE_COUNT {
            return Err((ErrorCode::INVAL, buf));
        }

        // Debug: page number, first data word
        unsafe {
            let w0 = u32::from_le_bytes([buf.0[0], buf.0[1], buf.0[2], buf.0[3]]);
            core::ptr::write_volatile(0x400E_1890 as *mut u32, page_number as u32);
            core::ptr::write_volatile(0x400E_1894 as *mut u32, w0);
        }

        let fsr = Self::load_latch(page_number, buf);

        // Debug: WP FSR result
        unsafe {
            core::ptr::write_volatile(0x400E_1898 as *mut u32, fsr);
        }

        let result = if fsr & 0x0E != 0 {
            Err(flash::Error::FlashError)
        } else {
            Ok(())
        };
        self.client.map(|c| c.write_complete(buf, result));
        Ok(())
    }

    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> {
        if page_number >= PAGE_COUNT {
            return Err(ErrorCode::INVAL);
        }

        let lock_region = page_number / 32;
        Self::call_iap(CMD_CLB, lock_region as u32);

        let fsr = Self::erase_block(page_number);

        let result = if fsr & 0x0E != 0 {
            Err(flash::Error::FlashError)
        } else {
            Ok(())
        };
        self.client.map(|c| c.erase_complete(result));
        Ok(())
    }
}
