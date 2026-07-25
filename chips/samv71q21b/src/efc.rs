// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Embedded Flash Controller (EFC/EEFC) driver for SAMV71Q21B.
//!
//! - Flash size: 2 MB, base address 0x00400000 (aliased at 0x00000000)
//! - Page size: 512 bytes  →  4096 pages total
//! - `read_page`:  direct memory copy from physical address (synchronous)
//! - `write_page`: CLB unlock + CPU latch fill + IAP WP (synchronous)
//! - `erase_page`: CLB unlock + EPA erase (synchronous)
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

/// EPA erase granularity: 16 pages (8 KB) per block.
/// FARG[1:0]=2 selects 16-page erase.  Using 32-page (FARG=3)
/// caused tockloader's post-install clear_bytes to erase app data
/// for small apps like c_hello (16 pages = 8 KB) because the
/// 32-page alignment rounded down into the app's own pages.
const EPA_ERASE_SIZE: u32 = 2;
const EPA_PAGE_ALIGN: u32 = 16;

/// ROM IAP function pointer address (NMI vector in ROM).
const IAP_ENTRY_ADDR: usize = 0x0080_0008;

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
    ///
    /// CLOE (Code Loop Optimization) is intentionally left disabled.
    /// The EEFC's internal read buffer can serve stale data after
    /// erase/write operations, causing CRC mismatches during
    /// tockloader app installs.  The performance cost is negligible
    /// for Tock's flash access patterns.
    pub fn init(&self) {
        self.regs.fmr.write(Fmr::FWS.val(6));
    }

    /// No-op — commands use synchronous IAP; no FRDY interrupt needed.
    pub fn handle_interrupt(&self) {}

    /// Issue an EFC command via the ROM IAP function. Returns FSR.
    ///
    /// Must run from SRAM — the IAP call triggers a flash operation (erase
    /// or write); fetching instructions from flash during that operation
    /// causes a read-while-write bus fault.
    #[link_section = ".ramfunc"]
    fn call_iap(cmd: u32, arg: u32) -> u32 {
        let fcr = (FKEY << 24) | ((arg & 0xFFFF) << 8) | (cmd & 0xFF);
        let iap_ptr = unsafe { core::ptr::read_volatile(IAP_ENTRY_ADDR as *const u32) };
        let iap: extern "C" fn(u32, u32) -> u32 = unsafe { core::mem::transmute(iap_ptr) };
        iap(0, fcr)
    }

    /// Erase the 16-page block containing `page_number` via EPA + IAP.
    #[link_section = ".ramfunc"]
    fn erase_block(page_number: usize) -> u32 {
        let first_page = (page_number as u32) & !(EPA_PAGE_ALIGN - 1);
        let epa_arg = first_page | EPA_ERASE_SIZE;
        Self::call_iap(CMD_EPA, epa_arg)
    }

    /// Fill the EEFC page latch and program a flash page — entirely
    /// from SRAM.
    ///
    /// The caller must erase the target page/block beforehand via
    /// `erase_page`; this function only does CLB + latch fill + WP.
    ///
    /// The entire sequence (cache cleanup → CLB → latch fill → WP →
    /// restore) runs from `.ramfunc` so the CPU never fetches
    /// instructions from flash during EFC operations.
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

            // --- 2. Re-init EEFC: FWS=6, CLOE OFF for the write ---
            let fmr_prev = core::ptr::read_volatile(EFC_FMR);
            core::ptr::write_volatile(EFC_FMR, 0x0000_0600);
            core::ptr::read_volatile(EFC_FSR);
            core::ptr::read_volatile(EFC_FSR);
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            // --- 3. CLB unlock via IAP (from ROM) ---
            let lock_region = page_number as u32 / 32;
            let clb_fcr = (FKEY << 24) | ((lock_region & 0xFFFF) << 8) | (CMD_CLB & 0xFF);
            iap(0, clb_fcr);
            core::arch::asm!("dsb sy", "isb sy", options(nostack, preserves_flags));

            // --- 4. Force I-Code completely idle ---
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

            // --- 5. Re-configure MATRIX flash slave default master ---
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

            // --- 6. Copy 128 words to flash via inline asm ---
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

            // --- 7. WP via IAP (from ROM) ---
            let wp_fcr = (FKEY << 24) | ((page_number as u32 & 0xFFFF) << 8) | (CMD_WP & 0xFF);
            let fsr = iap(0, wp_fcr);

            // --- 8. Restore NVIC, VTOR, MATRIX, EEFC ---
            core::ptr::write_volatile(SCB_VTOR, vtor_prev);
            core::ptr::write_volatile(0xE000_E100 as *mut u32, icer0); // ISER0
            core::ptr::write_volatile(0xE000_E104 as *mut u32, icer1);
            core::ptr::write_volatile(0xE000_E108 as *mut u32, icer2);
            core::arch::asm!("cpsie i", options(nostack, preserves_flags));
            core::ptr::write_volatile(MATRIX_SCFG2, scfg2_prev);
            core::ptr::write_volatile(MATRIX_SCFG3, scfg3_prev);
            core::ptr::write_volatile(EFC_FMR, fmr_prev);
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

        // Erase if the page isn't clean. Tockloader may not send
        // explicit ErasePage commands before WritePage, so we must
        // handle it here. The EPA erases a 16-page block, but since
        // tockloader writes pages in ascending order, the block-erase
        // only hits pages that haven't been written yet.
        let page_addr = FLASH_BASE + page_number * PAGE_SIZE;
        let needs_erase = unsafe {
            let p = page_addr as *const u32;
            let mut dirty = false;
            for i in 0..(PAGE_SIZE / 4) {
                if core::ptr::read_volatile(p.add(i)) != 0xFFFF_FFFF {
                    dirty = true;
                    break;
                }
            }
            dirty
        };
        if needs_erase {
            let lock_region = page_number / 32;
            Self::call_iap(CMD_CLB, lock_region as u32);
            Self::erase_block(page_number);
        }

        let fsr = Self::load_latch(page_number, buf);

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

        // EPA erases a 16-page (8 KB) block, not a single page.
        // Skip if the target page is already 0xFF to avoid collateral
        // erasure of neighboring pages in the same block.
        let page_addr = FLASH_BASE + page_number * PAGE_SIZE;
        let already_erased = unsafe {
            let p = page_addr as *const u32;
            let mut erased = true;
            let words = PAGE_SIZE / 4;
            for i in 0..words {
                if core::ptr::read_volatile(p.add(i)) != 0xFFFF_FFFF {
                    erased = false;
                    break;
                }
            }
            erased
        };

        if already_erased {
            self.client.map(|c| c.erase_complete(Ok(())));
            return Ok(());
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
