// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Embedded Flash Controller (EFC/EEFC) driver for SAMV71Q21B.
//!
//! - Flash size: 2 MB, base address 0x00400000 (also aliased at 0x00000000)
//! - Page size: 512 bytes  →  4096 pages total
//! - `read_page`:  direct memory copy (synchronous, callback fired inline)
//! - `write_page`: EWP (Erase and Write Page) command, interrupt-driven
//! - `erase_page`: no-op; EWP in write_page handles erasing

use core::cell::Cell;

use kernel::hil::flash;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
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

/// EFC peripheral ID (for PMC clock enable).
pub const EFC_PID: u32 = 6;

/// Flash memory base (physical, non-aliased).
const FLASH_BASE: usize = 0x0040_0000;

/// Flash page size in bytes.
pub const PAGE_SIZE: usize = 512;

/// Total number of 512-byte pages in 2 MB flash.
pub const PAGE_COUNT: usize = 4096;

/// EFC command key (must be written with every FCR write).
const FKEY: u32 = 0x5A;

/// EWP – Erase and Write Page.
const CMD_EWP: u32 = 0x03;

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
// Driver state
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, PartialEq)]
enum EfcState {
    Idle,
    Writing(usize),  // page number
}

pub struct Efc {
    regs: StaticRef<EfcRegisters>,
    client: OptionalCell<&'static dyn flash::Client<Efc>>,
    state: Cell<EfcState>,
    write_buf: TakeCell<'static, Sam71Page>,
}

impl Efc {
    pub const fn new() -> Self {
        Efc {
            regs: EFC_BASE,
            client: OptionalCell::empty(),
            state: Cell::new(EfcState::Idle),
            write_buf: TakeCell::empty(),
        }
    }

    /// Configure flash wait states for 150 MHz MCK (≥6 wait states required).
    /// Call once after PMC clock setup and before any flash access.
    pub fn init(&self) {
        // FWS=6: 7 cycles, suitable up to 166 MHz. Enable CLOE for performance.
        self.regs.fmr.write(Fmr::FWS.val(6) + Fmr::CLOE::SET);
    }

    /// Called from the NVIC handler for EFC (interrupt 6).
    pub fn handle_interrupt(&self) {
        let fsr = self.regs.fsr.extract();
        // FRDY = 1 means the last command finished.
        if !fsr.is_set(Fsr::FRDY) {
            return;
        }
        // Disable FRDY interrupt.
        self.regs.fmr.modify(Fmr::FRDY::CLEAR);

        match self.state.get() {
            EfcState::Writing(_page) => {
                self.state.set(EfcState::Idle);
                if let Some(buf) = self.write_buf.take() {
                    let result = if fsr.is_set(Fsr::FCMDE) || fsr.is_set(Fsr::FLOCKE) || fsr.is_set(Fsr::FLERR) {
                        Err(flash::Error::FlashError)
                    } else {
                        Ok(())
                    };
                    self.client.map(|c| c.write_complete(buf, result));
                }
            }
            EfcState::Idle => {}
        }
    }

    /// Write 512 bytes into the flash write latch for the given page.
    fn load_latch(&self, page_number: usize, buf: &Sam71Page) {
        let page_addr = FLASH_BASE + page_number * PAGE_SIZE;
        // Write 128 32-bit words. The EFC latches them until the EWP command.
        for i in 0..128 {
            let word = u32::from_le_bytes([
                buf.0[i * 4],
                buf.0[i * 4 + 1],
                buf.0[i * 4 + 2],
                buf.0[i * 4 + 3],
            ]);
            unsafe {
                core::ptr::write_volatile((page_addr + i * 4) as *mut u32, word);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// HIL: Flash
// ---------------------------------------------------------------------------

impl<'a, C: flash::Client<Self>> flash::HasClient<'a, C> for Efc {
    fn set_client(&'a self, client: &'a C) {
        // Safety: the bootloader's static lifetime ensures client outlives Efc.
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
        // Flash is memory-mapped: direct copy.
        let page_addr = FLASH_BASE + page_number * PAGE_SIZE;
        let flash_slice =
            unsafe { core::slice::from_raw_parts(page_addr as *const u8, PAGE_SIZE) };
        buf.0.copy_from_slice(flash_slice);
        // Synchronous read: fire callback inline.
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
        if self.state.get() != EfcState::Idle {
            return Err((ErrorCode::BUSY, buf));
        }

        // Load the 512-byte latch.
        self.load_latch(page_number, buf);
        self.write_buf.replace(buf);
        self.state.set(EfcState::Writing(page_number));

        // Enable FRDY interrupt, then issue EWP.
        self.regs.fmr.modify(Fmr::FRDY::SET);
        self.regs.fcr.write(
            Fcr::FKEY.val(FKEY)
                + Fcr::FARG.val(page_number as u32)
                + Fcr::FCMD.val(CMD_EWP),
        );
        Ok(())
    }

    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> {
        if page_number >= PAGE_COUNT {
            return Err(ErrorCode::INVAL);
        }
        // EWP in write_page handles erase; fire callback immediately.
        self.client.map(|c| c.erase_complete(Ok(())));
        Ok(())
    }
}
