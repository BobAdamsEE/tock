// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! XDMAC (Extended DMA Controller) driver for SAMV71Q21B.
//!
//! Provides peripheral-to-memory DMA transfers used by the USART1 driver
//! for receive operations. The XDMAC has 24 channels; each can be bound
//! to any peripheral via the PERID field in the channel configuration.
//!
//! SAMV71 XDMAC peripheral IDs (Table 22-1):
//!   PERID  9 = USART1 TX
//!   PERID 10 = USART1 RX

use core::cell::Cell;

/// XDMAC base address.
const XDMAC_BASE: usize = 0x4007_8000;

/// XDMAC peripheral ID for PMC clock enable.
pub const XDMAC_PID: u32 = 58;

// Global register offsets
#[allow(dead_code)] // part of the XDMAC register map; not yet used
const GIE: usize = 0x0C;
#[allow(dead_code)] // part of the XDMAC register map; not yet used
const GID: usize = 0x10;
const GIS: usize = 0x18;
const GE: usize = 0x1C;
const GD: usize = 0x20;
const GS: usize = 0x24;

// Per-channel register offsets (base = XDMAC_BASE + 0x50 + ch * 0x40)
#[allow(dead_code)] // part of the XDMAC register map; not yet used
const CIE: usize = 0x00;
#[allow(dead_code)] // part of the XDMAC register map; not yet used
const CID: usize = 0x04;
const CIS: usize = 0x0C;
const CSA: usize = 0x10;
const CDA: usize = 0x14;
const CNDA: usize = 0x18;
const CNDC: usize = 0x1C;
const CUBC: usize = 0x20;
const CBC: usize = 0x24;
const CC: usize = 0x28;

// CC register field positions
const CC_TYPE_POS: u32 = 0;
const CC_DSYNC_POS: u32 = 4;
const CC_DWIDTH_POS: u32 = 11;
const CC_SIF_POS: u32 = 13;
const CC_DIF_POS: u32 = 14;
const CC_SAM_POS: u32 = 16;
const CC_DAM_POS: u32 = 18;
const CC_PERID_POS: u32 = 24;

// CIS / CIE bits
const BIS: u32 = 1 << 0; // End of Block

/// XDMAC channel used for USART1 RX DMA.
pub const USART1_RX_CHANNEL: usize = 0;

/// XDMAC PERID for USART1 RX (datasheet Table 22-1).
pub const USART1_RX_PERID: u32 = 10;

#[inline(always)]
fn global_reg(offset: usize) -> *mut u32 {
    (XDMAC_BASE + offset) as *mut u32
}

#[inline(always)]
fn channel_reg(ch: usize, offset: usize) -> *mut u32 {
    (XDMAC_BASE + 0x50 + ch * 0x40 + offset) as *mut u32
}

pub struct Xdmac {
    usart1_rx_original_count: Cell<u32>,
}

impl Xdmac {
    pub const fn new() -> Self {
        Xdmac {
            usart1_rx_original_count: Cell::new(0),
        }
    }

    /// Configure a channel for peripheral-to-memory byte transfer.
    ///
    /// - `src_addr`: peripheral data register (e.g. USART1 RHR)
    /// - `dst_addr`: SRAM buffer address
    /// - `count`: number of bytes to transfer
    /// - `perid`: XDMAC peripheral ID for hardware handshake
    pub fn configure_periph_to_mem(
        &self,
        channel: usize,
        perid: u32,
        src_addr: u32,
        dst_addr: u32,
        count: u32,
    ) {
        unsafe {
            // Disable channel first
            core::ptr::write_volatile(global_reg(GD), 1 << channel);
            // Read CIS to clear any pending status
            core::ptr::read_volatile(channel_reg(channel, CIS));

            core::ptr::write_volatile(channel_reg(channel, CSA), src_addr);
            core::ptr::write_volatile(channel_reg(channel, CDA), dst_addr);
            core::ptr::write_volatile(channel_reg(channel, CNDA), 0);
            core::ptr::write_volatile(channel_reg(channel, CNDC), 0);
            core::ptr::write_volatile(channel_reg(channel, CUBC), count);
            core::ptr::write_volatile(channel_reg(channel, CBC), 0);

            // CC: PER_TRAN | PER2MEM | BYTE | SIF=IF1 | DIF=IF0 |
            //     SAM=FIXED | DAM=INCR | PERID
            let cc = (1 << CC_TYPE_POS)        // PER_TRAN
                | (0 << CC_DSYNC_POS)          // PER2MEM
                | (0 << CC_DWIDTH_POS)         // BYTE
                | (1 << CC_SIF_POS)            // IF1 (peripheral bus)
                | (0 << CC_DIF_POS)            // IF0 (system bus / SRAM)
                | (0 << CC_SAM_POS)            // FIXED (source = RHR)
                | (1 << CC_DAM_POS)            // INCREMENTED (destination)
                | (perid << CC_PERID_POS);
            core::ptr::write_volatile(channel_reg(channel, CC), cc);
        }

        if channel == USART1_RX_CHANNEL {
            self.usart1_rx_original_count.set(count);
        }
    }

    pub fn enable_channel(&self, channel: usize) {
        unsafe {
            core::ptr::write_volatile(global_reg(GE), 1 << channel);
        }
    }

    pub fn disable_channel(&self, channel: usize) {
        unsafe {
            core::ptr::write_volatile(global_reg(GD), 1 << channel);
        }
    }

    pub fn is_channel_enabled(&self, channel: usize) -> bool {
        unsafe { core::ptr::read_volatile(global_reg(GS)) & (1 << channel) != 0 }
    }

    /// Returns the number of bytes remaining in the current transfer.
    pub fn remaining_count(&self, channel: usize) -> u32 {
        unsafe { core::ptr::read_volatile(channel_reg(channel, CUBC)) & 0x00FF_FFFF }
    }

    /// Returns bytes transferred so far on the USART1 RX channel.
    pub fn usart1_rx_transferred(&self) -> u32 {
        let remaining = self.remaining_count(USART1_RX_CHANNEL);
        self.usart1_rx_original_count.get() - remaining
    }

    /// Handle XDMAC global interrupt. Returns the channel mask of
    /// channels that completed an end-of-block transfer.
    pub fn handle_interrupt(&self) -> u32 {
        let gis = unsafe { core::ptr::read_volatile(global_reg(GIS)) };
        let mut completed: u32 = 0;

        if gis & (1 << USART1_RX_CHANNEL) != 0 {
            let cis =
                unsafe { core::ptr::read_volatile(channel_reg(USART1_RX_CHANNEL, CIS)) };
            if cis & BIS != 0 {
                completed |= 1 << USART1_RX_CHANNEL;
            }
        }

        completed
    }
}
