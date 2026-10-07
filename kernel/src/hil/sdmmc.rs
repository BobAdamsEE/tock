// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Interface for a native-mode (SD bus, not SPI) SD memory card host.
//!
//! Deliberately **read-only**. The first user of this interface inspects cards
//! for deleted-but-recoverable content, and the only way to guarantee that such
//! a card is not altered is for no write path to exist at all. A write
//! extension belongs in a separate trait, so a board has to opt in to it.
//!
//! Sequence: `initialize()` -> `init_done()`, then any number of
//! `read_blocks()` -> `read_done()`. Only one operation is outstanding at a
//! time; starting another returns `BUSY`.

use crate::ErrorCode;

/// Size of one SD block. SDSC cards can be told to use other lengths; this
/// interface always uses 512, which is the only length SDHC/SDXC support.
pub const BLOCK_SIZE: usize = 512;

/// Everything learned about a card while bringing it up.
///
/// The register images are stored exactly as the card sent them, most
/// significant byte first, so they can be decoded against the SD Physical
/// Layer Simplified Specification bit tables without any reordering.
#[derive(Clone, Copy, Debug)]
pub struct CardInfo {
    /// Card Identification register, CID[127:0]. Byte 15 holds CRC7 and the
    /// always-one stop bit.
    pub cid: [u8; 16],
    /// Card Specific Data register, CSD[127:0].
    pub csd: [u8; 16],
    /// SD Configuration register, SCR[63:0]. All zero if it could not be read.
    pub scr: [u8; 8],
    /// Operating Conditions register, as returned by the final ACMD41.
    pub ocr: u32,
    /// Relative card address assigned by CMD3.
    pub rca: u16,
    /// Block-addressed (SDHC/SDXC) rather than byte-addressed (SDSC).
    pub high_capacity: bool,
    /// Answered CMD8, i.e. a Physical Layer 2.00 or later card.
    pub version2: bool,
    /// Bus width in use after initialization: 1 or 4.
    pub bus_width: u8,
    /// Card capacity in 512-byte blocks, from the CSD.
    pub block_count: u32,
    /// Card clock in use after initialization, in Hz.
    pub clock_hz: u32,
    /// SD Status (ACMD13), SSR[511:0]: speed class, UHS/video speed grades,
    /// application performance class, AU and erase parameters.
    pub ssr: [u8; 64],
    /// `ssr` was read successfully.
    pub ssr_valid: bool,
    /// CMD6 mode 0 (check) status, [511:0]: supported functions per group.
    pub switch_status: [u8; 64],
    /// `switch_status` was read successfully. False on cards without
    /// command class 10, which have no CMD6.
    pub switch_valid: bool,
}

// Written out because `Default` is not derived for arrays longer than 32.
impl Default for CardInfo {
    fn default() -> Self {
        CardInfo {
            cid: [0; 16],
            csd: [0; 16],
            scr: [0; 8],
            ocr: 0,
            rca: 0,
            high_capacity: false,
            version2: false,
            bus_width: 0,
            block_count: 0,
            clock_hz: 0,
            ssr: [0; 64],
            ssr_valid: false,
            switch_status: [0; 64],
            switch_valid: false,
        }
    }
}

/// Callbacks from an SD host.
pub trait SdmmcClient {
    /// Initialization finished. On success the card is selected, in transfer
    /// state, and ready for `read_blocks`.
    fn init_done(&self, result: Result<CardInfo, ErrorCode>);

    /// A read finished. `buffer` is returned either way; on success the first
    /// `count * BLOCK_SIZE` bytes hold the data.
    fn read_done(&self, buffer: &'static mut [u8], count: u32, result: Result<(), ErrorCode>);
}

/// A read-only SD memory card host.
pub trait Sdmmc<'a> {
    /// Set the client for completion callbacks.
    fn set_client(&self, client: &'a dyn SdmmcClient);

    /// Whether the card-detect switch reports a card. Advisory: a holder may
    /// have no switch, so a caller may still attempt `initialize`.
    fn is_card_present(&self) -> bool;

    /// Reset the host, identify the card and bring it to transfer state.
    fn initialize(&self) -> Result<(), ErrorCode>;

    /// The result of the last successful initialization, if there was one.
    fn card_info(&self) -> Option<CardInfo>;

    /// Read `count` consecutive blocks starting at block `lba` into `buffer`.
    ///
    /// `buffer` must hold at least `count * BLOCK_SIZE` bytes. Fails with
    /// `INVAL` if it does not or if the range runs past the end of the card,
    /// with `OFF` if the card has not been initialized, and with `BUSY` if an
    /// operation is already in progress.
    fn read_blocks(
        &self,
        buffer: &'static mut [u8],
        lba: u32,
        count: u32,
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;
}
