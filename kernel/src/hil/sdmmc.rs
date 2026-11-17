// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Interface for a native-mode (SD bus, not SPI) SD memory card host.
//!
//! `Sdmmc` is deliberately **read-only**. The first user of this interface
//! inspects cards for deleted-but-recoverable content, and the only way to
//! guarantee that such a card is not altered is for no write path to exist at
//! all. Writing, and issuing arbitrary commands, live in the separate
//! `SdmmcWrite` trait, so a board has to opt in to them.
//!
//! Sequence: `initialize()` -> `init_done()`, then any number of
//! `read_blocks()` -> `read_done()` (and, with `SdmmcWrite`, `write_blocks()`
//! -> `write_done()` and `raw_command()` -> `raw_done()`). Only one operation
//! is outstanding at a time; starting another returns `BUSY`.

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
    /// 4-bit transfers failed and 1-bit ones worked, so the host fell back to
    /// a 1-bit bus: a contact or card fault on DAT1-3.
    pub bus_fallback: bool,
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
            bus_fallback: false,
        }
    }
}

/// Where an operation failed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(u8)]
pub enum ErrorStage {
    /// No failure recorded.
    #[default]
    None = 0,
    /// The command's response: timeout, CRC, index, end bit.
    Response = 1,
    /// The card answered but its R1 status reports an error.
    CardStatus = 2,
    /// The data phase: data timeout, data CRC, overrun.
    Data = 3,
    /// Waiting for the card to release DAT0 after a busy command.
    Busy = 4,
    /// The controller never reported completion at all.
    Stuck = 5,
}

/// Diagnostics for the most recent failed operation.
///
/// They tell "the read failed" apart from "the card is not answering" or
/// "the data lines are bad". Raw values, to be decoded against the
/// controller's status register and the SD R1 card-status bits.
#[derive(Clone, Copy, Debug, Default)]
pub struct ErrorInfo {
    /// Command index that failed (e.g. 17, 18, 13).
    pub cmd: u8,
    pub stage: ErrorStage,
    /// Host controller status register at the moment of failure.
    pub sr: u32,
    /// Last response word, the card's R1 status for R1 commands.
    pub r1: u32,
    /// Card state (R1 bits 12:9) from a CMD13 issued after the failure:
    /// 4 = tran, 5 = data, 6 = rcv, ...; 0xFF if CMD13 itself failed.
    pub card_state: u8,
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

    /// Why the most recent failed operation failed.
    fn last_error(&self) -> ErrorInfo;

    /// Why the SD Status read during initialization failed, if it did.
    fn init_error(&self) -> ErrorInfo;

    /// Change the bus width (1 or 4) and card clock of an initialized card,
    /// for marginal cards or contacts. Returns the clock actually set, which
    /// never exceeds `clock_hz`. Synchronous; fails with `BUSY` during a read.
    fn set_bus(&self, width: u8, clock_hz: u32) -> Result<u32, ErrorCode>;

    /// Set the data timeout, in ms (1-1000). The default is the SD
    /// specification's 100 ms. Shortening it trades the chance of calling a
    /// slow-but-good block bad for speed on a card with many bad blocks,
    /// each of which costs a full timeout.
    fn set_data_timeout(&self, ms: u32) -> Result<(), ErrorCode>;
}

/// Response format of a raw command (SD Physical Layer Simplified
/// Specification, section 4.9).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(u8)]
pub enum RawResponse {
    /// No response (CMD0, CMD4, CMD15).
    #[default]
    None = 0,
    /// 48 bits with CRC: R1, R6, R7.
    R1 = 1,
    /// R1 followed by busy signalling on DAT0 (CMD7, CMD12, CMD28, CMD38...).
    R1b = 2,
    /// 136 bits: R2 (CID, CSD).
    R2 = 3,
    /// 48 bits without CRC: R3 (OCR), R4.
    R3 = 4,
}

/// Data phase of a raw command.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(u8)]
pub enum RawData {
    #[default]
    None = 0,
    /// Card to host.
    Read = 1,
    /// Host to card.
    Write = 2,
}

/// One command, sent exactly as described: nothing is checked against the
/// card's state, no CMD55 prefix is added, and an error in the card's status
/// is reported rather than acted on.
#[derive(Clone, Copy, Debug, Default)]
pub struct RawCommand {
    /// Command index, 0-63.
    pub index: u8,
    /// Command argument.
    pub arg: u32,
    pub response: RawResponse,
    pub data: RawData,
    /// Bytes per data block (1-2048). Ignored without a data phase.
    pub block_len: u16,
    /// Data blocks (1-65535). More than one makes it a multiple-block
    /// transfer, which the card ends only on CMD12.
    pub block_count: u16,
    /// Treat the data phase as a multiple-block transfer, whatever
    /// `block_count`, and send CMD12 once it is done -- what CMD18 and CMD25
    /// need even for a single block.
    pub auto_stop: bool,
    /// Send the 74-clock initialization sequence instead of a command.
    pub init_clocks: bool,
}

/// What came back from a raw command.
#[derive(Clone, Copy, Debug, Default)]
pub struct RawResult {
    /// The response. A 48-bit response is the card's 32-bit payload in
    /// `resp[0]` (bits 39:8 of the response); an R2 response is bits
    /// 127:0 in `resp[0..4]`, most significant word first.
    pub resp: [u32; 4],
    /// Host controller status register when the command finished or failed.
    pub sr: u32,
    /// CMD12's R1, if `auto_stop` sent it.
    pub stop_resp: u32,
    /// Data bytes moved.
    pub bytes: u32,
}

/// Callbacks for `SdmmcWrite` operations.
pub trait SdmmcWriteClient {
    /// A write finished. `buffer` is returned either way.
    fn write_done(&self, buffer: &'static mut [u8], count: u32, result: Result<(), ErrorCode>);

    /// A raw command finished. For a read, the data is at the start of
    /// `buffer`. The result is filled in as far as the command got even when
    /// it failed: a response timeout leaves `resp` meaningless, but `sr`
    /// says so.
    fn raw_done(&self, buffer: &'static mut [u8], result: RawResult, status: Result<(), ErrorCode>);
}

/// Writes, and arbitrary commands, on an SD host that also implements
/// `Sdmmc`. Kept apart from `Sdmmc` so that a board exposes a write path
/// only by choice.
pub trait SdmmcWrite<'a> {
    fn set_write_client(&self, client: &'a dyn SdmmcWriteClient);

    /// Write `count` blocks from `buffer` starting at block `lba`. Fails with
    /// the same codes as `Sdmmc::read_blocks`. Success means the card
    /// reported the data programmed, not just received.
    fn write_blocks(
        &self,
        buffer: &'static mut [u8],
        lba: u32,
        count: u32,
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Issue one command, with an optional data phase through `buffer`
    /// (`block_len * block_count` bytes; for a write, the data to send).
    /// Allowed whether or not the card is initialized, so a card can be
    /// brought up by hand. The host's record of the card (`card_info`) is not
    /// updated: a command that changes the card's state or settings can leave
    /// later `read_blocks` failing until the next `initialize`.
    fn raw_command(
        &self,
        command: RawCommand,
        buffer: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])>;

    /// Set the host's bus width (1 or 4) and card clock without telling the
    /// card, to follow a width or speed change made with raw commands.
    /// Returns the clock actually set, which never exceeds `clock_hz`.
    fn set_host_bus(&self, width: u8, clock_hz: u32) -> Result<u32, ErrorCode>;
}
