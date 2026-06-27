// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! TWIHS (Two-Wire Interface High Speed) I2C master driver for SAMV71Q21B.
//!
//! Interrupt-driven, byte-by-byte I2C master implementing Tock's
//! `kernel::hil::i2c::I2CMaster` trait.  Supports TWIHS0, TWIHS1,
//! and TWIHS2 instances.
//!
//! Pin assignments (from datasheet / board schematic):
//!   - TWIHS0: PA3 = TWD0 (SDA), PA4 = TWCK0 (SCL) — Peripheral A
//!   - TWIHS1: PB4 = TWD1 (SDA), PB5 = TWCK1 (SCL) — Peripheral A
//!   - TWIHS2: PD27 = TWD2 (SDA), PD28 = TWCK2 (SCL) — Peripheral C
//!
//! Register map: SAMV71 datasheet §43, Table 43-3.
//!
//! Base addresses:
//!   TWIHS0 = 0x4001_8000  (PID 19)
//!   TWIHS1 = 0x4001_C000  (PID 20)
//!   TWIHS2 = 0x4006_0000  (PID 41)

use core::cell::Cell;

use kernel::hil;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;

// ---------------------------------------------------------------------------
// Register layout — SAMV71 TWIHS (§43, Table 43-3)
// ---------------------------------------------------------------------------

register_structs! {
    TwihsRegisters {
        (0x00 => cr:    WriteOnly<u32,  Cr::Register>),
        (0x04 => mmr:   ReadWrite<u32,  Mmr::Register>),
        (0x08 => smr:   ReadWrite<u32>),
        (0x0C => iadr:  ReadWrite<u32,  Iadr::Register>),
        (0x10 => cwgr:  ReadWrite<u32,  Cwgr::Register>),
        (0x14 => _reserved0),
        (0x20 => sr:    ReadOnly<u32,   Sr::Register>),
        (0x24 => ier:   WriteOnly<u32,  Sr::Register>),
        (0x28 => idr:   WriteOnly<u32,  Sr::Register>),
        (0x2C => imr:   ReadOnly<u32,   Sr::Register>),
        (0x30 => rhr:   ReadOnly<u32,   Rhr::Register>),
        (0x34 => thr:   WriteOnly<u32,  Thr::Register>),
        (0x38 => smbtr: ReadWrite<u32>),
        (0x3C => _reserved1),
        (0x44 => filtr: ReadWrite<u32,  Filtr::Register>),
        (0x48 => _reserved2),
        (0x4C => swmr:  ReadWrite<u32>),
        (0x50 => _reserved3),
        (0xE4 => wpmr:  ReadWrite<u32,  Wpmr::Register>),
        (0xE8 => wpsr:  ReadOnly<u32>),
        (0xEC => @END),
    }
}

register_bitfields![u32,
    Cr [
        START   OFFSET(0)  NUMBITS(1) [],
        STOP    OFFSET(1)  NUMBITS(1) [],
        MSEN    OFFSET(2)  NUMBITS(1) [],
        MSDIS   OFFSET(3)  NUMBITS(1) [],
        SVEN    OFFSET(4)  NUMBITS(1) [],
        SVDIS   OFFSET(5)  NUMBITS(1) [],
        QUICK   OFFSET(6)  NUMBITS(1) [],
        SWRST   OFFSET(7)  NUMBITS(1) [],
        HSEN    OFFSET(8)  NUMBITS(1) [],
        HSDIS   OFFSET(9)  NUMBITS(1) [],
        SMBEN   OFFSET(10) NUMBITS(1) [],
        SMBDIS  OFFSET(11) NUMBITS(1) [],
        PECEN   OFFSET(12) NUMBITS(1) [],
        PECDIS  OFFSET(13) NUMBITS(1) [],
        PECRQ   OFFSET(14) NUMBITS(1) [],
        CLEAR   OFFSET(15) NUMBITS(1) [],
        ACMEN   OFFSET(16) NUMBITS(1) [],
        ACMDIS  OFFSET(17) NUMBITS(1) [],
        THRCLR  OFFSET(24) NUMBITS(1) [],
        LOCKCLR OFFSET(26) NUMBITS(1) [],
        FIFOEN  OFFSET(28) NUMBITS(1) [],
        FIFODIS OFFSET(29) NUMBITS(1) [],
    ],
    Mmr [
        IADRSZ OFFSET(8)  NUMBITS(2) [
            None     = 0,
            OneByte  = 1,
            TwoBytes = 2,
            ThreeBytes = 3,
        ],
        MREAD  OFFSET(12) NUMBITS(1) [],
        DADR   OFFSET(16) NUMBITS(7) [],
    ],
    Iadr [
        IADR OFFSET(0) NUMBITS(24) [],
    ],
    Cwgr [
        CLDIV  OFFSET(0)  NUMBITS(8) [],
        CHDIV  OFFSET(8)  NUMBITS(8) [],
        CKDIV  OFFSET(16) NUMBITS(3) [],
        HOLD   OFFSET(24) NUMBITS(6) [],
    ],
    Sr [
        TXCOMP OFFSET(0)  NUMBITS(1) [],
        RXRDY  OFFSET(1)  NUMBITS(1) [],
        TXRDY  OFFSET(2)  NUMBITS(1) [],
        SVREAD OFFSET(3)  NUMBITS(1) [],
        SVACC  OFFSET(4)  NUMBITS(1) [],
        GACC   OFFSET(5)  NUMBITS(1) [],
        OVRE   OFFSET(6)  NUMBITS(1) [],
        UNRE   OFFSET(7)  NUMBITS(1) [],
        NACK   OFFSET(8)  NUMBITS(1) [],
        ARBLST OFFSET(9)  NUMBITS(1) [],
        SCLWS  OFFSET(10) NUMBITS(1) [],
        EOSACC OFFSET(11) NUMBITS(1) [],
        MCACK  OFFSET(16) NUMBITS(1) [],
        TOUT   OFFSET(18) NUMBITS(1) [],
        PECERR OFFSET(19) NUMBITS(1) [],
        SMBDAM OFFSET(20) NUMBITS(1) [],
        SMBHHM OFFSET(21) NUMBITS(1) [],
        SCL    OFFSET(24) NUMBITS(1) [],
        SDA    OFFSET(25) NUMBITS(1) [],
    ],
    Rhr [
        RXDATA OFFSET(0) NUMBITS(8) [],
    ],
    Thr [
        TXDATA OFFSET(0) NUMBITS(8) [],
    ],
    Filtr [
        FILT   OFFSET(0)  NUMBITS(1) [],
        PADFEN OFFSET(1)  NUMBITS(1) [],
        PADFCFG OFFSET(2) NUMBITS(1) [],
        THRES  OFFSET(8)  NUMBITS(3) [],
    ],
    Wpmr [
        WPEN  OFFSET(0)  NUMBITS(1)  [],
        WPKEY OFFSET(8)  NUMBITS(24) [],
    ],
];

// ---------------------------------------------------------------------------
// Base addresses and peripheral IDs
// ---------------------------------------------------------------------------

const TWIHS0_BASE: StaticRef<TwihsRegisters> =
    unsafe { StaticRef::new(0x4001_8000 as *const TwihsRegisters) };

const TWIHS1_BASE: StaticRef<TwihsRegisters> =
    unsafe { StaticRef::new(0x4001_C000 as *const TwihsRegisters) };

const TWIHS2_BASE: StaticRef<TwihsRegisters> =
    unsafe { StaticRef::new(0x4006_0000 as *const TwihsRegisters) };

pub const TWIHS0_PID: u32 = 19;
pub const TWIHS1_PID: u32 = 20;
pub const TWIHS2_PID: u32 = 41;

// Write-protection key: ASCII "TWI" = 0x545749
const WP_KEY: u32 = 0x54_57_49;

// MCK = 150 MHz (after PMC setup in main.rs).
const MCK_HZ: u32 = 150_000_000;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, PartialEq)]
enum State {
    Idle,
    /// Pure write: sending `len` bytes from the buffer.
    Writing { len: usize, pos: usize },
    /// Write phase of a write-then-read: sending `write_len` bytes,
    /// then switching to read of `read_len` bytes.
    WriteReading {
        write_len: usize,
        write_pos: usize,
        read_len: usize,
    },
    /// Read phase (either standalone read or second half of write_read).
    Reading { len: usize, pos: usize },
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Twihs<'a> {
    regs: StaticRef<TwihsRegisters>,
    state: Cell<State>,
    addr: Cell<u8>,
    buffer: TakeCell<'static, [u8]>,
    client: OptionalCell<&'a dyn hil::i2c::I2CHwMasterClient>,
}

impl<'a> Twihs<'a> {
    pub const fn new_twihs0() -> Self {
        Self::new(TWIHS0_BASE)
    }

    pub const fn new_twihs1() -> Self {
        Self::new(TWIHS1_BASE)
    }

    pub const fn new_twihs2() -> Self {
        Self::new(TWIHS2_BASE)
    }

    const fn new(regs: StaticRef<TwihsRegisters>) -> Self {
        Twihs {
            regs,
            state: Cell::new(State::Idle),
            addr: Cell::new(0),
            buffer: TakeCell::empty(),
            client: OptionalCell::empty(),
        }
    }

    /// Configure the bus clock for the requested frequency.
    ///
    /// From §43.8.5:  `CLDIV × 2^CKDIV = (f_mck / (2 × f_twi)) − 3`
    /// where CLDIV = CHDIV (symmetric clock).
    pub fn set_speed(&self, freq_hz: u32) {
        // Write protection covers MMR, CWGR, and other config registers
        // (datasheet Table 43-2).  Keep it disabled so that later MMR
        // writes in write()/read()/write_read() are not silently dropped.
        self.regs.wpmr.write(Wpmr::WPKEY.val(WP_KEY) + Wpmr::WPEN::CLEAR);

        let target = MCK_HZ / (2 * freq_hz);
        let mut ckdiv: u32 = 0;
        let mut div = target.saturating_sub(3);
        while div > 255 && ckdiv < 7 {
            ckdiv += 1;
            div /= 2;
        }

        self.regs.cwgr.write(
            Cwgr::CLDIV.val(div)
                + Cwgr::CHDIV.val(div)
                + Cwgr::CKDIV.val(ckdiv)
                + Cwgr::HOLD.val(0),
        );
    }

    fn disable_all_interrupts(&self) {
        self.regs.idr.set(0xFFFF_FFFF);
    }

    fn enable_error_interrupts(&self) {
        self.regs.ier.write(
            Sr::NACK::SET + Sr::ARBLST::SET + Sr::OVRE::SET,
        );
    }

    fn reset(&self) {
        self.regs.cr.write(Cr::SWRST::SET);
        self.regs.cr.write(Cr::SVDIS::SET + Cr::MSEN::SET);
    }

    fn finish(&self, status: Result<(), hil::i2c::Error>) {
        self.disable_all_interrupts();
        self.state.set(State::Idle);
        self.client.map(|client| {
            self.buffer.take().map(|buf| {
                client.command_complete(buf, status);
            });
        });
    }

    fn finish_with_error(&self, error: hil::i2c::Error) {
        self.reset();
        self.finish(Err(error));
    }

    // -----------------------------------------------------------------------
    // Interrupt handler
    // -----------------------------------------------------------------------

    pub fn handle_interrupt(&self) {
        let sr = self.regs.sr.extract();
        let imr = self.regs.imr.extract();

        // Only consider enabled interrupts.
        let active = sr.get() & imr.get();
        if active == 0 {
            return;
        }

        // --- Error checks (highest priority) ---
        if sr.is_set(Sr::NACK) {
            self.finish_with_error(hil::i2c::Error::AddressNak);
            return;
        }
        if sr.is_set(Sr::ARBLST) {
            self.finish_with_error(hil::i2c::Error::ArbitrationLost);
            return;
        }
        if sr.is_set(Sr::OVRE) {
            self.finish_with_error(hil::i2c::Error::Overrun);
            return;
        }

        match self.state.get() {
            // ---- Writing (pure write) ----
            State::Writing { len, pos } => {
                if sr.is_set(Sr::TXRDY) {
                    if pos < len {
                        // Send next byte.
                        self.buffer.map(|buf| {
                            self.regs.thr.write(Thr::TXDATA.val(buf[pos] as u32));
                        });
                        self.state.set(State::Writing { len, pos: pos + 1 });
                    } else {
                        // All bytes sent — issue STOP and wait for TXCOMP.
                        self.regs.cr.write(Cr::STOP::SET);
                        self.disable_all_interrupts();
                        self.regs.ier.write(Sr::TXCOMP::SET);
                        self.enable_error_interrupts();
                    }
                }
                if sr.is_set(Sr::TXCOMP) {
                    self.finish(Ok(()));
                }
            }

            // ---- Write phase of write_read ----
            State::WriteReading {
                write_len,
                write_pos,
                read_len,
            } => {
                if sr.is_set(Sr::TXRDY) {
                    if write_pos < write_len {
                        self.buffer.map(|buf| {
                            self.regs.thr.write(Thr::TXDATA.val(buf[write_pos] as u32));
                        });
                        self.state.set(State::WriteReading {
                            write_len,
                            write_pos: write_pos + 1,
                            read_len,
                        });
                    } else {
                        // Write phase done — switch to read.
                        self.disable_all_interrupts();
                        self.regs.mmr.modify(
                            Mmr::DADR.val(self.addr.get() as u32) + Mmr::MREAD::SET,
                        );
                        // Issue repeated START (and STOP if only 1 byte to read).
                        if read_len == 1 {
                            self.regs.cr.write(Cr::START::SET + Cr::STOP::SET);
                        } else {
                            self.regs.cr.write(Cr::START::SET);
                        }
                        self.state.set(State::Reading { len: read_len, pos: 0 });
                        self.regs.ier.write(Sr::RXRDY::SET);
                        self.enable_error_interrupts();
                    }
                }
            }

            // ---- Reading ----
            State::Reading { len, pos } => {
                if sr.is_set(Sr::RXRDY) {
                    let byte = self.regs.rhr.read(Rhr::RXDATA) as u8;
                    self.buffer.map(|buf| {
                        buf[pos] = byte;
                    });
                    let next = pos + 1;

                    if next == len {
                        // Last byte received — done.
                        self.disable_all_interrupts();
                        self.regs.ier.write(Sr::TXCOMP::SET);
                        self.enable_error_interrupts();
                        self.state.set(State::Reading { len, pos: next });
                    } else {
                        if next == len - 1 {
                            // Penultimate byte — issue STOP before reading last.
                            self.regs.cr.write(Cr::STOP::SET);
                        }
                        self.state.set(State::Reading { len, pos: next });
                    }
                }
                if sr.is_set(Sr::TXCOMP) {
                    self.finish(Ok(()));
                }
            }

            State::Idle => {}
        }
    }
}

// ---------------------------------------------------------------------------
// I2CMaster implementation
// ---------------------------------------------------------------------------

impl<'a> hil::i2c::I2CMaster<'a> for Twihs<'a> {
    fn set_master_client(&self, client: &'a dyn hil::i2c::I2CHwMasterClient) {
        self.client.set(client);
    }

    fn enable(&self) {
        self.regs.cr.write(Cr::SWRST::SET);
        self.set_speed(400_000);
        self.regs.cr.write(Cr::SVDIS::SET + Cr::MSEN::SET);
    }

    fn disable(&self) {
        self.disable_all_interrupts();
        self.regs.cr.write(Cr::MSDIS::SET);
    }

    fn write(
        &self,
        addr: u8,
        data: &'static mut [u8],
        len: usize,
    ) -> Result<(), (hil::i2c::Error, &'static mut [u8])> {
        if self.state.get() != State::Idle {
            return Err((hil::i2c::Error::Busy, data));
        }
        if len == 0 || len > data.len() {
            return Err((hil::i2c::Error::Overrun, data));
        }

        self.addr.set(addr);
        // MMR: write direction, 7-bit address, no internal address.
        self.regs.mmr.write(
            Mmr::DADR.val(addr as u32) + Mmr::MREAD::CLEAR + Mmr::IADRSZ::None,
        );

        // The first byte written to THR triggers the START automatically.
        let first = data[0];
        self.buffer.replace(data);
        self.state.set(State::Writing { len, pos: 1 });

        self.disable_all_interrupts();
        self.regs.thr.write(Thr::TXDATA.val(first as u32));
        self.regs.ier.write(Sr::TXRDY::SET);
        self.enable_error_interrupts();

        Ok(())
    }

    fn read(
        &self,
        addr: u8,
        buffer: &'static mut [u8],
        len: usize,
    ) -> Result<(), (hil::i2c::Error, &'static mut [u8])> {
        if self.state.get() != State::Idle {
            return Err((hil::i2c::Error::Busy, buffer));
        }
        if len == 0 || len > buffer.len() {
            return Err((hil::i2c::Error::Overrun, buffer));
        }

        self.addr.set(addr);
        self.regs.mmr.write(
            Mmr::DADR.val(addr as u32) + Mmr::MREAD::SET + Mmr::IADRSZ::None,
        );

        self.buffer.replace(buffer);
        self.state.set(State::Reading { len, pos: 0 });

        self.disable_all_interrupts();
        if len == 1 {
            self.regs.cr.write(Cr::START::SET + Cr::STOP::SET);
        } else {
            self.regs.cr.write(Cr::START::SET);
        }
        self.regs.ier.write(Sr::RXRDY::SET);
        self.enable_error_interrupts();

        Ok(())
    }

    fn write_read(
        &self,
        addr: u8,
        data: &'static mut [u8],
        write_len: usize,
        read_len: usize,
    ) -> Result<(), (hil::i2c::Error, &'static mut [u8])> {
        if self.state.get() != State::Idle {
            return Err((hil::i2c::Error::Busy, data));
        }
        if write_len == 0 || read_len == 0 {
            return Err((hil::i2c::Error::Overrun, data));
        }
        let needed = core::cmp::max(write_len, read_len);
        if needed > data.len() {
            return Err((hil::i2c::Error::Overrun, data));
        }

        self.addr.set(addr);

        if write_len <= 3 {
            // Use the TWIHS internal address register (IADR) — the
            // hardware generates START + addr_W + IADR bytes + repeated
            // START + addr_R automatically, then clocks in data.
            let iadrsz = match write_len {
                1 => Mmr::IADRSZ::OneByte,
                2 => Mmr::IADRSZ::TwoBytes,
                _ => Mmr::IADRSZ::ThreeBytes,
            };

            let mut iadr: u32 = 0;
            for i in 0..write_len {
                iadr = (iadr << 8) | data[i] as u32;
            }

            self.regs.mmr.write(
                Mmr::DADR.val(addr as u32) + Mmr::MREAD::SET + iadrsz,
            );
            self.regs.iadr.write(Iadr::IADR.val(iadr));

            self.buffer.replace(data);
            self.state.set(State::Reading { len: read_len, pos: 0 });

            self.disable_all_interrupts();
            if read_len == 1 {
                self.regs.cr.write(Cr::START::SET + Cr::STOP::SET);
            } else {
                self.regs.cr.write(Cr::START::SET);
            }
            self.regs.ier.write(Sr::RXRDY::SET);
            self.enable_error_interrupts();
        } else {
            // For write phases longer than 3 bytes, send them manually
            // via THR, then transition to read in the interrupt handler.
            self.regs.mmr.write(
                Mmr::DADR.val(addr as u32) + Mmr::MREAD::CLEAR + Mmr::IADRSZ::None,
            );

            let first = data[0];
            self.buffer.replace(data);
            self.state.set(State::WriteReading {
                write_len,
                write_pos: 1,
                read_len,
            });

            self.disable_all_interrupts();
            self.regs.thr.write(Thr::TXDATA.val(first as u32));
            self.regs.ier.write(Sr::TXRDY::SET);
            self.enable_error_interrupts();
        }

        Ok(())
    }
}
