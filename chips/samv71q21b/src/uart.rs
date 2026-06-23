// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! USART1 driver for SAMV71Q21B.
//!
//! Wired to the EDBG USB-to-UART bridge on the SAMV71 Xplained Ultra:
//!   - RXD = PA21 (Peripheral A)
//!   - TXD = PB4  (Peripheral D)
//!   - Baud: CD = MCK/(16×baud) = 150_000_000/(16×115_200) = 81

use core::cell::Cell;

use kernel::hil;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use crate::xdmac::{self, Xdmac};

// ---------------------------------------------------------------------------
// Register layout (identical to SAM4L USART – same Microchip IP)
// ---------------------------------------------------------------------------

register_structs! {
    UsartRegisters {
        (0x000 => cr:   WriteOnly<u32, Cr::Register>),
        (0x004 => mr:   ReadWrite<u32, Mr::Register>),
        (0x008 => ier:  WriteOnly<u32, Ir::Register>),
        (0x00C => idr:  WriteOnly<u32, Ir::Register>),
        (0x010 => imr:  ReadOnly<u32,  Ir::Register>),
        (0x014 => csr:  ReadOnly<u32,  Csr::Register>),
        (0x018 => rhr:  ReadOnly<u32,  Rhr::Register>),
        (0x01C => thr:  WriteOnly<u32, Thr::Register>),
        (0x020 => brgr: ReadWrite<u32, Brgr::Register>),
        (0x024 => rtor: ReadWrite<u32, Rtor::Register>),
        (0x028 => _reserved),
        (0x100 => @END),
    }
}

register_bitfields![u32,
    Cr [
        RSTRX  OFFSET(2)  NUMBITS(1) [],
        RSTTX  OFFSET(3)  NUMBITS(1) [],
        RXEN   OFFSET(4)  NUMBITS(1) [],
        RXDIS  OFFSET(5)  NUMBITS(1) [],
        TXEN   OFFSET(6)  NUMBITS(1) [],
        TXDIS  OFFSET(7)  NUMBITS(1) [],
        RSTSTA OFFSET(8)  NUMBITS(1) [],
        STTTO  OFFSET(11) NUMBITS(1) [],
    ],
    Mr [
        USCLKS OFFSET(4) NUMBITS(2) [
            Mck    = 0,
            MckDiv = 1,
        ],
        CHRL OFFSET(6) NUMBITS(2) [
            Bits8 = 3,
        ],
        PAR OFFSET(9) NUMBITS(3) [
            Even  = 0,
            Odd   = 1,
            None  = 4,
        ],
        NBSTOP OFFSET(12) NUMBITS(2) [
            One = 0,
            Two = 2,
        ],
        OVER OFFSET(19) NUMBITS(1) [
            X16 = 0,
            X8  = 1,
        ],
    ],
    Ir [
        RXRDY   OFFSET(0) NUMBITS(1) [],
        TXRDY   OFFSET(1) NUMBITS(1) [],
        OVRE    OFFSET(5) NUMBITS(1) [],
        FRAME   OFFSET(6) NUMBITS(1) [],
        PARE    OFFSET(7) NUMBITS(1) [],
        TIMEOUT OFFSET(8) NUMBITS(1) [],
        TXEMPTY OFFSET(9) NUMBITS(1) [],
    ],
    Csr [
        RXRDY   OFFSET(0) NUMBITS(1) [],
        TXRDY   OFFSET(1) NUMBITS(1) [],
        OVRE    OFFSET(5) NUMBITS(1) [],
        FRAME   OFFSET(6) NUMBITS(1) [],
        PARE    OFFSET(7) NUMBITS(1) [],
        TIMEOUT OFFSET(8) NUMBITS(1) [],
        TXEMPTY OFFSET(9) NUMBITS(1) [],
    ],
    Rhr [
        RXCHR OFFSET(0) NUMBITS(9) [],
    ],
    Thr [
        TXCHR OFFSET(0) NUMBITS(9) [],
    ],
    Brgr [
        CD OFFSET(0)  NUMBITS(16) [],
        FP OFFSET(16) NUMBITS(3)  [],
    ],
    Rtor [
        TO OFFSET(0) NUMBITS(17) [],
    ],
];

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

const USART1_BASE: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(0x4002_8000 as *const UsartRegisters) };

/// USART1 peripheral ID (for PMC clock enable).
pub const USART1_PID: u32 = 14;

// MCK = 150 MHz after PMC setup.
const MCK_HZ: u32 = 150_000_000;

/// USART1 RHR physical address (source for DMA receive).
const USART1_RHR_ADDR: u32 = 0x4002_8018;

/// Multiplier applied to the interbyte_timeout value (u8) before
/// programming into the 17-bit RTOR register. A larger RTOR value
/// tolerates gaps introduced by the EDBG USB-CDC bridge between USB
/// packets, preventing premature timeout during multi-packet commands
/// like WritePage. With DMA receive the CPU is not in the data path,
/// so a moderate multiplier suffices.
const RTOR_MULTIPLIER: u32 = 5;

#[derive(Copy, Clone, PartialEq)]
enum TxState {
    Idle,
    Sending,
    WaitingEmpty,
}

#[derive(Copy, Clone, PartialEq)]
enum RxState {
    Idle,
    Receiving,
    ReceivingDma,
}

pub struct Usart1<'a> {
    regs: StaticRef<UsartRegisters>,
    tx_client: OptionalCell<&'a dyn hil::uart::TransmitClient>,
    rx_client: OptionalCell<&'a dyn hil::uart::ReceiveClient>,
    tx_buffer: TakeCell<'static, [u8]>,
    tx_len: Cell<usize>,
    tx_pos: Cell<usize>,
    tx_state: Cell<TxState>,
    rx_buffer: TakeCell<'static, [u8]>,
    rx_len: Cell<usize>,
    rx_pos: Cell<usize>,
    rx_state: Cell<RxState>,
    xdmac: OptionalCell<&'static Xdmac>,
}

impl<'a> Usart1<'a> {
    pub const fn new() -> Self {
        Usart1 {
            regs: USART1_BASE,
            tx_client: OptionalCell::empty(),
            rx_client: OptionalCell::empty(),
            tx_buffer: TakeCell::empty(),
            tx_len: Cell::new(0),
            tx_pos: Cell::new(0),
            tx_state: Cell::new(TxState::Idle),
            rx_buffer: TakeCell::empty(),
            rx_len: Cell::new(0),
            rx_pos: Cell::new(0),
            rx_state: Cell::new(RxState::Idle),
            xdmac: OptionalCell::empty(),
        }
    }

    pub fn set_xdmac(&self, xdmac: &'static Xdmac) {
        self.xdmac.set(xdmac);
    }

    pub fn handle_interrupt(&self) {
        let csr = self.regs.csr.extract();
        let imr = self.regs.imr.extract();

        // ---- DMA RX: overrun error ------------------------------------
        // With DMA active, RXRDY is not enabled as an interrupt; the
        // XDMAC reads RHR via hardware handshake. OVRE fires if the DMA
        // can't keep up (shouldn't happen) or if CUBC reached zero and
        // bytes keep arriving.
        if csr.is_set(Csr::OVRE) && self.rx_state.get() == RxState::ReceivingDma {
            self.regs.idr.write(Ir::TIMEOUT::SET + Ir::OVRE::SET);
            self.regs.cr.write(Cr::RSTSTA::SET);
            self.regs.rtor.write(Rtor::TO.val(0));
            self.xdmac.map(|x| x.disable_channel(xdmac::USART1_RX_CHANNEL));
            let received = self.xdmac.map_or(0, |x| x.usart1_rx_transferred() as usize);
            if let Some(buf) = self.rx_buffer.take() {
                self.rx_state.set(RxState::Idle);
                self.rx_client.map(|c| {
                    c.received_buffer(buf, received, Err(ErrorCode::FAIL), hil::uart::Error::OverrunError)
                });
            }
            return;
        }

        // ---- DMA RX: timeout (normal completion) ----------------------
        if csr.is_set(Csr::TIMEOUT) && imr.is_set(Ir::TIMEOUT)
            && self.rx_state.get() == RxState::ReceivingDma
        {
            self.regs.idr.write(Ir::TIMEOUT::SET + Ir::OVRE::SET);
            self.regs.rtor.write(Rtor::TO.val(0));
            self.xdmac.map(|x| x.disable_channel(xdmac::USART1_RX_CHANNEL));
            let received = self.xdmac.map_or(0, |x| x.usart1_rx_transferred() as usize);
            if let Some(buf) = self.rx_buffer.take() {
                self.rx_state.set(RxState::Idle);
                self.rx_client.map(|c| {
                    c.received_buffer(buf, received, Ok(()), hil::uart::Error::None)
                });
            }
            return;
        }

        // ---- Interrupt-driven RX errors -------------------------------
        if (csr.is_set(Csr::OVRE) || csr.is_set(Csr::FRAME) || csr.is_set(Csr::PARE))
            && imr.is_set(Ir::RXRDY)
        {
            self.regs.idr.write(Ir::RXRDY::SET + Ir::TIMEOUT::SET + Ir::OVRE::SET + Ir::FRAME::SET + Ir::PARE::SET);
            self.regs.cr.write(Cr::RSTSTA::SET);
            let error = if csr.is_set(Csr::OVRE) {
                hil::uart::Error::OverrunError
            } else if csr.is_set(Csr::FRAME) {
                hil::uart::Error::FramingError
            } else {
                hil::uart::Error::ParityError
            };
            if let Some(buf) = self.rx_buffer.take() {
                let pos = self.rx_pos.get();
                self.rx_state.set(RxState::Idle);
                self.rx_client
                    .map(|c| c.received_buffer(buf, pos, Err(ErrorCode::FAIL), error));
            }
            return;
        }

        // ---- Interrupt-driven RX timeout ------------------------------
        if csr.is_set(Csr::TIMEOUT) && imr.is_set(Ir::TIMEOUT) {
            self.regs.idr.write(Ir::RXRDY::SET + Ir::TIMEOUT::SET);
            self.regs.rtor.write(Rtor::TO.val(0));
            if let Some(buf) = self.rx_buffer.take() {
                let pos = self.rx_pos.get();
                self.rx_state.set(RxState::Idle);
                self.rx_client
                    .map(|c| c.received_buffer(buf, pos, Ok(()), hil::uart::Error::None));
            }
            return;
        }

        // ---- Interrupt-driven RX data ready ---------------------------
        if csr.is_set(Csr::RXRDY) && imr.is_set(Ir::RXRDY) {
            let byte = self.regs.rhr.read(Rhr::RXCHR) as u8;
            self.rx_buffer.map(|buf| {
                let pos = self.rx_pos.get();
                if pos < buf.len() {
                    buf[pos] = byte;
                    self.rx_pos.set(pos + 1);
                }
            });
            let pos = self.rx_pos.get();
            let len = self.rx_len.get();
            if pos >= len {
                self.regs.idr.write(Ir::RXRDY::SET + Ir::TIMEOUT::SET);
                self.regs.rtor.write(Rtor::TO.val(0));
                if let Some(buf) = self.rx_buffer.take() {
                    self.rx_state.set(RxState::Idle);
                    self.rx_client.map(|c| {
                        c.received_buffer(buf, pos, Ok(()), hil::uart::Error::None)
                    });
                }
            }
        }

        // ---- TX ready (send next byte) --------------------------------
        if csr.is_set(Csr::TXRDY) && imr.is_set(Ir::TXRDY) {
            let pos = self.tx_pos.get();
            let len = self.tx_len.get();
            if pos < len {
                self.tx_buffer.map(|buf| {
                    self.regs.thr.write(Thr::TXCHR.val(buf[pos] as u32));
                });
                self.tx_pos.set(pos + 1);
                if self.tx_pos.get() >= len {
                    // Last byte written; wait for shift register empty.
                    self.regs.idr.write(Ir::TXRDY::SET);
                    self.regs.ier.write(Ir::TXEMPTY::SET);
                    self.tx_state.set(TxState::WaitingEmpty);
                }
            }
        }

        // ---- TX shift register empty (done) ---------------------------
        if csr.is_set(Csr::TXEMPTY) && imr.is_set(Ir::TXEMPTY) {
            self.regs.idr.write(Ir::TXEMPTY::SET);
            if let Some(buf) = self.tx_buffer.take() {
                let len = self.tx_len.get();
                self.tx_state.set(TxState::Idle);
                self.tx_client
                    .map(|c| c.transmitted_buffer(buf, len, Ok(())));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// HIL: Configure
// ---------------------------------------------------------------------------

impl<'a> hil::uart::Configure for Usart1<'a> {
    fn configure(&self, params: hil::uart::Parameters) -> Result<(), ErrorCode> {
        // Reset and disable TX/RX.
        self.regs.cr.write(Cr::RSTRX::SET + Cr::RSTTX::SET + Cr::RXDIS::SET + Cr::TXDIS::SET);

        // Mode: normal USART, MCK clock, 8-bit, selected parity, stop bits, 16× oversampling.
        let par = match params.parity {
            hil::uart::Parity::None => Mr::PAR::None,
            hil::uart::Parity::Even => Mr::PAR::Even,
            hil::uart::Parity::Odd  => Mr::PAR::Odd,
        };
        let stop = match params.stop_bits {
            hil::uart::StopBits::One => Mr::NBSTOP::One,
            hil::uart::StopBits::Two => Mr::NBSTOP::Two,
        };
        self.regs.mr.write(Mr::USCLKS::Mck + Mr::CHRL::Bits8 + par + stop + Mr::OVER::X16);

        // Baud rate generator: CD = MCK / (16 × baud_rate).
        let cd = MCK_HZ / (16 * params.baud_rate);
        self.regs.brgr.write(Brgr::CD.val(cd));

        // Clear status flags and enable TX + RX.
        self.regs.cr.write(Cr::RSTSTA::SET + Cr::RXEN::SET + Cr::TXEN::SET);

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// HIL: Transmit
// ---------------------------------------------------------------------------

impl<'a> hil::uart::Transmit<'a> for Usart1<'a> {
    fn set_transmit_client(&self, client: &'a dyn hil::uart::TransmitClient) {
        self.tx_client.set(client);
    }

    fn transmit_buffer(
        &self,
        tx_buffer: &'static mut [u8],
        tx_len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        if self.tx_state.get() != TxState::Idle {
            return Err((ErrorCode::BUSY, tx_buffer));
        }
        if tx_len == 0 || tx_len > tx_buffer.len() {
            return Err((ErrorCode::SIZE, tx_buffer));
        }
        self.tx_len.set(tx_len);
        self.tx_pos.set(0);
        self.tx_state.set(TxState::Sending);
        self.tx_buffer.replace(tx_buffer);
        // Enable TXRDY interrupt; first byte goes in handle_interrupt.
        self.regs.ier.write(Ir::TXRDY::SET);
        Ok(())
    }

    fn transmit_word(&self, _word: u32) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }

    fn transmit_abort(&self) -> Result<(), ErrorCode> {
        if self.tx_state.get() == TxState::Idle {
            return Ok(());
        }
        self.regs.idr.write(Ir::TXRDY::SET + Ir::TXEMPTY::SET);
        self.tx_state.set(TxState::Idle);
        if let Some(buf) = self.tx_buffer.take() {
            let len = self.tx_pos.get();
            self.tx_client
                .map(|c| c.transmitted_buffer(buf, len, Err(ErrorCode::CANCEL)));
        }
        Err(ErrorCode::BUSY)
    }
}

// ---------------------------------------------------------------------------
// HIL: Receive
// ---------------------------------------------------------------------------

impl<'a> hil::uart::Receive<'a> for Usart1<'a> {
    fn set_receive_client(&self, client: &'a dyn hil::uart::ReceiveClient) {
        self.rx_client.set(client);
    }

    fn receive_buffer(
        &self,
        rx_buffer: &'static mut [u8],
        rx_len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        if self.rx_state.get() != RxState::Idle {
            return Err((ErrorCode::BUSY, rx_buffer));
        }
        if rx_len == 0 || rx_len > rx_buffer.len() {
            return Err((ErrorCode::SIZE, rx_buffer));
        }
        self.rx_len.set(rx_len);
        self.rx_pos.set(0);
        self.rx_state.set(RxState::Receiving);
        self.rx_buffer.replace(rx_buffer);
        self.regs.ier.write(Ir::RXRDY::SET + Ir::OVRE::SET + Ir::FRAME::SET + Ir::PARE::SET);
        Ok(())
    }

    fn receive_word(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }

    fn receive_abort(&self) -> Result<(), ErrorCode> {
        let state = self.rx_state.get();
        if state == RxState::Idle {
            return Ok(());
        }
        if state == RxState::ReceivingDma {
            self.xdmac.map(|x| x.disable_channel(xdmac::USART1_RX_CHANNEL));
            self.regs.idr.write(Ir::TIMEOUT::SET + Ir::OVRE::SET);
            let received = self.xdmac.map_or(0, |x| x.usart1_rx_transferred() as usize);
            self.regs.rtor.write(Rtor::TO.val(0));
            self.rx_state.set(RxState::Idle);
            if let Some(buf) = self.rx_buffer.take() {
                self.rx_client
                    .map(|c| c.received_buffer(buf, received, Err(ErrorCode::CANCEL), hil::uart::Error::Aborted));
            }
        } else {
            self.regs.idr.write(Ir::RXRDY::SET + Ir::TIMEOUT::SET + Ir::OVRE::SET + Ir::FRAME::SET + Ir::PARE::SET);
            self.regs.rtor.write(Rtor::TO.val(0));
            self.rx_state.set(RxState::Idle);
            if let Some(buf) = self.rx_buffer.take() {
                let pos = self.rx_pos.get();
                self.rx_client
                    .map(|c| c.received_buffer(buf, pos, Err(ErrorCode::CANCEL), hil::uart::Error::Aborted));
            }
        }
        Err(ErrorCode::BUSY)
    }
}

// ---------------------------------------------------------------------------
// HIL: ReceiveAdvanced (interbyte timeout via US_RTOR)
// ---------------------------------------------------------------------------

impl<'a> hil::uart::ReceiveAdvanced<'a> for Usart1<'a> {
    fn receive_automatic(
        &self,
        rx_buffer: &'static mut [u8],
        rx_len: usize,
        interbyte_timeout: u8,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        if self.rx_state.get() != RxState::Idle {
            return Err((ErrorCode::BUSY, rx_buffer));
        }
        if rx_len == 0 || rx_len > rx_buffer.len() {
            return Err((ErrorCode::SIZE, rx_buffer));
        }
        self.rx_len.set(rx_len);
        self.rx_pos.set(0);

        // Use DMA receive when XDMAC is available. The XDMAC reads
        // bytes from USART1 RHR directly into the buffer via hardware
        // handshake, eliminating per-byte CPU interrupts. The USART
        // RTOR timeout still fires after the last byte, at which point
        // we stop the DMA and deliver the buffer.
        if let Some(xdmac) = self.xdmac.get() {
            let dst_addr = rx_buffer.as_ptr() as u32;
            self.rx_state.set(RxState::ReceivingDma);
            self.rx_buffer.replace(rx_buffer);

            xdmac.configure_periph_to_mem(
                xdmac::USART1_RX_CHANNEL,
                xdmac::USART1_RX_PERID,
                USART1_RHR_ADDR,
                dst_addr,
                rx_len as u32,
            );
            xdmac.enable_channel(xdmac::USART1_RX_CHANNEL);

            let to = interbyte_timeout as u32 * RTOR_MULTIPLIER;
            self.regs.rtor.write(Rtor::TO.val(to));
            self.regs.cr.write(Cr::STTTO::SET);
            self.regs.ier.write(Ir::TIMEOUT::SET + Ir::OVRE::SET);
        } else {
            self.rx_state.set(RxState::Receiving);
            self.rx_buffer.replace(rx_buffer);
            self.regs.rtor.write(Rtor::TO.val(interbyte_timeout as u32));
            self.regs.cr.write(Cr::STTTO::SET);
            self.regs
                .ier
                .write(Ir::RXRDY::SET + Ir::TIMEOUT::SET + Ir::OVRE::SET + Ir::FRAME::SET + Ir::PARE::SET);
        }
        Ok(())
    }
}
