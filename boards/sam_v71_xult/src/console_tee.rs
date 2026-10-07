//! A UART that also mirrors everything it transmits to a second transmitter.
//!
//! The board's console and `debug!` output go to USART1 (the EDBG virtual COM
//! port) and, through this, to SEGGER RTT as well. RTT reaches the host over
//! the J-Link SWD connection, so console output stays visible when the EDBG
//! USB cable is not attached and the J-Link is the only thing connected.
//!
//! Transmission is sequential: the primary first, then the mirror with the
//! same buffer, then the client is called back with the primary's result. The
//! mirror cannot hold the primary up -- Tock's RTT copies into its ring
//! synchronously and runs in overwrite mode, so it never waits for a host.
//! Reception and configuration are the primary's alone.

use core::cell::Cell;

use kernel::hil::uart;
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;

#[derive(Clone, Copy, PartialEq)]
enum Stage {
    Idle,
    Primary,
    Mirror,
}

pub struct ConsoleTee<'a> {
    primary: &'a dyn uart::Uart<'a>,
    mirror: &'a dyn uart::Transmit<'a>,
    client: OptionalCell<&'a dyn uart::TransmitClient>,
    stage: Cell<Stage>,
    len: Cell<usize>,
    status: Cell<Result<(), ErrorCode>>,
}

impl<'a> ConsoleTee<'a> {
    pub fn new(primary: &'a dyn uart::Uart<'a>, mirror: &'a dyn uart::Transmit<'a>) -> Self {
        ConsoleTee {
            primary,
            mirror,
            client: OptionalCell::empty(),
            stage: Cell::new(Stage::Idle),
            len: Cell::new(0),
            status: Cell::new(Ok(())),
        }
    }

    fn finish(&self, buffer: &'static mut [u8]) {
        self.stage.set(Stage::Idle);
        let (len, status) = (self.len.get(), self.status.get());
        self.client.map(move |c| c.transmitted_buffer(buffer, len, status));
    }
}

impl<'a> uart::Transmit<'a> for ConsoleTee<'a> {
    fn set_transmit_client(&self, client: &'a dyn uart::TransmitClient) {
        self.client.set(client);
    }

    fn transmit_buffer(
        &self,
        tx_buffer: &'static mut [u8],
        tx_len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        if self.stage.get() != Stage::Idle {
            return Err((ErrorCode::BUSY, tx_buffer));
        }
        self.primary.transmit_buffer(tx_buffer, tx_len)?;
        self.len.set(tx_len);
        self.stage.set(Stage::Primary);
        Ok(())
    }

    fn transmit_word(&self, word: u32) -> Result<(), ErrorCode> {
        self.primary.transmit_word(word)
    }

    fn transmit_abort(&self) -> Result<(), ErrorCode> {
        self.primary.transmit_abort()
    }
}

impl uart::TransmitClient for ConsoleTee<'_> {
    fn transmitted_word(&self, rval: Result<(), ErrorCode>) {
        self.client.map(|c| c.transmitted_word(rval));
    }

    fn transmitted_buffer(
        &self,
        tx_buffer: &'static mut [u8],
        tx_len: usize,
        rval: Result<(), ErrorCode>,
    ) {
        match self.stage.get() {
            Stage::Primary => {
                self.len.set(tx_len);
                self.status.set(rval);
                self.stage.set(Stage::Mirror);
                if let Err((_, buf)) = self.mirror.transmit_buffer(tx_buffer, tx_len) {
                    self.finish(buf);
                }
            }
            Stage::Mirror => self.finish(tx_buffer),
            Stage::Idle => {}
        }
    }
}

impl<'a> uart::Receive<'a> for ConsoleTee<'a> {
    fn set_receive_client(&self, client: &'a dyn uart::ReceiveClient) {
        self.primary.set_receive_client(client);
    }

    fn receive_buffer(
        &self,
        rx_buffer: &'static mut [u8],
        rx_len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        self.primary.receive_buffer(rx_buffer, rx_len)
    }

    fn receive_word(&self) -> Result<(), ErrorCode> {
        self.primary.receive_word()
    }

    fn receive_abort(&self) -> Result<(), ErrorCode> {
        self.primary.receive_abort()
    }
}

impl uart::Configure for ConsoleTee<'_> {
    fn configure(&self, params: uart::Parameters) -> Result<(), ErrorCode> {
        self.primary.configure(params)
    }
}
