
// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Driver for the AT24MAC402 I2C EEPROM (2 Kbit / 256 bytes, 16-byte pages)
//! with factory-programmed EUI-48 MAC address and 128-bit serial number.
//!
//! Datasheet:
//! <https://ww1.microchip.com/downloads/en/DeviceDoc/AT24MAC402-602-I2C-Compatible-Two-Wire-Serial-EEPROM-with-EUI-48-or-EUI-64-Node-Identity-20002735C.pdf>
//!
//! This capsule uses **two** virtualized I2C devices:
//!
//! | Device   | 7-bit addr | Purpose                                |
//! |----------|------------|----------------------------------------|
//! | `i2c`    | 0x57       | User EEPROM (256 bytes, R/W)           |
//! | `i2c_ext`| 0x5F       | Extended block (MAC + serial, R/O)     |
//!
//! ## Metadata layout in the extended block
//!
//! | Word address | Length | Content                  |
//! |--------------|--------|--------------------------|
//! | 0x80         | 16     | 128-bit serial number    |
//! | 0x9A         | 6      | EUI-48 MAC address       |
//!
//! ## Usage
//!
//! ```rust,ignore
//! // Two virtualized I2C devices on the same bus:
//! let eeprom_i2c     = I2CComponent::new(mux_i2c, 0x57).finalize(i2c_component_static!(...));
//! let eeprom_i2c_ext = I2CComponent::new(mux_i2c, 0x5F).finalize(i2c_component_static!(...));
//!
//! let eeprom_buf = static_init!([u8; 18], [0; 18]);
//! let eeprom = static_init!(
//!     At24Mac402<'static>,
//!     At24Mac402::new(eeprom_i2c, eeprom_i2c_ext, eeprom_buf)
//! );
//! eeprom_i2c.set_client(eeprom);
//! eeprom_i2c_ext.set_client(eeprom);
//! ```

use core::cell::Cell;
use core::cmp;

use kernel::hil::i2c::{Error, I2CClient, I2CDevice};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::{hil, ErrorCode};

const PAGE_SIZE: usize = 16;
const EEPROM_SIZE: usize = 256;
const EUI48_LEN: usize = 6;
const SERIAL_LEN: usize = 16;

const EXT_SERIAL_ADDR: u8 = 0x80;
const EXT_EUI48_ADDR: u8 = 0x9A;

pub struct EEPROMPage(pub [u8; PAGE_SIZE]);

impl Default for EEPROMPage {
    fn default() -> Self {
        Self([0; PAGE_SIZE])
    }
}

impl AsMut<[u8]> for EEPROMPage {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum State {
    Idle,
    /// Reading a page from user EEPROM (via `i2c` at 0x57).
    ReadingPage,
    /// Writing a page to user EEPROM.
    WritingPage,
    /// Erasing (writing zeros to) a page.
    ErasingPage,
    /// Reading the EUI-48 MAC from the extended block (via `i2c_ext` at 0x5F).
    ReadingMac,
    /// Reading the 128-bit serial from the extended block.
    ReadingSerial,
}

/// Client trait for metadata queries (MAC address, serial number).
pub trait At24Mac402Client {
    fn mac_read_complete(&self, mac: &[u8; EUI48_LEN], status: Result<(), ErrorCode>);
    fn serial_read_complete(&self, serial: &[u8; SERIAL_LEN], status: Result<(), ErrorCode>);
}

pub struct At24Mac402<'a> {
    i2c: &'a dyn I2CDevice,
    i2c_ext: &'a dyn I2CDevice,
    buffer: TakeCell<'static, [u8]>,
    client_page: TakeCell<'a, EEPROMPage>,
    flash_client: OptionalCell<&'a dyn hil::flash::Client<At24Mac402<'a>>>,
    meta_client: OptionalCell<&'a dyn At24Mac402Client>,
    state: Cell<State>,
}

impl<'a> At24Mac402<'a> {
    /// Create a new AT24MAC402 capsule.
    ///
    /// `buffer` must be at least 18 bytes (1 address byte + PAGE_SIZE data,
    /// or 1 address byte + SERIAL_LEN for metadata reads).
    pub fn new(
        i2c: &'a dyn I2CDevice,
        i2c_ext: &'a dyn I2CDevice,
        buffer: &'static mut [u8],
    ) -> Self {
        Self {
            i2c,
            i2c_ext,
            buffer: TakeCell::new(buffer),
            client_page: TakeCell::empty(),
            flash_client: OptionalCell::empty(),
            meta_client: OptionalCell::empty(),
            state: Cell::new(State::Idle),
        }
    }

    pub fn set_meta_client(&self, client: &'a dyn At24Mac402Client) {
        self.meta_client.set(client);
    }

    // -----------------------------------------------------------------------
    // Metadata operations (extended block at 0x5F)
    // -----------------------------------------------------------------------

    /// Read the factory-programmed EUI-48 MAC address (6 bytes).
    pub fn read_mac_address(&self) -> Result<(), ErrorCode> {
        if self.state.get() != State::Idle {
            return Err(ErrorCode::BUSY);
        }
        self.buffer.take().map_or(Err(ErrorCode::RESERVE), |buf| {
            buf[0] = EXT_EUI48_ADDR;
            self.i2c_ext.enable();
            self.state.set(State::ReadingMac);
            if let Err((error, buffer)) = self.i2c_ext.write_read(buf, 1, EUI48_LEN) {
                self.buffer.replace(buffer);
                self.i2c_ext.disable();
                self.state.set(State::Idle);
                Err(error.into())
            } else {
                Ok(())
            }
        })
    }

    /// Read the factory-programmed 128-bit serial number (16 bytes).
    pub fn read_serial_number(&self) -> Result<(), ErrorCode> {
        if self.state.get() != State::Idle {
            return Err(ErrorCode::BUSY);
        }
        self.buffer.take().map_or(Err(ErrorCode::RESERVE), |buf| {
            buf[0] = EXT_SERIAL_ADDR;
            self.i2c_ext.enable();
            self.state.set(State::ReadingSerial);
            if let Err((error, buffer)) = self.i2c_ext.write_read(buf, 1, SERIAL_LEN) {
                self.buffer.replace(buffer);
                self.i2c_ext.disable();
                self.state.set(State::Idle);
                Err(error.into())
            } else {
                Ok(())
            }
        })
    }

    // -----------------------------------------------------------------------
    // EEPROM page operations (user memory at 0x57)
    // -----------------------------------------------------------------------

    fn read_sector(
        &self,
        page_number: usize,
        buf: &'static mut EEPROMPage,
    ) -> Result<(), (ErrorCode, &'static mut EEPROMPage)> {
        let address = page_number * PAGE_SIZE;
        if address >= EEPROM_SIZE {
            return Err((ErrorCode::INVAL, buf));
        }
        if let Some(rxbuffer) = self.buffer.take() {
            rxbuffer[0] = address as u8;

            self.i2c.enable();
            self.state.set(State::ReadingPage);
            if let Err((error, local_buffer)) = self.i2c.write_read(rxbuffer, 1, PAGE_SIZE) {
                self.buffer.replace(local_buffer);
                self.i2c.disable();
                self.state.set(State::Idle);
                Err((error.into(), buf))
            } else {
                self.client_page.replace(buf);
                Ok(())
            }
        } else {
            Err((ErrorCode::RESERVE, buf))
        }
    }

    fn write_sector(
        &self,
        page_number: usize,
        buf: &'static mut EEPROMPage,
    ) -> Result<(), (ErrorCode, &'static mut EEPROMPage)> {
        let address = page_number * PAGE_SIZE;
        if address >= EEPROM_SIZE {
            return Err((ErrorCode::INVAL, buf));
        }
        if let Some(txbuffer) = self.buffer.take() {
            txbuffer[0] = address as u8;

            let write_len = cmp::min(txbuffer.len() - 1, buf.0.len());
            txbuffer[1..(write_len + 1)].copy_from_slice(&buf.0[..write_len]);

            self.i2c.enable();
            self.state.set(State::WritingPage);
            if let Err((error, txbuffer)) = self.i2c.write(txbuffer, write_len + 1) {
                self.buffer.replace(txbuffer);
                self.i2c.disable();
                self.state.set(State::Idle);
                Err((error.into(), buf))
            } else {
                self.client_page.replace(buf);
                Ok(())
            }
        } else {
            Err((ErrorCode::RESERVE, buf))
        }
    }

    fn erase_sector(&self, page_number: usize) -> Result<(), ErrorCode> {
        let address = page_number * PAGE_SIZE;
        if address >= EEPROM_SIZE {
            return Err(ErrorCode::INVAL);
        }
        if let Some(txbuffer) = self.buffer.take() {
            txbuffer[0] = address as u8;

            let write_len = cmp::min(txbuffer.len() - 1, PAGE_SIZE);
            for i in 0..write_len {
                txbuffer[i + 1] = 0xFF;
            }

            self.i2c.enable();
            self.state.set(State::ErasingPage);
            if let Err((error, txbuffer)) = self.i2c.write(txbuffer, write_len + 1) {
                self.buffer.replace(txbuffer);
                self.i2c.disable();
                self.state.set(State::Idle);
                Err(error.into())
            } else {
                Ok(())
            }
        } else {
            Err(ErrorCode::RESERVE)
        }
    }
}

// ---------------------------------------------------------------------------
// I2CClient — handles completion callbacks from both I2C devices
// ---------------------------------------------------------------------------

impl I2CClient for At24Mac402<'static> {
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), Error>) {
        match self.state.get() {
            State::ReadingMac => {
                self.state.set(State::Idle);
                self.i2c_ext.disable();
                let mut mac = [0u8; EUI48_LEN];
                mac.copy_from_slice(&buffer[..EUI48_LEN]);
                self.buffer.replace(buffer);
                self.meta_client.map(|c| {
                    let result = if status.is_err() {
                        Err(ErrorCode::FAIL)
                    } else {
                        Ok(())
                    };
                    c.mac_read_complete(&mac, result);
                });
            }
            State::ReadingSerial => {
                self.state.set(State::Idle);
                self.i2c_ext.disable();
                let mut serial = [0u8; SERIAL_LEN];
                serial.copy_from_slice(&buffer[..SERIAL_LEN]);
                self.buffer.replace(buffer);
                self.meta_client.map(|c| {
                    let result = if status.is_err() {
                        Err(ErrorCode::FAIL)
                    } else {
                        Ok(())
                    };
                    c.serial_read_complete(&serial, result);
                });
            }
            State::ReadingPage => {
                self.state.set(State::Idle);
                self.i2c.disable();
                if let Some(client_page) = self.client_page.take() {
                    client_page.0[..PAGE_SIZE].copy_from_slice(&buffer[..PAGE_SIZE]);
                    self.buffer.replace(buffer);
                    self.flash_client.map(|client| {
                        if status.is_err() {
                            client.read_complete(client_page, Err(hil::flash::Error::FlashError));
                        } else {
                            client.read_complete(client_page, Ok(()));
                        }
                    });
                }
            }
            State::WritingPage => {
                self.state.set(State::Idle);
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.flash_client.map(|client| {
                    if let Some(client_page) = self.client_page.take() {
                        if status.is_err() {
                            client.write_complete(client_page, Err(hil::flash::Error::FlashError));
                        } else {
                            client.write_complete(client_page, Ok(()));
                        }
                    }
                });
            }
            State::ErasingPage => {
                self.state.set(State::Idle);
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.flash_client.map(move |client| {
                    if status.is_err() {
                        client.erase_complete(Err(hil::flash::Error::FlashError));
                    } else {
                        client.erase_complete(Ok(()));
                    }
                });
            }
            State::Idle => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Flash HIL implementation (page-level access to user EEPROM)
// ---------------------------------------------------------------------------

impl hil::flash::Flash for At24Mac402<'_> {
    type Page = EEPROMPage;

    fn read_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)> {
        self.read_sector(page_number, buf)
    }

    fn write_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)> {
        self.write_sector(page_number, buf)
    }

    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> {
        self.erase_sector(page_number)
    }
}

impl<'a, C: hil::flash::Client<Self>> hil::flash::HasClient<'a, C> for At24Mac402<'a> {
    fn set_client(&'a self, client: &'a C) {
        self.flash_client.set(client);
    }
}
