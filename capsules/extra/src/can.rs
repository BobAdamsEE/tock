// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022
// Copyright OxidOS Automotive SRL 2022
//
// Author: Teona Severin <teona.severin@oxidos.io>

//! Syscall driver capsule for CAN communication.
//!
//! Several processes may use the CAN bus at once. Each one gets its own
//! buffers, upcalls and identifier subscriptions; the capsule serializes
//! transmission between them and fans received frames out to every process
//! that asked for the identifier.
//!
//! Reception is opt-in. A process registers the identifiers it wants with
//! `subscribe_standard` / `subscribe_extended`, and receives nothing until it
//! does. This replaces the previous behaviour, where a single owning process
//! received every frame the hardware accepted and discarded the rest itself.
//!
//! The receive buffer is a `StreamingProcessSlice`: an 8-byte header followed
//! by fixed 16-byte chunks, one per frame.
//!
//! ```text
//! offset  size  field
//!      0     4  identifier, little endian
//!      4     1  data length, 0..=8
//!      5     1  flags: bit 0 set for a 29-bit (extended) identifier
//!      6     2  reserved, zero
//!      8     8  data
//! ```
//!
//! The identifier travels *in the chunk* rather than only as an upcall
//! argument, because several frames can be appended before a process is
//! scheduled; with the identifier passed out of band, only the last one would
//! be knowable and a burst could not be demultiplexed.
//!
//! Bitrate, bit timing and operation mode are properties of the shared bus, so
//! they are configured once during board setup and are no longer reachable
//! from userspace.
//!
//! Usage
//! -----
//!
//! ```rust,ignore
//! let can = capsules_extra::can::CanCapsule::new(
//!     can_device,          // anything implementing hil::can::Can
//!     Some(can_device),    // optional &dyn Subscribe, to track subscriptions
//!     grant_can,
//!     tx_buffer,
//!     rx_buffer,
//! );
//!
//! kernel::hil::can::Controller::set_client(can_device, Some(can));
//! kernel::hil::can::Transmit::set_client(can_device, Some(can));
//! kernel::hil::can::Receive::set_client(can_device, Some(can));
//! ```

use core::mem::size_of;

use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
use kernel::hil::can;
use kernel::processbuffer::{ReadableProcessBuffer, WriteableProcessBuffer};
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::streaming_process_slice::StreamingProcessSlice;
use kernel::ErrorCode;
use kernel::ProcessId;

use capsules_core::driver;
use capsules_core::virtualizers::virtual_can::{covering_filter, Subscribe, Subscription};

pub const DRIVER_NUM: usize = driver::NUM::Can as usize;

/// Identifier subscriptions each process may hold.
pub const MAX_APP_SUBSCRIPTIONS: usize = 4;

/// Upper bound on subscriptions collected across all processes when
/// recomputing the device's view. Extra ones are covered by the fallback.
const MAX_TOTAL_SUBSCRIPTIONS: usize = 16;

/// Bytes per frame in the receive buffer. See the module documentation.
pub const RX_CHUNK_SIZE: usize = 16;

const CHUNK_FLAG_EXTENDED: u8 = 0x01;

mod error_upcalls {
    pub const ERROR_TX: usize = 100;
    pub const ERROR_RX: usize = 101;
}

mod up_calls {
    pub const UPCALL_ENABLE: usize = 0;
    pub const UPCALL_DISABLE: usize = 1;
    pub const UPCALL_MESSAGE_SENT: usize = 2;
    pub const UPCALL_MESSAGE_RECEIVED: usize = 3;
    pub const UPCALL_RECEIVED_STOPPED: usize = 4;
    pub const UPCALL_TRANSMISSION_ERROR: usize = 5;
    pub const COUNT: u8 = 6;
}

mod ro_allow {
    pub const RO_ALLOW_BUFFER: usize = 0;
    pub const COUNT: u8 = 1;
}

mod rw_allow {
    pub const RW_ALLOW_BUFFER: usize = 0;
    pub const COUNT: u8 = 1;
}

pub struct CanCapsule<'a, Can: can::Can> {
    can: &'a Can,

    /// Optional handle for keeping the underlying device's subscriptions in
    /// step with the union of the processes'. `None` when the capsule sits on
    /// a bare peripheral, in which case acceptance is whatever the board
    /// configured and only the per-process software filter applies.
    subscriber: Option<&'a dyn Subscribe>,

    can_tx: TakeCell<'static, [u8; can::STANDARD_CAN_PACKET_SIZE]>,
    can_rx: TakeCell<'static, [u8; can::STANDARD_CAN_PACKET_SIZE]>,

    processes: Grant<
        App,
        UpcallCount<{ up_calls::COUNT }>,
        AllowRoCount<{ ro_allow::COUNT }>,
        AllowRwCount<{ rw_allow::COUNT }>,
    >,

    /// Process whose frame is currently with the hardware.
    tx_inflight: OptionalCell<ProcessId>,

    /// How many processes want the peripheral enabled / receiving.
    enable_count: core::cell::Cell<usize>,
    receive_count: core::cell::Cell<usize>,
    hw_enabled: core::cell::Cell<bool>,
    hw_receiving: core::cell::Cell<bool>,

    peripheral_state: OptionalCell<can::State>,
}

pub struct App {
    subscriptions: [Option<Subscription>; MAX_APP_SUBSCRIPTIONS],
    /// This process asked for the peripheral to be enabled.
    enabled: bool,
    /// Waiting on the enable/disable callback the hardware will deliver.
    awaiting_enable: bool,
    awaiting_disable: bool,
    /// This process asked to receive.
    receiving: bool,
    /// Queued transmission, waiting for the shared transmit slot.
    ///
    /// The frame's bytes are copied in when it is queued rather than read back
    /// out of the process buffer when the slot frees up. A process does not
    /// stop running while its frame waits -- with several processes sharing
    /// one transmit slot a frame can wait for milliseconds -- so by the time
    /// it reached the hardware the buffer could hold the *next* frame, and the
    /// wrong bytes would go out under the right identifier.
    tx_pending: Option<(can::Id, usize, [u8; can::STANDARD_CAN_PACKET_SIZE])>,
    lost_messages: u32,
}

impl Default for App {
    fn default() -> Self {
        App {
            subscriptions: [None; MAX_APP_SUBSCRIPTIONS],
            enabled: false,
            awaiting_enable: false,
            awaiting_disable: false,
            receiving: false,
            tx_pending: None,
            lost_messages: 0,
        }
    }
}

impl<'a, Can: can::Can> CanCapsule<'a, Can> {
    pub fn new(
        can: &'a Can,
        subscriber: Option<&'a dyn Subscribe>,
        grant: Grant<
            App,
            UpcallCount<{ up_calls::COUNT }>,
            AllowRoCount<{ ro_allow::COUNT }>,
            AllowRwCount<{ rw_allow::COUNT }>,
        >,
        can_tx: &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE],
        can_rx: &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE],
    ) -> CanCapsule<'a, Can> {
        CanCapsule {
            can,
            subscriber,
            can_tx: TakeCell::new(can_tx),
            can_rx: TakeCell::new(can_rx),
            processes: grant,
            tx_inflight: OptionalCell::empty(),
            enable_count: core::cell::Cell::new(0),
            receive_count: core::cell::Cell::new(0),
            hw_enabled: core::cell::Cell::new(false),
            hw_receiving: core::cell::Cell::new(false),
            peripheral_state: OptionalCell::empty(),
        }
    }

    fn upcall(&self, processid: ProcessId, number: usize, data: (usize, usize, usize)) {
        let _ = self.processes.enter(processid, |_, kernel_data| {
            let _ = kernel_data.schedule_upcall(number, data);
        });
    }

    /// Rebuild the device's subscriptions from the union of every process's.
    ///
    /// Must not be called while inside `processes.enter()` for any process, or
    /// that process's subscriptions would be skipped.
    fn resync_subscriptions(&self) {
        let Some(subscriber) = self.subscriber else {
            return;
        };
        subscriber.clear_subscriptions();

        let mut all = [Subscription {
            id: can::Id::Standard(0),
            mask: 0,
        }; MAX_TOTAL_SUBSCRIPTIONS];
        let mut count = 0;

        for process in self.processes.iter() {
            process.enter(|app, _| {
                for slot in app.subscriptions.iter() {
                    if let Some(s) = slot {
                        if count < all.len() {
                            all[count] = *s;
                            count += 1;
                        }
                    }
                }
            });
        }

        if count == 0 {
            return;
        }

        if count <= subscriber.subscription_capacity() {
            for s in all.iter().take(count) {
                let _ = subscriber.subscribe(s.id, s.mask);
            }
            return;
        }

        // Too many to install individually: one covering subscription per
        // identifier class accepts a superset, and the per-process filter in
        // `message_received` rejects the extras.
        for extended in [false, true] {
            let mut class = [Subscription {
                id: can::Id::Standard(0),
                mask: 0,
            }; MAX_TOTAL_SUBSCRIPTIONS];
            let mut n = 0;
            for s in all.iter().take(count) {
                if matches!(s.id, can::Id::Extended(_)) == extended {
                    class[n] = *s;
                    n += 1;
                }
            }
            if n > 0 {
                if let Some(cover) = covering_filter(&class[..n]) {
                    let _ = subscriber.subscribe(cover.id, cover.mask);
                }
            }
        }
    }

    /// Read this process's transmit buffer into `frame`.
    fn copy_out_frame(
        &self,
        processid: ProcessId,
        length: usize,
        frame: &mut [u8; can::STANDARD_CAN_PACKET_SIZE],
    ) -> Result<(), ErrorCode> {
        if length > can::STANDARD_CAN_PACKET_SIZE {
            return Err(ErrorCode::SIZE);
        }
        self.processes
            .enter(processid, |_, kernel_data| {
                kernel_data
                    .get_readonly_processbuffer(ro_allow::RO_ALLOW_BUFFER)
                    .map_or_else(
                        |err| err.into(),
                        |buffer_ref| {
                            buffer_ref
                                .enter(|buffer| {
                                    if buffer.len() < length {
                                        return Err(ErrorCode::SIZE);
                                    }
                                    for i in 0..length {
                                        frame[i] = buffer[i].get();
                                    }
                                    Ok(())
                                })
                                .unwrap_or_else(|err| err.into())
                        },
                    )
            })
            .unwrap_or_else(|err| err.into())
    }

    /// Put an already-copied frame into the shared buffer and start it.
    fn start_transmission(
        &self,
        id: can::Id,
        frame: &[u8; can::STANDARD_CAN_PACKET_SIZE],
        length: usize,
    ) -> Result<(), ErrorCode> {
        if length > can::STANDARD_CAN_PACKET_SIZE {
            return Err(ErrorCode::SIZE);
        }
        self.can_tx.take().map_or(Err(ErrorCode::NOMEM), |dest| {
            dest[..length].copy_from_slice(&frame[..length]);
            match self.can.send(id, dest, length) {
                Ok(()) => Ok(()),
                Err((err, buf)) => {
                    self.can_tx.replace(buf);
                    Err(err)
                }
            }
        })
    }

    /// Hand the transmitter to the next process with a queued frame.
    fn next_transmission(&self) {
        if self.tx_inflight.is_some() {
            return;
        }
        for process in self.processes.iter() {
            let processid = process.processid();
            let queued = process.enter(|app, _| app.tx_pending.take());
            if let Some((id, length, frame)) = queued {
                match self.start_transmission(id, &frame, length) {
                    Ok(()) => {
                        self.tx_inflight.set(processid);
                        return;
                    }
                    Err(err) => {
                        self.upcall(
                            processid,
                            up_calls::UPCALL_TRANSMISSION_ERROR,
                            (error_upcalls::ERROR_TX, err as usize, 0),
                        );
                    }
                }
            }
        }
    }
}

impl<Can: can::Can> SyscallDriver for CanCapsule<'_, Can> {
    fn command(
        &self,
        command_num: usize,
        arg1: usize,
        arg2: usize,
        processid: ProcessId,
    ) -> CommandReturn {
        // This driver exists.
        if command_num == 0 {
            return CommandReturn::success();
        }

        match command_num {
            // Bitrate (1), operation mode (2) and bit timing (9) describe the
            // shared bus, not one process's use of it. They are set during
            // board setup; letting any process change them out from under the
            // others was a bug waiting to happen.
            1 | 2 | 9 => CommandReturn::failure(ErrorCode::NOSUPPORT),

            // Enable the peripheral.
            3 => {
                let already = self.hw_enabled.get();
                let res = self.processes.enter(processid, |app, _| {
                    if app.enabled {
                        return Err(ErrorCode::ALREADY);
                    }
                    app.enabled = true;
                    app.awaiting_enable = !already;
                    Ok(())
                });
                match res {
                    Ok(Ok(())) => {
                        self.enable_count.set(self.enable_count.get() + 1);
                        if already {
                            // Running for someone else already.
                            self.upcall(processid, up_calls::UPCALL_ENABLE, (0, 0, 0));
                            CommandReturn::success()
                        } else if self.enable_count.get() == 1 {
                            match self.can.enable() {
                                Ok(()) => CommandReturn::success(),
                                Err(err) => {
                                    self.enable_count
                                        .set(self.enable_count.get().saturating_sub(1));
                                    let _ = self.processes.enter(processid, |app, _| {
                                        app.enabled = false;
                                        app.awaiting_enable = false;
                                    });
                                    CommandReturn::failure(err)
                                }
                            }
                        } else {
                            // An enable is already in flight; its completion
                            // notifies every waiting process.
                            CommandReturn::success()
                        }
                    }
                    Ok(Err(err)) => CommandReturn::failure(err),
                    Err(err) => CommandReturn::failure(err.into()),
                }
            }

            // Disable the peripheral.
            4 => {
                let res = self.processes.enter(processid, |app, _| {
                    if !app.enabled {
                        return Err(ErrorCode::ALREADY);
                    }
                    app.enabled = false;
                    Ok(())
                });
                match res {
                    Ok(Ok(())) => {
                        self.enable_count
                            .set(self.enable_count.get().saturating_sub(1));
                        if self.enable_count.get() > 0 {
                            // Others still need the bus.
                            self.upcall(processid, up_calls::UPCALL_DISABLE, (0, 0, 0));
                            CommandReturn::success()
                        } else {
                            let _ = self.processes.enter(processid, |app, _| {
                                app.awaiting_disable = true;
                            });
                            match self.can.disable() {
                                Ok(()) => CommandReturn::success(),
                                Err(err) => {
                                    let _ = self.processes.enter(processid, |app, _| {
                                        app.awaiting_disable = false;
                                    });
                                    CommandReturn::failure(err)
                                }
                            }
                        }
                    }
                    Ok(Err(err)) => CommandReturn::failure(err),
                    Err(err) => CommandReturn::failure(err.into()),
                }
            }

            // Send a frame with an 11-bit (5) or 29-bit (6) identifier.
            5 | 6 => {
                let id = if command_num == 5 {
                    can::Id::Standard(arg1 as u16)
                } else {
                    can::Id::Extended(arg1 as u32)
                };

                // Read the frame out of the process buffer now, whether it goes
                // straight to the hardware or waits for the shared slot. See
                // `App::tx_pending`.
                let mut frame = [0u8; can::STANDARD_CAN_PACKET_SIZE];
                if let Err(err) = self.copy_out_frame(processid, arg2, &mut frame) {
                    return CommandReturn::failure(err);
                }

                if self.tx_inflight.is_none() {
                    match self.start_transmission(id, &frame, arg2) {
                        Ok(()) => {
                            self.tx_inflight.set(processid);
                            CommandReturn::success()
                        }
                        Err(err) => CommandReturn::failure(err),
                    }
                } else {
                    // Queue behind the frame currently on the bus.
                    let res = self.processes.enter(processid, |app, _| {
                        if app.tx_pending.is_some() {
                            return Err(ErrorCode::BUSY);
                        }
                        app.tx_pending = Some((id, arg2, frame));
                        Ok(())
                    });
                    match res {
                        Ok(Ok(())) => CommandReturn::success(),
                        Ok(Err(err)) => CommandReturn::failure(err),
                        Err(err) => CommandReturn::failure(err.into()),
                    }
                }
            }

            // Start receiving.
            7 => {
                let res = self.processes.enter(processid, |app, kernel| {
                    if app.receiving {
                        return Err(ErrorCode::ALREADY);
                    }
                    kernel
                        .get_readwrite_processbuffer(rw_allow::RW_ALLOW_BUFFER)
                        .map_or_else(
                            |err| Err(err.into()),
                            |buffer_ref| {
                                buffer_ref
                                    .enter(|buffer| {
                                        // Room for the header plus at least
                                        // two frames.
                                        if buffer.len() >= 2 * RX_CHUNK_SIZE + 2 * size_of::<u32>()
                                        {
                                            Ok(())
                                        } else {
                                            Err(ErrorCode::SIZE)
                                        }
                                    })
                                    .unwrap_or_else(|err| Err(err.into()))
                            },
                        )?;
                    app.receiving = true;
                    Ok(())
                });

                match res {
                    Ok(Ok(())) => {
                        self.receive_count.set(self.receive_count.get() + 1);
                        if self.hw_receiving.get() {
                            return CommandReturn::success();
                        }
                        match self.can_rx.take() {
                            Some(buffer) => match self.can.start_receive_process(buffer) {
                                Ok(()) => {
                                    self.hw_receiving.set(true);
                                    CommandReturn::success()
                                }
                                Err((err, buffer)) => {
                                    self.can_rx.replace(buffer);
                                    self.receive_count
                                        .set(self.receive_count.get().saturating_sub(1));
                                    let _ = self.processes.enter(processid, |app, _| {
                                        app.receiving = false;
                                    });
                                    CommandReturn::failure(err)
                                }
                            },
                            None => {
                                self.receive_count
                                    .set(self.receive_count.get().saturating_sub(1));
                                CommandReturn::failure(ErrorCode::NOMEM)
                            }
                        }
                    }
                    Ok(Err(err)) => CommandReturn::failure(err),
                    Err(err) => CommandReturn::failure(err.into()),
                }
            }

            // Stop receiving.
            8 => {
                let res = self.processes.enter(processid, |app, _| {
                    if !app.receiving {
                        return Err(ErrorCode::ALREADY);
                    }
                    app.receiving = false;
                    Ok(())
                });
                match res {
                    Ok(Ok(())) => {
                        self.receive_count
                            .set(self.receive_count.get().saturating_sub(1));
                        if self.receive_count.get() > 0 || !self.hw_receiving.get() {
                            self.upcall(processid, up_calls::UPCALL_RECEIVED_STOPPED, (0, 0, 0));
                            CommandReturn::success()
                        } else {
                            match self.can.stop_receive() {
                                Ok(()) => CommandReturn::success(),
                                Err(err) => CommandReturn::failure(err),
                            }
                        }
                    }
                    Ok(Err(err)) => CommandReturn::failure(err),
                    Err(err) => CommandReturn::failure(err.into()),
                }
            }

            // Subscribe to an 11-bit (10) or 29-bit (11) identifier range.
            // arg1 = identifier, arg2 = mask. A frame matches when
            // `(received & mask) == (id & mask)`.
            10 | 11 => {
                let id = if command_num == 10 {
                    can::Id::Standard(arg1 as u16)
                } else {
                    can::Id::Extended(arg1 as u32)
                };
                let subscription = Subscription {
                    id,
                    mask: arg2 as u32,
                };
                let res = self.processes.enter(processid, |app, _| {
                    for slot in app.subscriptions.iter_mut() {
                        if slot.is_none() {
                            *slot = Some(subscription);
                            return Ok(());
                        }
                    }
                    Err(ErrorCode::NOMEM)
                });
                match res {
                    // Outside the grant: resync re-enters every process.
                    Ok(Ok(())) => {
                        self.resync_subscriptions();
                        CommandReturn::success()
                    }
                    Ok(Err(err)) => CommandReturn::failure(err),
                    Err(err) => CommandReturn::failure(err.into()),
                }
            }

            // Drop all of this process's subscriptions.
            12 => {
                let res = self.processes.enter(processid, |app, _| {
                    app.subscriptions = [None; MAX_APP_SUBSCRIPTIONS];
                });
                match res {
                    Ok(()) => {
                        self.resync_subscriptions();
                        CommandReturn::success()
                    }
                    Err(err) => CommandReturn::failure(err.into()),
                }
            }

            // How many subscriptions a process may hold.
            13 => CommandReturn::success_u32(MAX_APP_SUBSCRIPTIONS as u32),

            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, process_id: ProcessId) -> Result<(), kernel::process::Error> {
        self.processes.enter(process_id, |_, _| {})
    }
}

impl<Can: can::Can> can::ControllerClient for CanCapsule<'_, Can> {
    fn state_changed(&self, state: can::State) {
        self.peripheral_state.replace(state);
    }

    fn enabled(&self, status: Result<(), ErrorCode>) {
        let code = match status {
            Ok(()) => match self.peripheral_state.take() {
                Some(can::State::Running) => {
                    self.hw_enabled.set(true);
                    0
                }
                Some(can::State::Error(err)) => err as usize,
                Some(can::State::Disabled) | None => ErrorCode::OFF as usize,
            },
            Err(err) => {
                self.peripheral_state.take();
                err as usize
            }
        };

        if code != 0 {
            self.enable_count.set(0);
        }

        for process in self.processes.iter() {
            let processid = process.processid();
            let notify = process.enter(|app, _| {
                if app.awaiting_enable {
                    app.awaiting_enable = false;
                    if code != 0 {
                        app.enabled = false;
                    }
                    true
                } else {
                    false
                }
            });
            if notify {
                self.upcall(processid, up_calls::UPCALL_ENABLE, (code, 0, 0));
            }
        }
    }

    fn disabled(&self, status: Result<(), ErrorCode>) {
        self.hw_enabled.set(false);
        self.hw_receiving.set(false);

        let code = match status {
            Ok(()) => match self.peripheral_state.take() {
                Some(can::State::Disabled) => 0,
                Some(can::State::Error(err)) => err as usize,
                Some(can::State::Running) | None => ErrorCode::FAIL as usize,
            },
            Err(err) => {
                self.peripheral_state.take();
                err as usize
            }
        };

        for process in self.processes.iter() {
            let processid = process.processid();
            let notify = process.enter(|app, _| {
                if app.awaiting_disable {
                    app.awaiting_disable = false;
                    true
                } else {
                    false
                }
            });
            if notify {
                self.upcall(processid, up_calls::UPCALL_DISABLE, (code, 0, 0));
            }
        }
    }
}

impl<Can: can::Can> can::TransmitClient<{ can::STANDARD_CAN_PACKET_SIZE }> for CanCapsule<'_, Can> {
    fn transmit_complete(
        &self,
        status: Result<(), can::Error>,
        buffer: &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE],
    ) {
        self.can_tx.replace(buffer);

        if let Some(processid) = self.tx_inflight.take() {
            match status {
                Ok(()) => self.upcall(processid, up_calls::UPCALL_MESSAGE_SENT, (0, 0, 0)),
                Err(err) => self.upcall(
                    processid,
                    up_calls::UPCALL_TRANSMISSION_ERROR,
                    (error_upcalls::ERROR_TX, err as usize, 0),
                ),
            }
        }

        self.next_transmission();
    }
}

impl<Can: can::Can> can::ReceiveClient<{ can::STANDARD_CAN_PACKET_SIZE }> for CanCapsule<'_, Can> {
    fn message_received(
        &self,
        id: can::Id,
        buffer: &mut [u8; can::STANDARD_CAN_PACKET_SIZE],
        len: usize,
        status: Result<(), can::Error>,
    ) {
        if let Err(err) = status {
            for process in self.processes.iter() {
                let processid = process.processid();
                let receiving = process.enter(|app, _| app.receiving);
                if receiving {
                    self.upcall(
                        processid,
                        up_calls::UPCALL_TRANSMISSION_ERROR,
                        (error_upcalls::ERROR_RX, err as usize, 0),
                    );
                }
            }
            return;
        }

        // Fixed-size chunk so a process can walk the buffer without parsing
        // variable-length records. See the module documentation.
        let raw_id = match id {
            can::Id::Standard(v) => v as u32,
            can::Id::Extended(v) => v,
        };
        let mut chunk = [0u8; RX_CHUNK_SIZE];
        chunk[0..4].copy_from_slice(&raw_id.to_le_bytes());
        chunk[4] = len.min(can::STANDARD_CAN_PACKET_SIZE) as u8;
        chunk[5] = if matches!(id, can::Id::Extended(_)) {
            CHUNK_FLAG_EXTENDED
        } else {
            0
        };
        chunk[8..8 + can::STANDARD_CAN_PACKET_SIZE].copy_from_slice(&buffer[..]);

        for process in self.processes.iter() {
            let processid = process.processid();

            let result = process.enter(|app, kernel_data| {
                if !app.receiving {
                    return None;
                }
                // Per-process filter: the device may accept a superset of what
                // this process asked for.
                if !app
                    .subscriptions
                    .iter()
                    .flatten()
                    .any(|s| subscription_matches(s, id))
                {
                    return None;
                }
                Some(
                    kernel_data
                        .get_readwrite_processbuffer(rw_allow::RW_ALLOW_BUFFER)
                        .map_or_else(
                            |err| Err(err.into()),
                            |buffer_ref| {
                                buffer_ref
                                    .mut_enter(|user_slice| {
                                        StreamingProcessSlice::new(user_slice)
                                            .append_chunk(&chunk)
                                            .inspect_err(|_| {
                                                app.lost_messages += 1;
                                            })
                                    })
                                    .unwrap_or_else(|err| Err(err.into()))
                            },
                        ),
                )
            });

            match result {
                Some(Ok((_first, new_offset))) => self.upcall(
                    processid,
                    up_calls::UPCALL_MESSAGE_RECEIVED,
                    (0, new_offset as usize, raw_id as usize),
                ),
                Some(Err(err)) => self.upcall(
                    processid,
                    up_calls::UPCALL_TRANSMISSION_ERROR,
                    (error_upcalls::ERROR_RX, err as usize, 0),
                ),
                None => {}
            }
        }
    }

    fn stopped(&self, buffer: &'static mut [u8; can::STANDARD_CAN_PACKET_SIZE]) {
        self.can_rx.replace(buffer);
        self.hw_receiving.set(false);

        for process in self.processes.iter() {
            let processid = process.processid();
            let notify = process.enter(|app, _| !app.receiving);
            if notify {
                self.upcall(processid, up_calls::UPCALL_RECEIVED_STOPPED, (0, 0, 0));
            }
        }
    }
}

/// Does `subscription` accept `received`?
fn subscription_matches(subscription: &Subscription, received: can::Id) -> bool {
    let (sub_extended, sub_raw) = match subscription.id {
        can::Id::Standard(v) => (false, v as u32),
        can::Id::Extended(v) => (true, v),
    };
    let (rx_extended, rx_raw) = match received {
        can::Id::Standard(v) => (false, v as u32),
        can::Id::Extended(v) => (true, v),
    };
    sub_extended == rx_extended && (rx_raw & subscription.mask) == (sub_raw & subscription.mask)
}
