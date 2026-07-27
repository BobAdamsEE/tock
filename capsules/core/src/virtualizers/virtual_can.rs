// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Virtualize a CAN peripheral.
//!
//! `MuxCan` provides shared access to one CAN controller from several clients.
//! Each client owns a [`CanDevice`], which implements [`hil::can::Can`] and can
//! therefore be dropped in anywhere the bare peripheral was used before.
//!
//! Transmit and receive virtualize very differently:
//!
//! * **Transmit** is a shared resource, so it is serialized. Devices queue a
//!   frame and `MuxCan` issues them one at a time, rotating the starting point
//!   after each completion so a chatty client cannot starve the others. Note
//!   that *bus* priority is still CAN arbitration's job; this ordering only
//!   matters once frames back up locally.
//!
//! * **Receive** is a broadcast, so it fans out. `MuxCan` owns the single
//!   `&'static mut` buffer the peripheral requires and hands each matching
//!   device a *borrow* of the received frame — [`hil::can::ReceiveClient::
//!   message_received`] takes `&mut [u8; N]` rather than `&'static mut`, so no
//!   copying or buffer juggling is needed inside the mux.
//!
//! Reception is opt-in: a device receives only frames matching one of its
//! [`Subscription`]s, and a device with no subscriptions receives nothing.
//!
//! Hardware filters are an optimisation, not the correctness mechanism. The
//! software post-filter in [`CanDevice::matches`] always runs. `MuxCan`
//! additionally programs the peripheral's filter slots to cover the union of
//! all subscriptions, purely to keep unwanted traffic from generating
//! interrupts. When the union does not fit in the available slots it collapses
//! to a single covering filter per identifier class (see
//! `covering_filter`), which accepts a superset. Correctness therefore never
//! depends on how many filter slots the hardware has.
//!
//! Usage
//! -----
//!
//! ```rust,ignore
//! let mux_can = static_init!(
//!     capsules_core::virtualizers::virtual_can::MuxCan<'static, Mcan>,
//!     capsules_core::virtualizers::virtual_can::MuxCan::new(&peripherals.mcan1, rx_buf));
//! kernel::hil::can::Controller::set_client(&peripherals.mcan1, Some(mux_can));
//! kernel::hil::can::Transmit::set_client(&peripherals.mcan1, Some(mux_can));
//! kernel::hil::can::Receive::set_client(&peripherals.mcan1, Some(mux_can));
//!
//! let can_device = static_init!(
//!     capsules_core::virtualizers::virtual_can::CanDevice<'static, Mcan>,
//!     capsules_core::virtualizers::virtual_can::CanDevice::new(mux_can));
//! can_device.setup();
//! ```

use core::cell::Cell;

use kernel::collections::list::{List, ListLink, ListNode};
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::can;
use kernel::hil::can::{Configure, Controller, Receive, Transmit};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::ErrorCode;

/// Number of identifier subscriptions each [`CanDevice`] can hold.
pub const MAX_SUBSCRIPTIONS: usize = 4;

/// Size of the frames this virtualizer carries. Classic CAN only; CAN FD would
/// need a second set of implementations against `hil::can::CanFd`.
const PACKET_SIZE: usize = can::STANDARD_CAN_PACKET_SIZE;

/// A logical request to receive a range of identifiers.
///
/// A frame matches when `(received & mask) == (id & mask)`, and only when its
/// identifier is of the same class (standard vs extended) as `id`. An all-ones
/// mask therefore selects a single identifier and a zero mask accepts every
/// identifier of that class.
#[derive(Copy, Clone)]
pub struct Subscription {
    pub id: can::Id,
    pub mask: u32,
}

impl Subscription {
    fn matches(&self, received: can::Id) -> bool {
        if is_extended(self.id) != is_extended(received) {
            return false;
        }
        (raw_id(received) & self.mask) == (raw_id(self.id) & self.mask)
    }
}

fn raw_id(id: can::Id) -> u32 {
    match id {
        can::Id::Standard(v) => v as u32,
        can::Id::Extended(v) => v,
    }
}

fn is_extended(id: can::Id) -> bool {
    matches!(id, can::Id::Extended(_))
}

/// Lets a client register which identifiers it wants to receive.
///
/// This is deliberately not part of `hil::can`: it describes the virtualizer's
/// dispatch policy, not a property of CAN hardware. Capsules that want to keep
/// a device's subscriptions in sync with their own clients take this as a
/// `&dyn Subscribe` so they remain usable with an unvirtualized peripheral.
pub trait Subscribe {
    /// Register one identifier range. Fails with `NOMEM` when full.
    fn subscribe(&self, id: can::Id, mask: u32) -> Result<(), ErrorCode>;

    /// Drop every subscription, after which nothing is received.
    fn clear_subscriptions(&self);

    /// How many subscriptions can be held at once.
    fn subscription_capacity(&self) -> usize;
}

/// Collapse several subscriptions of one identifier class into a single filter
/// that accepts a superset of their union.
///
/// The result keeps only the mask bits that are (a) present in *every*
/// subscription's mask and (b) identical across every subscription's
/// identifier. Any frame accepted by some subscription agrees with that
/// subscription's id on those bits, and that id agrees with the first id on
/// those bits, so the frame is accepted here too. Bits outside the result are
/// simply left free, which can only widen acceptance -- and the software
/// post-filter rejects the extras.
pub fn covering_filter(subs: &[Subscription]) -> Option<Subscription> {
    let first = *subs.first()?;
    let mut mask = first.mask;
    let mut differing = 0u32;
    for s in subs.iter() {
        mask &= s.mask;
        differing |= raw_id(s.id) ^ raw_id(first.id);
    }
    Some(Subscription {
        id: first.id,
        mask: mask & !differing,
    })
}

/// A callback a device is waiting on.
#[derive(Copy, Clone, PartialEq)]
enum Pending {
    None,
    Enable,
    Disable,
    Stop,
    /// The peripheral refused the frame; report the failure asynchronously.
    TxFail,
}

pub struct MuxCan<'a, C: can::Can + can::Filter + 'static> {
    can: &'a C,
    devices: List<'a, CanDevice<'a, C>>,

    /// Device whose transmission is currently with the hardware.
    inflight: OptionalCell<&'a CanDevice<'a, C>>,
    /// Where to resume scanning for the next frame to send, so that a device
    /// with a full queue cannot monopolise the peripheral.
    next_tx: Cell<usize>,

    /// The single receive buffer owned by the peripheral while receiving.
    rx_buffer: TakeCell<'static, [u8; PACKET_SIZE]>,

    /// How many devices currently want the peripheral enabled / receiving.
    enable_count: Cell<usize>,
    receive_count: Cell<usize>,
    hw_enabled: Cell<bool>,
    hw_receiving: Cell<bool>,

    deferred_call: DeferredCall,
}

impl<'a, C: can::Can + can::Filter> MuxCan<'a, C> {
    pub fn new(can: &'a C, rx_buffer: &'static mut [u8; PACKET_SIZE]) -> MuxCan<'a, C> {
        MuxCan {
            can,
            devices: List::new(),
            inflight: OptionalCell::empty(),
            next_tx: Cell::new(0),
            rx_buffer: TakeCell::new(rx_buffer),
            enable_count: Cell::new(0),
            receive_count: Cell::new(0),
            hw_enabled: Cell::new(false),
            hw_receiving: Cell::new(false),
            deferred_call: DeferredCall::new(),
        }
    }

    /// Queue a callback the hardware will not generate, because the peripheral
    /// is already in the state this device asked for.
    fn defer(&self, device: &'a CanDevice<'a, C>, pending: Pending) {
        device.pending.set(pending);
        device.pending_deferred.set(true);
        self.deferred_call.set();
    }

    // -- Receive ------------------------------------------------------------

    fn device_start_receive(&self) -> Result<(), ErrorCode> {
        self.receive_count.set(self.receive_count.get() + 1);
        if self.hw_receiving.get() {
            return Ok(());
        }
        match self.rx_buffer.take() {
            Some(buffer) => match self.can.start_receive_process(buffer) {
                Ok(()) => {
                    self.hw_receiving.set(true);
                    Ok(())
                }
                Err((e, buffer)) => {
                    self.rx_buffer.replace(buffer);
                    self.receive_count
                        .set(self.receive_count.get().saturating_sub(1));
                    Err(e)
                }
            },
            None => {
                self.receive_count
                    .set(self.receive_count.get().saturating_sub(1));
                Err(ErrorCode::NOMEM)
            }
        }
    }

    // -- Transmit -----------------------------------------------------------

    /// Hand the next queued frame to the peripheral, if it is idle.
    fn do_next_tx(&self) {
        if self.inflight.is_some() {
            return;
        }

        let count = self.devices.iter().count();
        if count == 0 {
            return;
        }
        let start = self.next_tx.get() % count;

        // Scan every device once, beginning after the one served last.
        let chosen = (0..count)
            .map(|offset| (start + offset) % count)
            .find_map(|index| {
                self.devices
                    .iter()
                    .nth(index)
                    .filter(|d| d.tx_pending.get())
                    .map(|d| (index, d))
            });

        if let Some((index, device)) = chosen {
            let id = match device.tx_id.take() {
                Some(id) => id,
                None => {
                    device.tx_pending.set(false);
                    return;
                }
            };
            if let Some(buffer) = device.tx_buffer.take() {
                device.tx_pending.set(false);
                self.next_tx.set(index + 1);
                match self.can.send(id, buffer, device.tx_len.get()) {
                    Ok(()) => {
                        self.inflight.set(device);
                    }
                    Err((_e, buffer)) => {
                        device.tx_buffer.replace(buffer);
                        self.defer(device, Pending::TxFail);
                    }
                }
            } else {
                device.tx_pending.set(false);
            }
        }
    }

    // -- Filters ------------------------------------------------------------

    /// Reprogram the hardware filter slots to cover every subscription.
    ///
    /// Best effort: failures are ignored because the software post-filter is
    /// what actually enforces the subscriptions. The only cost of failing here
    /// is extra interrupts, or -- if the peripheral rejects everything
    /// unmatched -- traffic that never arrives, which is why the covering
    /// filter fallback deliberately errs on the side of accepting too much.
    fn program_filters(&self) {
        let slots = self.can.filter_count();
        let mut used: u32 = 0;

        for extended in [false, true] {
            let mut subs: [Subscription; MAX_SUBSCRIPTIONS * 4] = [Subscription {
                id: can::Id::Standard(0),
                mask: 0,
            };
                MAX_SUBSCRIPTIONS * 4];
            let mut n = 0;

            for device in self.devices.iter() {
                for slot in device.subscriptions.iter() {
                    if let Some(s) = slot.get() {
                        if is_extended(s.id) == extended && n < subs.len() {
                            subs[n] = s;
                            n += 1;
                        }
                    }
                }
            }
            if n == 0 {
                continue;
            }

            // Try to give every subscription its own slot; if they do not all
            // fit, one covering filter stands in for the whole class.
            let exact = self.place_all(&subs[..n], &mut used, slots);
            if !exact {
                if let Some(cover) = covering_filter(&subs[..n]) {
                    let _ = self.place(cover, &mut used, slots);
                }
            }
        }

        // Anything we did not program must not linger from a previous call.
        for slot in 0..slots {
            if used & (1 << slot) == 0 {
                let _ = self.can.disable_filter(slot as u32);
            }
        }
    }

    fn place_all(&self, subs: &[Subscription], used: &mut u32, slots: usize) -> bool {
        let saved = *used;
        for s in subs.iter() {
            if !self.place(*s, used, slots) {
                *used = saved;
                return false;
            }
        }
        true
    }

    /// Install one subscription into the first free slot that accepts it.
    ///
    /// The HIL does not say how many slots hold standard versus extended
    /// identifiers, so this probes: `enable_filter` rejects a slot of the wrong
    /// class with `INVAL` and, by contract, without side effects.
    fn place(&self, sub: Subscription, used: &mut u32, slots: usize) -> bool {
        for slot in 0..slots {
            if *used & (1 << slot) != 0 {
                continue;
            }
            let params = can::FilterParameters {
                number: slot as u32,
                scale_bits: if is_extended(sub.id) {
                    can::ScaleBits::Bits32
                } else {
                    can::ScaleBits::Bits16
                },
                identifier_mode: can::IdentifierMode::Mask,
                fifo_number: 0,
                id: sub.id,
                mask: sub.mask,
            };
            if self.can.enable_filter(params).is_ok() {
                *used |= 1 << slot;
                return true;
            }
        }
        false
    }
}

impl<'a, C: can::Can + can::Filter> DeferredCallClient for MuxCan<'a, C> {
    fn register(&'static self) {
        self.deferred_call.register(self);
    }

    fn handle_deferred_call(&self) {
        for device in self.devices.iter() {
            if !device.pending_deferred.get() {
                continue;
            }
            let pending = device.pending.get();
            device.pending.set(Pending::None);
            device.pending_deferred.set(false);

            match pending {
                Pending::Enable => {
                    device.controller_client.map(|c| c.enabled(Ok(())));
                }
                Pending::Disable => {
                    device.controller_client.map(|c| c.disabled(Ok(())));
                }
                Pending::Stop => {
                    if let Some(buffer) = device.rx_buffer.take() {
                        device.receive_client.map(|c| c.stopped(buffer));
                    }
                }
                Pending::TxFail => {
                    if let Some(buffer) = device.tx_buffer.take() {
                        device.transmit_client.map(|c| {
                            c.transmit_complete(Err(can::Error::SetBySoftware), buffer)
                        });
                    }
                    self.do_next_tx();
                }
                Pending::None => {}
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Callbacks from the underlying peripheral
// ---------------------------------------------------------------------------

impl<'a, C: can::Can + can::Filter> can::ControllerClient for MuxCan<'a, C> {
    fn state_changed(&self, state: can::State) {
        // Bus state is global; every client needs to know.
        for device in self.devices.iter() {
            device.controller_client.map(|c| c.state_changed(state));
        }
    }

    fn enabled(&self, status: Result<(), ErrorCode>) {
        self.hw_enabled.set(status.is_ok());
        if status.is_err() {
            self.enable_count.set(0);
        } else {
            // Apply whatever subscriptions were registered while disabled.
            self.program_filters();
        }

        for device in self.devices.iter() {
            if device.pending.get() == Pending::Enable && !device.pending_deferred.get() {
                device.pending.set(Pending::None);
                if status.is_err() {
                    device.enabled.set(false);
                }
                device.controller_client.map(|c| c.enabled(status));
            }
        }
    }

    fn disabled(&self, status: Result<(), ErrorCode>) {
        self.hw_enabled.set(false);
        self.hw_receiving.set(false);

        for device in self.devices.iter() {
            if device.pending.get() == Pending::Disable && !device.pending_deferred.get() {
                device.pending.set(Pending::None);
                device.controller_client.map(|c| c.disabled(status));
            }
        }
    }
}

impl<'a, C: can::Can + can::Filter> can::TransmitClient<PACKET_SIZE> for MuxCan<'a, C> {
    fn transmit_complete(
        &self,
        status: Result<(), can::Error>,
        buffer: &'static mut [u8; PACKET_SIZE],
    ) {
        match self.inflight.take() {
            Some(device) => {
                device
                    .transmit_client
                    .map(move |c| c.transmit_complete(status, buffer));
            }
            None => {
                // No owner: keep the buffer rather than leak it. This should
                // not happen, but dropping a &'static mut would be permanent.
                self.rx_buffer.replace(buffer);
            }
        }
        self.do_next_tx();
    }
}

impl<'a, C: can::Can + can::Filter> can::ReceiveClient<PACKET_SIZE> for MuxCan<'a, C> {
    fn message_received(
        &self,
        id: can::Id,
        buffer: &mut [u8; PACKET_SIZE],
        len: usize,
        status: Result<(), can::Error>,
    ) {
        // `buffer` is a borrow, so every matching device can be handed the same
        // frame in turn without copying it.
        for device in self.devices.iter() {
            if device.receiving.get() && device.matches(id) {
                device
                    .receive_client
                    .map(|c| c.message_received(id, buffer, len, status));
            }
        }
    }

    fn stopped(&self, buffer: &'static mut [u8; PACKET_SIZE]) {
        self.rx_buffer.replace(buffer);
        self.hw_receiving.set(false);

        for device in self.devices.iter() {
            if device.pending.get() == Pending::Stop && !device.pending_deferred.get() {
                device.pending.set(Pending::None);
                if let Some(client_buffer) = device.rx_buffer.take() {
                    device.receive_client.map(|c| c.stopped(client_buffer));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CanDevice
// ---------------------------------------------------------------------------

pub struct CanDevice<'a, C: can::Can + can::Filter + 'static> {
    mux: &'a MuxCan<'a, C>,
    next: ListLink<'a, CanDevice<'a, C>>,

    controller_client: OptionalCell<&'static dyn can::ControllerClient>,
    transmit_client: OptionalCell<&'static dyn can::TransmitClient<PACKET_SIZE>>,
    receive_client: OptionalCell<&'static dyn can::ReceiveClient<PACKET_SIZE>>,

    subscriptions: [Cell<Option<Subscription>>; MAX_SUBSCRIPTIONS],

    tx_buffer: TakeCell<'static, [u8; PACKET_SIZE]>,
    tx_id: OptionalCell<can::Id>,
    tx_len: Cell<usize>,
    tx_pending: Cell<bool>,

    /// The client's receive buffer, parked for the lifetime of the receive
    /// process. Frames arrive as borrows from the mux, so this is only held so
    /// it can be handed back through `stopped()`.
    rx_buffer: TakeCell<'static, [u8; PACKET_SIZE]>,
    receiving: Cell<bool>,
    enabled: Cell<bool>,

    pending: Cell<Pending>,
    pending_deferred: Cell<bool>,
}

impl<'a, C: can::Can + can::Filter> CanDevice<'a, C> {
    pub fn new(mux: &'a MuxCan<'a, C>) -> CanDevice<'a, C> {
        CanDevice {
            mux,
            next: ListLink::empty(),
            controller_client: OptionalCell::empty(),
            transmit_client: OptionalCell::empty(),
            receive_client: OptionalCell::empty(),
            subscriptions: [const { Cell::new(None) }; MAX_SUBSCRIPTIONS],
            tx_buffer: TakeCell::empty(),
            tx_id: OptionalCell::empty(),
            tx_len: Cell::new(0),
            tx_pending: Cell::new(false),
            rx_buffer: TakeCell::empty(),
            receiving: Cell::new(false),
            enabled: Cell::new(false),
            pending: Cell::new(Pending::None),
            pending_deferred: Cell::new(false),
        }
    }

    /// Register with the mux. Must be called once, before use.
    pub fn setup(&'a self) {
        self.mux.devices.push_tail(self);
    }

    /// Ask to receive identifiers matching `id` under `mask`.
    ///
    /// A device with no subscriptions receives nothing.
    pub fn subscribe(&self, id: can::Id, mask: u32) -> Result<(), ErrorCode> {
        for slot in self.subscriptions.iter() {
            if slot.get().is_none() {
                slot.set(Some(Subscription { id, mask }));
                self.mux.program_filters();
                return Ok(());
            }
        }
        Err(ErrorCode::NOMEM)
    }

    /// Drop every subscription, after which this device receives nothing.
    pub fn clear_subscriptions(&self) {
        for slot in self.subscriptions.iter() {
            slot.set(None);
        }
        self.mux.program_filters();
    }

    /// Queue a callback for this device that the hardware will not generate,
    /// because the peripheral is already in the state the device asked for.
    fn defer_self(&self, pending: Pending) {
        self.pending.set(pending);
        self.pending_deferred.set(true);
        self.mux.deferred_call.set();
    }

    /// Software post-filter: does this device want `id`?
    fn matches(&self, id: can::Id) -> bool {
        self.subscriptions
            .iter()
            .filter_map(|s| s.get())
            .any(|s| s.matches(id))
    }
}

impl<'a, C: can::Can + can::Filter> ListNode<'a, CanDevice<'a, C>> for CanDevice<'a, C> {
    fn next(&'a self) -> &'a ListLink<'a, CanDevice<'a, C>> {
        &self.next
    }
}

/// Exposes the inherent subscription methods behind a trait object, so a
/// capsule can keep a device's subscriptions in sync with its own clients
/// without being generic over `CanDevice`.
impl<'a, C: can::Can + can::Filter> Subscribe for CanDevice<'a, C> {
    fn subscribe(&self, id: can::Id, mask: u32) -> Result<(), ErrorCode> {
        CanDevice::subscribe(self, id, mask)
    }

    fn clear_subscriptions(&self) {
        CanDevice::clear_subscriptions(self)
    }

    fn subscription_capacity(&self) -> usize {
        MAX_SUBSCRIPTIONS
    }
}

// ---------------------------------------------------------------------------
// CanDevice: hil::can
// ---------------------------------------------------------------------------

/// Configuration is global to the peripheral, so these calls pass straight
/// through and the *last* caller wins.
///
/// That is safe today only because bitrate, timing and mode are set once during
/// board setup. Once several processes can reach a CAN device, these must stop
/// being reachable from userspace -- see the syscall rework in
/// `can_reprogramming_design.md` section 10.5.
impl<'a, C: can::Can + can::Filter> Configure for CanDevice<'a, C> {
    const MIN_BIT_TIMINGS: can::BitTiming = C::MIN_BIT_TIMINGS;
    const MAX_BIT_TIMINGS: can::BitTiming = C::MAX_BIT_TIMINGS;
    const SYNC_SEG: u8 = C::SYNC_SEG;

    fn set_bitrate(&self, bitrate: u32) -> Result<(), ErrorCode> {
        self.mux.can.set_bitrate(bitrate)
    }

    fn set_bit_timing(&self, bit_timing: can::BitTiming) -> Result<(), ErrorCode> {
        self.mux.can.set_bit_timing(bit_timing)
    }

    fn set_operation_mode(&self, mode: can::OperationMode) -> Result<(), ErrorCode> {
        self.mux.can.set_operation_mode(mode)
    }

    fn get_bit_timing(&self) -> Result<can::BitTiming, ErrorCode> {
        self.mux.can.get_bit_timing()
    }

    fn get_operation_mode(&self) -> Result<can::OperationMode, ErrorCode> {
        self.mux.can.get_operation_mode()
    }

    fn set_automatic_retransmission(&self, automatic: bool) -> Result<(), ErrorCode> {
        self.mux.can.set_automatic_retransmission(automatic)
    }

    fn set_wake_up(&self, wake_up: bool) -> Result<(), ErrorCode> {
        self.mux.can.set_wake_up(wake_up)
    }

    fn get_automatic_retransmission(&self) -> Result<bool, ErrorCode> {
        self.mux.can.get_automatic_retransmission()
    }

    fn get_wake_up(&self) -> Result<bool, ErrorCode> {
        self.mux.can.get_wake_up()
    }

    fn receive_fifo_count(&self) -> usize {
        self.mux.can.receive_fifo_count()
    }
}

impl<'a, C: can::Can + can::Filter> Controller for CanDevice<'a, C> {
    fn set_client(&self, client: Option<&'static dyn can::ControllerClient>) {
        match client {
            Some(c) => self.controller_client.set(c),
            None => self.controller_client.clear(),
        }
    }

    fn enable(&self) -> Result<(), ErrorCode> {
        if self.enabled.get() {
            return Err(ErrorCode::ALREADY);
        }
        self.enabled.set(true);
        let mux = self.mux;
        mux.enable_count.set(mux.enable_count.get() + 1);

        if mux.hw_enabled.get() {
            // Already running for someone else; synthesise the callback.
            self.defer_self(Pending::Enable);
            return Ok(());
        }

        self.pending.set(Pending::Enable);
        self.pending_deferred.set(false);

        if mux.enable_count.get() == 1 {
            if let Err(e) = mux.can.enable() {
                mux.enable_count
                    .set(mux.enable_count.get().saturating_sub(1));
                self.pending.set(Pending::None);
                self.enabled.set(false);
                return Err(e);
            }
        }
        // Otherwise an enable from another device is already in flight, and its
        // completion notifies every device waiting on it.
        Ok(())
    }

    fn disable(&self) -> Result<(), ErrorCode> {
        if !self.enabled.get() {
            return Err(ErrorCode::ALREADY);
        }
        self.enabled.set(false);
        let mux = self.mux;
        mux.enable_count
            .set(mux.enable_count.get().saturating_sub(1));

        if mux.enable_count.get() > 0 {
            // Someone else still needs the peripheral running.
            self.defer_self(Pending::Disable);
            return Ok(());
        }

        self.pending.set(Pending::Disable);
        self.pending_deferred.set(false);
        if let Err(e) = mux.can.disable() {
            self.pending.set(Pending::None);
            return Err(e);
        }
        Ok(())
    }

    fn get_state(&self) -> Result<can::State, ErrorCode> {
        self.mux.can.get_state()
    }
}

impl<'a, C: can::Can + can::Filter> Transmit<PACKET_SIZE> for CanDevice<'a, C> {
    fn set_client(&self, client: Option<&'static dyn can::TransmitClient<PACKET_SIZE>>) {
        match client {
            Some(c) => self.transmit_client.set(c),
            None => self.transmit_client.clear(),
        }
    }

    fn send(
        &self,
        id: can::Id,
        buffer: &'static mut [u8; PACKET_SIZE],
        len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8; PACKET_SIZE])> {
        if self.tx_pending.get() || self.tx_buffer.is_some() {
            return Err((ErrorCode::BUSY, buffer));
        }
        self.tx_buffer.replace(buffer);
        self.tx_id.set(id);
        self.tx_len.set(len);
        self.tx_pending.set(true);
        self.mux.do_next_tx();
        Ok(())
    }
}

impl<'a, C: can::Can + can::Filter> Receive<PACKET_SIZE> for CanDevice<'a, C> {
    fn set_client(&self, client: Option<&'static dyn can::ReceiveClient<PACKET_SIZE>>) {
        match client {
            Some(c) => self.receive_client.set(c),
            None => self.receive_client.clear(),
        }
    }

    fn start_receive_process(
        &self,
        buffer: &'static mut [u8; PACKET_SIZE],
    ) -> Result<(), (ErrorCode, &'static mut [u8; PACKET_SIZE])> {
        if self.receiving.get() {
            return Err((ErrorCode::ALREADY, buffer));
        }
        match self.mux.device_start_receive() {
            Ok(()) => {
                self.rx_buffer.replace(buffer);
                self.receiving.set(true);
                Ok(())
            }
            Err(e) => Err((e, buffer)),
        }
    }

    fn stop_receive(&self) -> Result<(), ErrorCode> {
        if !self.receiving.get() {
            return Err(ErrorCode::ALREADY);
        }
        self.receiving.set(false);
        let mux = self.mux;
        mux.receive_count
            .set(mux.receive_count.get().saturating_sub(1));

        if mux.receive_count.get() > 0 || !mux.hw_receiving.get() {
            // Others are still receiving, so the peripheral keeps running and
            // will not produce a `stopped()`; synthesise one.
            self.defer_self(Pending::Stop);
            return Ok(());
        }

        self.pending.set(Pending::Stop);
        self.pending_deferred.set(false);
        if let Err(e) = mux.can.stop_receive() {
            self.pending.set(Pending::None);
            return Err(e);
        }
        Ok(())
    }
}

// `hil::can::Can` itself needs no impl here: the HIL provides a blanket
// implementation for anything that is Transmit + Configure + Controller +
// Receive, which `CanDevice` now is.
