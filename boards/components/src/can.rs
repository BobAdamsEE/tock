// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022
// Copyright OxidOS Automotive SRL 2022
//
// Author: Teona Severin <teona.severin@oxidos.io>

//! Component for CAN syscall interface.
//!
//! This provides one Component, `CanComponent`, which implements a
//! userspace syscall interface to the Can peripheral.
//!
//! Usage
//! -----
//! ```rust
//! let can = components::can::CanComponent::new(
//!     board_kernel,
//!     capsules_extra::can::DRIVER_NUM,
//!     &peripherals.can1
//! ).finalize(components::can_component_static!(
//!     stm32f429zi::can::Can<'static>
//! ));
//! ```
//!

use capsules_core::virtualizers::virtual_can::{CanDevice, MuxCan, Subscribe};
use capsules_extra::can::CanCapsule;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::deferred_call::DeferredCallClient;
use kernel::hil::can;
use kernel::{capabilities, create_capability};

#[macro_export]
macro_rules! can_component_static {
    ($C:ty $(,)?) => {{
        use capsules_extra::can::CanCapsule;
        use core::mem::MaybeUninit;
        use kernel::hil::can;
        use kernel::static_buf;

        let CAN_TX_BUF = static_buf!([u8; can::STANDARD_CAN_PACKET_SIZE]);
        let CAN_RX_BUF = static_buf!([u8; can::STANDARD_CAN_PACKET_SIZE]);
        let can = static_buf!(capsules_extra::can::CanCapsule<'static, $C>);
        (can, CAN_TX_BUF, CAN_RX_BUF)
    };};
}

pub struct CanComponent<A: 'static + can::Can> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    can: &'static A,
    subscriber: Option<&'static dyn Subscribe>,
}

impl<A: 'static + can::Can> CanComponent<A> {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        can: &'static A,
    ) -> CanComponent<A> {
        CanComponent {
            board_kernel,
            driver_num,
            can,
            subscriber: None,
        }
    }

    /// Let the capsule keep the underlying device's subscriptions in step with
    /// the union of its processes'.
    ///
    /// Pass the same `CanDevice` given to `new`. Without this the capsule
    /// still filters per process in software, but acceptance stays fixed at
    /// whatever the board configured.
    pub fn with_subscriber(mut self, subscriber: &'static dyn Subscribe) -> Self {
        self.subscriber = Some(subscriber);
        self
    }
}

impl<A: 'static + can::Can> Component for CanComponent<A> {
    type StaticInput = (
        &'static mut MaybeUninit<CanCapsule<'static, A>>,
        &'static mut MaybeUninit<[u8; can::STANDARD_CAN_PACKET_SIZE]>,
        &'static mut MaybeUninit<[u8; can::STANDARD_CAN_PACKET_SIZE]>,
    );
    type Output = &'static CanCapsule<'static, A>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);
        let grant_can = self.board_kernel.create_grant(self.driver_num, &grant_cap);

        let can = static_buffer.0.write(capsules_extra::can::CanCapsule::new(
            self.can,
            self.subscriber,
            grant_can,
            static_buffer.1.write([0; can::STANDARD_CAN_PACKET_SIZE]),
            static_buffer.2.write([0; can::STANDARD_CAN_PACKET_SIZE]),
        ));
        can::Controller::set_client(self.can, Some(can));
        can::Transmit::set_client(self.can, Some(can));
        can::Receive::set_client(self.can, Some(can));

        can
    }
}

// ---------------------------------------------------------------------------
// CAN virtualization
// ---------------------------------------------------------------------------

// Components for sharing one CAN peripheral between several clients.
//
// `CanMuxComponent` wraps the peripheral once; each client then gets its own
// `CanDeviceComponent`, which implements `hil::can::Can` and so can be handed
// to `CanComponent` (or any in-kernel capsule) exactly like the bare
// peripheral.
//
// Usage
// -----
// ```rust,ignore
// let can_mux = components::can::CanMuxComponent::new(&peripherals.mcan1)
//     .finalize(components::can_mux_component_static!(mcan::Mcan));
//
// let can_device = components::can::CanDeviceComponent::new(can_mux)
//     .finalize(components::can_device_component_static!(mcan::Mcan));
//
// let can = components::can::CanComponent::new(
//     board_kernel, capsules_extra::can::DRIVER_NUM, can_device)
//     .finalize(components::can_component_static!(
//         capsules_core::virtualizers::virtual_can::CanDevice<'static, mcan::Mcan>));
// ```

#[macro_export]
macro_rules! can_mux_component_static {
    ($C:ty $(,)?) => {{
        use capsules_core::virtualizers::virtual_can::MuxCan;
        use core::mem::MaybeUninit;
        use kernel::hil::can;
        use kernel::static_buf;

        let mux = static_buf!(MuxCan<'static, $C>);
        let rx = static_buf!([u8; can::STANDARD_CAN_PACKET_SIZE]);
        (mux, rx)
    };};
}

#[macro_export]
macro_rules! can_device_component_static {
    ($C:ty $(,)?) => {{
        use capsules_core::virtualizers::virtual_can::CanDevice;
        use core::mem::MaybeUninit;
        use kernel::static_buf;

        static_buf!(CanDevice<'static, $C>)
    };};
}

pub struct CanMuxComponent<C: 'static + can::Can + can::Filter> {
    can: &'static C,
}

impl<C: 'static + can::Can + can::Filter> CanMuxComponent<C> {
    pub fn new(can: &'static C) -> CanMuxComponent<C> {
        CanMuxComponent { can }
    }
}

impl<C: 'static + can::Can + can::Filter> Component for CanMuxComponent<C> {
    type StaticInput = (
        &'static mut MaybeUninit<MuxCan<'static, C>>,
        &'static mut MaybeUninit<[u8; can::STANDARD_CAN_PACKET_SIZE]>,
    );
    type Output = &'static MuxCan<'static, C>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let rx_buffer = s.1.write([0; can::STANDARD_CAN_PACKET_SIZE]);
        let mux = s.0.write(MuxCan::new(self.can, rx_buffer));

        // The mux is the peripheral's only client; it fans out from here.
        can::Controller::set_client(self.can, Some(mux));
        can::Transmit::set_client(self.can, Some(mux));
        can::Receive::set_client(self.can, Some(mux));

        // Needed for callbacks the hardware will not generate, e.g. enabling a
        // peripheral that another device already enabled.
        mux.register();

        mux
    }
}

pub struct CanDeviceComponent<C: 'static + can::Can + can::Filter> {
    mux: &'static MuxCan<'static, C>,
}

impl<C: 'static + can::Can + can::Filter> CanDeviceComponent<C> {
    pub fn new(mux: &'static MuxCan<'static, C>) -> CanDeviceComponent<C> {
        CanDeviceComponent { mux }
    }
}

impl<C: 'static + can::Can + can::Filter> Component for CanDeviceComponent<C> {
    type StaticInput = &'static mut MaybeUninit<CanDevice<'static, C>>;
    type Output = &'static CanDevice<'static, C>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let device = s.write(CanDevice::new(self.mux));
        device.setup();
        device
    }
}
