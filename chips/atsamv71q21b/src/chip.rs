// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Chip trait setup.

use core::fmt::Write;
use cortexm7::{CortexM7, CortexMVariant};
use kernel::platform::chip::{Chip, InterruptService};

use crate::gpio;
use crate::nvic;

pub struct Atsamv71q21b<I: InterruptService + 'static> {
    mpu: cortexm7::mpu::MPU,
    userspace_kernel_boundary: cortexm7::syscall::SysCall,
    interrupt_service: &'static I,
}

impl<I: InterruptService + 'static> Atsamv71q21b<I> {
    pub unsafe fn new(interrupt_service: &'static I) -> Self {
        Atsamv71q21b {
            mpu: cortexm7::mpu::new(),
            userspace_kernel_boundary: cortexm7::syscall::SysCall::new(),
            interrupt_service,
        }
    }
}

pub struct Atsamv71q21bDefaultPeripherals {
    pub pa: crate::gpio::Port<'static, 32>,
}

impl Atsamv71q21bDefaultPeripherals {
    //The new function is where the power manager could be referenced
    pub fn new() -> Self {
        Self {
            pa: crate::gpio::Port::new_port_a(),
        }
    }
}

impl InterruptService for Atsamv71q21bDefaultPeripherals {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        /*match interrupt {
            nvic::LPUART1 => self.lpuart1.handle_interrupt(),
            nvic::LPUART2 => self.lpuart2.handle_interrupt(),
            nvic::LPI2C1 => self.lpi2c1.handle_event(),
            nvic::GPT1 => self.gpt1.handle_interrupt(),
            nvic::GPT2 => self.gpt2.handle_interrupt(),
            nvic::GPIO1_1 => self.ports.gpio1.handle_interrupt(),
            nvic::GPIO1_2 => self.ports.gpio1.handle_interrupt(),
            nvic::GPIO2_1 => self.ports.gpio2.handle_interrupt(),
            nvic::GPIO2_2 => self.ports.gpio2.handle_interrupt(),
            nvic::GPIO3_1 => self.ports.gpio3.handle_interrupt(),
            nvic::GPIO3_2 => self.ports.gpio3.handle_interrupt(),
            nvic::GPIO4_1 => self.ports.gpio4.handle_interrupt(),
            nvic::GPIO4_2 => self.ports.gpio4.handle_interrupt(),
            nvic::GPIO5_1 => self.ports.gpio5.handle_interrupt(),
            nvic::GPIO5_2 => self.ports.gpio5.handle_interrupt(),
            nvic::SNVS_LP_WRAPPER => debug!("Interrupt: SNVS_LP_WRAPPER"),
            nvic::DMA0_16..=nvic::DMA15_31 => {
                let low = (interrupt - nvic::DMA0_16) as usize;
                let high = low + 16;
                for channel in [&self.dma.channels[low], &self.dma.channels[high]] {
                    if channel.is_interrupt() | channel.is_error() {
                        channel.handle_interrupt();
                    }
                }
            }
            nvic::DMA_ERROR => {
                while let Some(channel) = self.dma.error_channel() {
                    channel.handle_interrupt();
                }
            }
            _ => {
                return false;
            }
        }*/
        true
    }
}

impl<I: InterruptService + 'static> Chip for Atsamv71q21b<I> {
    type MPU = cortexm7::mpu::MPU;
    type UserspaceKernelBoundary = cortexm7::syscall::SysCall;

    fn service_pending_interrupts(&self) {
        unsafe {
            while let Some(interrupt) = cortexm7::nvic::next_pending() {
                let handled = self.interrupt_service.service_interrupt(interrupt);
                assert!(handled, "Unhandled interrupt number {}", interrupt);
                let n = cortexm7::nvic::Nvic::new(interrupt);
                n.clear_pending();
                n.enable();
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { cortexm7::nvic::has_pending() }
    }

    fn mpu(&self) -> &cortexm7::mpu::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &cortexm7::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn sleep(&self) {
        unsafe {
            cortexm7::scb::unset_sleepdeep();
            cortexm7::support::wfi();
        }
    }

    unsafe fn with_interrupts_disabled<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm7::support::with_interrupts_disabled(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        CortexM7::print_cortexm_state(write);
    }
}
