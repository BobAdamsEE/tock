// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Chip trait setup.

use core::fmt::Write;
use cortexm7::{CortexM7, CortexMVariant};
use kernel::platform::chip::{Chip, InterruptService};

use crate::efc;
use crate::mcan;
use crate::nvic;
use crate::tc;
use crate::twihs;
use crate::uart;
use crate::xdmac;

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
    pub pa: crate::gpio::PortA<'static>,
    pub pb: crate::gpio::PortB<'static>,
    pub pc: crate::gpio::PortC<'static>,
    pub pd: crate::gpio::PortD<'static>,
    pub pe: crate::gpio::PortE<'static>,
    pub usart1: uart::Usart1<'static>,
    pub tc0: tc::Tc<'static>,
    pub efc: efc::Efc,
    pub xdmac: xdmac::Xdmac,
    pub twihs0: twihs::Twihs<'static>,
    pub mcan1: mcan::Mcan,
}

impl Atsamv71q21bDefaultPeripherals {
    pub fn new(mcan1_msg_ram: &'static mut mcan::MessageRam) -> Self {
        Self {
            pa: crate::gpio::PortA::new_port_a(),
            pb: crate::gpio::PortB::new_port_b(),
            pc: crate::gpio::PortC::new_port_c(),
            pd: crate::gpio::PortD::new_port_d(),
            pe: crate::gpio::PortE::new_port_e(),
            usart1: uart::Usart1::new(),
            tc0: tc::Tc::new(),
            efc: efc::Efc::new(),
            xdmac: xdmac::Xdmac::new(),
            twihs0: twihs::Twihs::new_twihs0(),
            mcan1: mcan::Mcan::new_mcan1(mcan1_msg_ram),
        }
    }
}

impl InterruptService for Atsamv71q21bDefaultPeripherals {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            nvic::EFC      => self.efc.handle_interrupt(),
            nvic::USART1   => self.usart1.handle_interrupt(),
            nvic::XDMAC    => { self.xdmac.handle_interrupt(); }
            nvic::TC0_CH0  => self.tc0.handle_interrupt(),
            nvic::PIOA     => self.pa.handle_interrupt(),
            nvic::PIOB     => self.pb.handle_interrupt(),
            nvic::PIOC     => self.pc.handle_interrupt(),
            nvic::PIOD     => self.pd.handle_interrupt(),
            nvic::PIOE     => self.pe.handle_interrupt(),
            nvic::TWIHS0     => self.twihs0.handle_interrupt(),
            nvic::MCAN1_INT0 => self.mcan1.handle_interrupt(),
            nvic::MCAN1_INT1 => self.mcan1.handle_interrupt(),
            // Correctable ECC error — hardware has already corrected the data,
            // safe to continue. Uncorrectable ECC is a hard fault.
            nvic::ECC_WARNING => {}
            nvic::ECC_FAULT => panic!("Uncorrectable ECC fault"),
            _ => return false,
        }
        true
    }
}

impl<I: InterruptService + 'static> Chip for Atsamv71q21b<I> {
    type MPU = cortexm7::mpu::MPU;
    type UserspaceKernelBoundary = cortexm7::syscall::SysCall;
    type ThreadIdProvider = cortexm7::thread_id::CortexMThreadIdProvider;

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

    unsafe fn print_state(_this: Option<&Self>, write: &mut dyn Write) {
        CortexM7::print_cortexm_state(write);
    }
}
