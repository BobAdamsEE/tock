// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Peripheral implementations for the IMXRT1050 and IMXRT1060 MCUs.
//!
//! imxrt1050 chip: <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1050-evaluation-kit:MIMXRT1050-EVK>

#![no_std]

pub mod chip;
pub mod nvic;

// Peripherals
// pub mod ccm;
// pub mod ccm_analog;
// pub mod dcdc;
// pub mod dma;
// pub mod gpio;
// pub mod gpt;
// pub mod iomuxc;
// pub mod iomuxc_snvs;
// pub mod lpi2c;
// pub mod lpuart;

use cortexm7::{initialize_ram_jump_to_main, unhandled_interrupt, CortexM7, CortexMVariant};

extern "C" {
    // _estack is not really a function, but it makes the types work
    // You should never actually invoke it!!
    fn _estack();
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static BASE_VECTORS: [unsafe extern "C" fn(); 16] = [
    _estack,
    initialize_ram_jump_to_main,
    unhandled_interrupt,          // NMI
    CortexM7::HARD_FAULT_HANDLER, // Hard Fault
    unhandled_interrupt,          // MemManage
    unhandled_interrupt,          // BusFault
    unhandled_interrupt,          // UsageFault
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    CortexM7::SVC_HANDLER, // SVC
    unhandled_interrupt,   // DebugMon
    unhandled_interrupt,
    unhandled_interrupt,       // PendSV
    CortexM7::SYSTICK_HANDLER, // SysTick
];

// imxrt 1050 has total of 160 interrupts
#[cfg_attr(all(target_arch = "arm", target_os = "none"), link_section = ".irqs")]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static IRQS: [unsafe extern "C" fn(); 74] = [
    CortexM7::GENERIC_ISR, // SUPC (0)
    CortexM7::GENERIC_ISR, // RSTC (1)
    CortexM7::GENERIC_ISR, // RTC (2)
    CortexM7::GENERIC_ISR, // RTT (3)
    CortexM7::GENERIC_ISR, // WDT (4)
    CortexM7::GENERIC_ISR, // PMC (5)
    CortexM7::GENERIC_ISR, // EFC (6)
    CortexM7::GENERIC_ISR, // UART0 (7)
    CortexM7::GENERIC_ISR, // UART1 (8)
    CortexM7::GENERIC_ISR, // SMC (9)
    CortexM7::GENERIC_ISR, // PIOA (10)
    CortexM7::GENERIC_ISR, // PIOB (11)
    CortexM7::GENERIC_ISR, // PIOC (12)
    CortexM7::GENERIC_ISR, // USART0 (13)
    CortexM7::GENERIC_ISR, // USART1 (14)
    CortexM7::GENERIC_ISR, // USART2 (15)
    CortexM7::GENERIC_ISR, // PIOD (16)
    CortexM7::GENERIC_ISR, // PIOE (17)
    CortexM7::GENERIC_ISR, // HSMCI (18)
    CortexM7::GENERIC_ISR, // TWIHS0 (19)
    CortexM7::GENERIC_ISR, // TWIHS1 (20)
    CortexM7::GENERIC_ISR, // SPI0 (21)
    CortexM7::GENERIC_ISR, // SSC (22)
    CortexM7::GENERIC_ISR, // TC0_CHANNEL0 (23)
    CortexM7::GENERIC_ISR, // TC0_CHANNEL1 (24)
    CortexM7::GENERIC_ISR, // TC0_CHANNEL2 (25)
    CortexM7::GENERIC_ISR, // TC1_CHANNEL0 (26)
    CortexM7::GENERIC_ISR, // TC1_CHANNEL1 (27)
    CortexM7::GENERIC_ISR, // TC1_CHANNEL2 (28)
    CortexM7::GENERIC_ISR, // AFEC0 (29)
    CortexM7::GENERIC_ISR, // DACC (30)
    CortexM7::GENERIC_ISR, // PWM0 (31)
    CortexM7::GENERIC_ISR, // ICM (32)
    CortexM7::GENERIC_ISR, // ACC (33)
    CortexM7::GENERIC_ISR, // USBHS (34)
    CortexM7::GENERIC_ISR, // MCAN0 (35)
    CortexM7::GENERIC_ISR, // MCAN0 (36)
    CortexM7::GENERIC_ISR, // MCAN1 (37)
    CortexM7::GENERIC_ISR, // MCAN1 (38)
    CortexM7::GENERIC_ISR, // GMAC (39)
    CortexM7::GENERIC_ISR, // AFEC1 (40)
    CortexM7::GENERIC_ISR, // TWIHS2 (41)
    CortexM7::GENERIC_ISR, // SPI1 (42)
    CortexM7::GENERIC_ISR, // QSPI (43)
    CortexM7::GENERIC_ISR, // UART2 (44)
    CortexM7::GENERIC_ISR, // UART3 (45)
    CortexM7::GENERIC_ISR, // UART4 (46)
    CortexM7::GENERIC_ISR, // TC2_CHANNEL0 (47)
    CortexM7::GENERIC_ISR, // TC2_CHANNEL1 (48)
    CortexM7::GENERIC_ISR, // TC2_CHANNEL2 (49)
    CortexM7::GENERIC_ISR, // TC3_CHANNEL0 (50)
    CortexM7::GENERIC_ISR, // TC3_CHANNEL1 (51)
    CortexM7::GENERIC_ISR, // TC3_CHANNEL2 (52)
    CortexM7::GENERIC_ISR, // MLB (53)
    CortexM7::GENERIC_ISR, // MLB (54)
    CortexM7::GENERIC_ISR, // Reserved (55)
    CortexM7::GENERIC_ISR, // AES (56)
    CortexM7::GENERIC_ISR, // TRNG (57)
    CortexM7::GENERIC_ISR, // XDMAC (58)
    CortexM7::GENERIC_ISR, // ISI (59)
    CortexM7::GENERIC_ISR, // PWM1 (60)
    CortexM7::GENERIC_ISR, // ARM (61)
    CortexM7::GENERIC_ISR, // Reserved (62)
    CortexM7::GENERIC_ISR, // RSWDT (63)
    CortexM7::GENERIC_ISR, // ARM ECC WARNING (64)
    CortexM7::GENERIC_ISR, // ARM ECC FAULT (65)
    CortexM7::GENERIC_ISR, // GMAC Queue 1 (66)
    CortexM7::GENERIC_ISR, // GMAC Queue 2 (67)
    CortexM7::GENERIC_ISR, // ARM IXC (68)
    CortexM7::GENERIC_ISR, // I2SC0 (69)
    CortexM7::GENERIC_ISR, // I2SC1 (70)
    CortexM7::GENERIC_ISR, // GMAC Queue 3 (71)
    CortexM7::GENERIC_ISR, // GMAC Queue 4 (72)
    CortexM7::GENERIC_ISR, // GMAC Queue 5 (73)
];

pub unsafe fn init() {
    cortexm7::nvic::disable_all();
    cortexm7::nvic::clear_all_pending();

    cortexm7::scb::set_vector_table_offset(core::ptr::addr_of!(BASE_VECTORS) as *const ());

    cortexm7::nvic::enable_all();
}
