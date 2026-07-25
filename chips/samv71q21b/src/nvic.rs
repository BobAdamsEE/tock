// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Named constants for NVIC ids — SAMV71Q21B peripheral interrupt numbers.
//!
//! Numbers match the IRQS table in lib.rs and the SAMV71Q21B datasheet
//! section "Peripheral Identifiers" (Table 9-1).

pub const SUPC: u32 = 0;
pub const RSTC: u32 = 1;
pub const RTC: u32 = 2;
pub const RTT: u32 = 3;
pub const WDT: u32 = 4;
pub const PMC: u32 = 5;
pub const EFC: u32 = 6;
pub const UART0: u32 = 7;
pub const UART1: u32 = 8;
pub const SMC: u32 = 9;
pub const PIOA: u32 = 10;
pub const PIOB: u32 = 11;
pub const PIOC: u32 = 12;
pub const USART0: u32 = 13;
pub const USART1: u32 = 14;
pub const USART2: u32 = 15;
pub const PIOD: u32 = 16;
pub const PIOE: u32 = 17;
pub const HSMCI: u32 = 18;
pub const TWIHS0: u32 = 19;
pub const TWIHS1: u32 = 20;
pub const SPI0: u32 = 21;
pub const SSC: u32 = 22;
pub const TC0_CH0: u32 = 23;
pub const TC0_CH1: u32 = 24;
pub const TC0_CH2: u32 = 25;
pub const TC1_CH0: u32 = 26;
pub const TC1_CH1: u32 = 27;
pub const TC1_CH2: u32 = 28;
pub const AFEC0: u32 = 29;
pub const DACC: u32 = 30;
pub const PWM0: u32 = 31;
pub const ICM: u32 = 32;
pub const ACC: u32 = 33;
pub const USBHS: u32 = 34;
pub const MCAN0_INT0: u32 = 35;
pub const MCAN0_INT1: u32 = 36;
pub const MCAN1_INT0: u32 = 37;
pub const MCAN1_INT1: u32 = 38;
pub const GMAC: u32 = 39;
pub const AFEC1: u32 = 40;
pub const TWIHS2: u32 = 41;
pub const SPI1: u32 = 42;
pub const QSPI: u32 = 43;
pub const UART2: u32 = 44;
pub const UART3: u32 = 45;
pub const UART4: u32 = 46;
pub const TC2_CH0: u32 = 47;
pub const TC2_CH1: u32 = 48;
pub const TC2_CH2: u32 = 49;
pub const TC3_CH0: u32 = 50;
pub const TC3_CH1: u32 = 51;
pub const TC3_CH2: u32 = 52;
pub const MLB_INT0: u32 = 53;
pub const MLB_INT1: u32 = 54;
// 55 reserved
pub const AES: u32 = 56;
pub const TRNG: u32 = 57;
pub const XDMAC: u32 = 58;
pub const ISI: u32 = 59;
pub const PWM1: u32 = 60;
// 61 ARM (FPU)
// 62 reserved
pub const RSWDT: u32 = 63;
pub const ECC_WARNING: u32 = 64; // Cortex-M7 correctable ECC error
pub const ECC_FAULT: u32 = 65;   // Cortex-M7 uncorrectable ECC error
pub const GMAC_Q1: u32 = 66;
pub const GMAC_Q2: u32 = 67;
// 68 ARM IXC
pub const I2SC0: u32 = 69;
pub const I2SC1: u32 = 70;
pub const GMAC_Q3: u32 = 71;
pub const GMAC_Q4: u32 = 72;
pub const GMAC_Q5: u32 = 73;
