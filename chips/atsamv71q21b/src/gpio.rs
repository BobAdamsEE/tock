// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Implementation of the GPIO controller for the SAMV71.

use core::ops::{Index, IndexMut};
use core::sync::atomic::{AtomicUsize, Ordering};
use kernel::hil;
use kernel::hil::gpio;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly,
};
use kernel::utilities::StaticRef;

const BASE_ADDRESS: usize = 0x400E_0E00;
const SIZE: usize = 0x200;

register_structs! {
    GpioRegisters {
        (0x000 => per: WriteOnly<u32, PER::Register>),
        (0x004 => pdr: WriteOnly<u32, PDR::Register>),
        (0x008 => psr: ReadOnly<u32, PSR::Register>),
        (0x00C => _reserved0),
        (0x010 => oer: WriteOnly<u32, OER::Register>),
        (0x014 => odr: WriteOnly<u32, ODR::Register>),
        (0x018 => osr: ReadOnly<u32, OSR::Register>),
        (0x01C => _reserved1),
        (0x020 => ifer: WriteOnly<u32, IFER::Register>),
        (0x024 => ifdr: WriteOnly<u32, IFDR::Register>),
        (0x028 => ifsr: ReadOnly<u32, IFSR::Register>),
        (0x02C => _reserved2),
        (0x030 => sodr: WriteOnly<u32, SODR::Register>),
        (0x034 => codr: WriteOnly<u32, CODR::Register>),
        (0x038 => odsr: ReadWrite<u32, ODSR::Register>),
        (0x03C => pdsr: ReadOnly<u32, PDSR::Register>),
        (0x040 => ier: WriteOnly<u32, IER::Register>),
        (0x044 => idr: WriteOnly<u32, IDR::Register>),
        (0x048 => imr: ReadOnly<u32, IMR::Register>),
        (0x04C => isr: ReadOnly<u32, ISR::Register>),
        (0x050 => mder: WriteOnly<u32, MDER::Register>),
        (0x054 => mddr: WriteOnly<u32, MDDR::Register>),
        (0x058 => mdsr: ReadOnly<u32, MDSR::Register>),
        (0x05C => _reserved3),
        (0x060 => pudr: WriteOnly<u32, PUDR::Register>),
        (0x064 => puer: WriteOnly<u32, PUER::Register>),
        (0x068 => pusr: ReadOnly<u32, PUSR::Register>),
        (0x06C => _reserved4),
        (0x070 => abcdsr_0: ReadWrite<u32, ABCDSR0::Register>),
        (0x074 => abcdsr_1: ReadWrite<u32, ABCDSR1::Register>),
        (0x078 => _reserved5),
        (0x080 => ifscdr: WriteOnly<u32, IFSCDR::Register>),
        (0x084 => ifscer: WriteOnly<u32, IFSCER::Register>),
        (0x088 => ifscsr: ReadOnly<u32, IFSCSR::Register>),
        (0x08C => scdr: ReadWrite<u32>),
        (0x090 => ppddr: WriteOnly<u32, PPDDR::Register>),
        (0x094 => ppder: WriteOnly<u32, PPDER::Register>),
        (0x098 => ppdsr: ReadOnly<u32, PPDSR::Register>),
        (0x09C => _reserved6),
        (0x0A0 => ower: WriteOnly<u32, OWER::Register>),
        (0x0A4 => owdr: WriteOnly<u32, OWDR::Register>),
        (0x0A8 => owsr: ReadOnly<u32, OWSR::Register>),
        (0x0AC => _reserved7),
        (0x0B0 => aimer: WriteOnly<u32, AIMER::Register>),
        (0x0B4 => aimdr: WriteOnly<u32, AIMDR::Register>),
        (0x0B8 => aimmr: ReadOnly<u32, AIMMR::Register>),
        (0x0BC => _reserved8),
        (0x0C0 => esr: WriteOnly<u32, ESR::Register>),
        (0x0C4 => lsr: WriteOnly<u32, LSR::Register>),
        (0x0C8 => elsr: ReadOnly<u32, ELSR::Register>),
        (0x0CC => _reserved9),
        (0x0D0 => fellsr: WriteOnly<u32, FELLSR::Register>),
        (0x0D4 => rehlsr: WriteOnly<u32, REHLSR::Register>),
        (0x0D8 => frlhsr: ReadOnly<u32, FRLHSR::Register>),
        (0x0DC => _reserved10),
        (0x0E0 => locksr: ReadOnly<u32, LOCKSR::Register>),
        (0x0E4 => wpmr: ReadWrite<u32, WPMR::Register>),
        (0x0E8 => wpsr: ReadOnly<u32, WPSR::Register>),
        (0x0EC => _reserved11),
        (0x100 => schmitt: ReadWrite<u32, SCHMITT::Register>),
        (0x104 => _reserved12),
        (0x118 => driver: ReadWrite<u32, DRIVER::Register>),
        (0x11C => _reserved13),
        (0x150 => pcmr: ReadWrite<u32, PCMR::Register>),
        (0x154 => pcier: WriteOnly<u32, PCIER::Register>),
        (0x158 => pcidr: WriteOnly<u32, PCIDR::Register>),
        (0x15C => pcimr: ReadOnly<u32, PCIMR::Register>),
        (0x160 => pcisr: ReadOnly<u32, PCISR::Register>),
        (0x164 => pcrhr: ReadOnly<u32>),
        (0x168 => @END),
    }
}
register_bitfields![u32,
PER [
    /// PIO Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// PIO Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// PIO Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// PIO Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// PIO Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// PIO Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// PIO Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// PIO Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// PIO Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// PIO Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// PIO Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// PIO Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// PIO Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// PIO Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// PIO Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// PIO Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// PIO Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// PIO Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// PIO Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// PIO Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// PIO Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// PIO Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// PIO Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// PIO Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// PIO Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// PIO Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// PIO Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// PIO Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// PIO Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// PIO Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// PIO Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// PIO Enable
    P31 OFFSET(31) NUMBITS(1) []
],
PDR [
    /// PIO Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// PIO Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// PIO Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// PIO Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// PIO Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// PIO Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// PIO Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// PIO Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// PIO Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// PIO Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// PIO Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// PIO Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// PIO Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// PIO Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// PIO Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// PIO Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// PIO Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// PIO Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// PIO Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// PIO Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// PIO Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// PIO Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// PIO Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// PIO Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// PIO Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// PIO Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// PIO Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// PIO Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// PIO Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// PIO Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// PIO Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// PIO Disable
    P31 OFFSET(31) NUMBITS(1) []
],
PSR [
    /// PIO Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// PIO Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// PIO Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// PIO Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// PIO Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// PIO Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// PIO Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// PIO Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// PIO Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// PIO Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// PIO Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// PIO Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// PIO Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// PIO Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// PIO Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// PIO Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// PIO Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// PIO Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// PIO Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// PIO Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// PIO Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// PIO Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// PIO Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// PIO Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// PIO Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// PIO Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// PIO Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// PIO Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// PIO Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// PIO Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// PIO Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// PIO Status
    P31 OFFSET(31) NUMBITS(1) []
],
OER [
    /// Output Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Enable
    P31 OFFSET(31) NUMBITS(1) []
],
ODR [
    /// Output Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Disable
    P31 OFFSET(31) NUMBITS(1) []
],
OSR [
    /// Output Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Status
    P31 OFFSET(31) NUMBITS(1) []
],
IFER [
    /// Input Filter Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Filter Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Filter Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Filter Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Filter Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Filter Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Filter Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Filter Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Filter Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Filter Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Filter Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Filter Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Filter Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Filter Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Filter Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Filter Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Filter Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Filter Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Filter Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Filter Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Filter Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Filter Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Filter Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Filter Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Filter Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Filter Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Filter Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Filter Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Filter Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Filter Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Filter Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Filter Enable
    P31 OFFSET(31) NUMBITS(1) []
],
IFDR [
    /// Input Filter Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Filter Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Filter Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Filter Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Filter Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Filter Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Filter Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Filter Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Filter Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Filter Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Filter Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Filter Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Filter Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Filter Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Filter Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Filter Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Filter Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Filter Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Filter Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Filter Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Filter Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Filter Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Filter Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Filter Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Filter Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Filter Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Filter Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Filter Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Filter Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Filter Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Filter Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Filter Disable
    P31 OFFSET(31) NUMBITS(1) []
],
IFSR [
    /// Input Filter Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Filter Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Filter Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Filter Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Filter Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Filter Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Filter Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Filter Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Filter Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Filter Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Filter Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Filter Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Filter Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Filter Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Filter Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Filter Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Filter Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Filter Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Filter Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Filter Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Filter Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Filter Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Filter Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Filter Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Filter Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Filter Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Filter Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Filter Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Filter Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Filter Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Filter Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Filter Status
    P31 OFFSET(31) NUMBITS(1) []
],
SODR [
    /// Set Output Data
    P0 OFFSET(0) NUMBITS(1) [],
    /// Set Output Data
    P1 OFFSET(1) NUMBITS(1) [],
    /// Set Output Data
    P2 OFFSET(2) NUMBITS(1) [],
    /// Set Output Data
    P3 OFFSET(3) NUMBITS(1) [],
    /// Set Output Data
    P4 OFFSET(4) NUMBITS(1) [],
    /// Set Output Data
    P5 OFFSET(5) NUMBITS(1) [],
    /// Set Output Data
    P6 OFFSET(6) NUMBITS(1) [],
    /// Set Output Data
    P7 OFFSET(7) NUMBITS(1) [],
    /// Set Output Data
    P8 OFFSET(8) NUMBITS(1) [],
    /// Set Output Data
    P9 OFFSET(9) NUMBITS(1) [],
    /// Set Output Data
    P10 OFFSET(10) NUMBITS(1) [],
    /// Set Output Data
    P11 OFFSET(11) NUMBITS(1) [],
    /// Set Output Data
    P12 OFFSET(12) NUMBITS(1) [],
    /// Set Output Data
    P13 OFFSET(13) NUMBITS(1) [],
    /// Set Output Data
    P14 OFFSET(14) NUMBITS(1) [],
    /// Set Output Data
    P15 OFFSET(15) NUMBITS(1) [],
    /// Set Output Data
    P16 OFFSET(16) NUMBITS(1) [],
    /// Set Output Data
    P17 OFFSET(17) NUMBITS(1) [],
    /// Set Output Data
    P18 OFFSET(18) NUMBITS(1) [],
    /// Set Output Data
    P19 OFFSET(19) NUMBITS(1) [],
    /// Set Output Data
    P20 OFFSET(20) NUMBITS(1) [],
    /// Set Output Data
    P21 OFFSET(21) NUMBITS(1) [],
    /// Set Output Data
    P22 OFFSET(22) NUMBITS(1) [],
    /// Set Output Data
    P23 OFFSET(23) NUMBITS(1) [],
    /// Set Output Data
    P24 OFFSET(24) NUMBITS(1) [],
    /// Set Output Data
    P25 OFFSET(25) NUMBITS(1) [],
    /// Set Output Data
    P26 OFFSET(26) NUMBITS(1) [],
    /// Set Output Data
    P27 OFFSET(27) NUMBITS(1) [],
    /// Set Output Data
    P28 OFFSET(28) NUMBITS(1) [],
    /// Set Output Data
    P29 OFFSET(29) NUMBITS(1) [],
    /// Set Output Data
    P30 OFFSET(30) NUMBITS(1) [],
    /// Set Output Data
    P31 OFFSET(31) NUMBITS(1) []
],
CODR [
    /// Clear Output Data
    P0 OFFSET(0) NUMBITS(1) [],
    /// Clear Output Data
    P1 OFFSET(1) NUMBITS(1) [],
    /// Clear Output Data
    P2 OFFSET(2) NUMBITS(1) [],
    /// Clear Output Data
    P3 OFFSET(3) NUMBITS(1) [],
    /// Clear Output Data
    P4 OFFSET(4) NUMBITS(1) [],
    /// Clear Output Data
    P5 OFFSET(5) NUMBITS(1) [],
    /// Clear Output Data
    P6 OFFSET(6) NUMBITS(1) [],
    /// Clear Output Data
    P7 OFFSET(7) NUMBITS(1) [],
    /// Clear Output Data
    P8 OFFSET(8) NUMBITS(1) [],
    /// Clear Output Data
    P9 OFFSET(9) NUMBITS(1) [],
    /// Clear Output Data
    P10 OFFSET(10) NUMBITS(1) [],
    /// Clear Output Data
    P11 OFFSET(11) NUMBITS(1) [],
    /// Clear Output Data
    P12 OFFSET(12) NUMBITS(1) [],
    /// Clear Output Data
    P13 OFFSET(13) NUMBITS(1) [],
    /// Clear Output Data
    P14 OFFSET(14) NUMBITS(1) [],
    /// Clear Output Data
    P15 OFFSET(15) NUMBITS(1) [],
    /// Clear Output Data
    P16 OFFSET(16) NUMBITS(1) [],
    /// Clear Output Data
    P17 OFFSET(17) NUMBITS(1) [],
    /// Clear Output Data
    P18 OFFSET(18) NUMBITS(1) [],
    /// Clear Output Data
    P19 OFFSET(19) NUMBITS(1) [],
    /// Clear Output Data
    P20 OFFSET(20) NUMBITS(1) [],
    /// Clear Output Data
    P21 OFFSET(21) NUMBITS(1) [],
    /// Clear Output Data
    P22 OFFSET(22) NUMBITS(1) [],
    /// Clear Output Data
    P23 OFFSET(23) NUMBITS(1) [],
    /// Clear Output Data
    P24 OFFSET(24) NUMBITS(1) [],
    /// Clear Output Data
    P25 OFFSET(25) NUMBITS(1) [],
    /// Clear Output Data
    P26 OFFSET(26) NUMBITS(1) [],
    /// Clear Output Data
    P27 OFFSET(27) NUMBITS(1) [],
    /// Clear Output Data
    P28 OFFSET(28) NUMBITS(1) [],
    /// Clear Output Data
    P29 OFFSET(29) NUMBITS(1) [],
    /// Clear Output Data
    P30 OFFSET(30) NUMBITS(1) [],
    /// Clear Output Data
    P31 OFFSET(31) NUMBITS(1) []
],
ODSR [
    /// Output Data Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Data Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Data Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Data Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Data Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Data Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Data Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Data Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Data Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Data Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Data Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Data Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Data Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Data Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Data Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Data Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Data Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Data Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Data Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Data Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Data Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Data Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Data Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Data Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Data Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Data Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Data Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Data Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Data Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Data Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Data Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Data Status
    P31 OFFSET(31) NUMBITS(1) []
],
PDSR [
    /// Output Data Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Data Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Data Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Data Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Data Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Data Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Data Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Data Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Data Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Data Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Data Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Data Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Data Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Data Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Data Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Data Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Data Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Data Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Data Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Data Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Data Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Data Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Data Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Data Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Data Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Data Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Data Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Data Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Data Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Data Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Data Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Data Status
    P31 OFFSET(31) NUMBITS(1) []
],
IER [
    /// Input Change Interrupt Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Change Interrupt Enable
    P31 OFFSET(31) NUMBITS(1) []
],
IDR [
    /// Input Change Interrupt Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Change Interrupt Disable
    P31 OFFSET(31) NUMBITS(1) []
],
IMR [
    /// Input Change Interrupt Mask
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Change Interrupt Mask
    P31 OFFSET(31) NUMBITS(1) []
],
ISR [
    /// Input Change Interrupt Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Input Change Interrupt Status
    P31 OFFSET(31) NUMBITS(1) []
],
MDER [
    /// Multi-drive Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Multi-drive Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Multi-drive Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Multi-drive Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Multi-drive Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Multi-drive Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Multi-drive Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Multi-drive Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Multi-drive Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Multi-drive Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Multi-drive Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Multi-drive Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Multi-drive Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Multi-drive Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Multi-drive Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Multi-drive Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Multi-drive Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Multi-drive Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Multi-drive Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Multi-drive Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Multi-drive Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Multi-drive Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Multi-drive Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Multi-drive Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Multi-drive Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Multi-drive Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Multi-drive Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Multi-drive Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Multi-drive Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Multi-drive Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Multi-drive Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Multi-drive Enable
    P31 OFFSET(31) NUMBITS(1) []
],
MDDR [
    /// Multi-drive Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Multi-drive Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Multi-drive Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Multi-drive Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Multi-drive Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Multi-drive Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Multi-drive Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Multi-drive Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Multi-drive Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Multi-drive Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Multi-drive Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Multi-drive Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Multi-drive Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Multi-drive Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Multi-drive Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Multi-drive Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Multi-drive Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Multi-drive Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Multi-drive Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Multi-drive Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Multi-drive Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Multi-drive Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Multi-drive Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Multi-drive Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Multi-drive Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Multi-drive Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Multi-drive Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Multi-drive Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Multi-drive Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Multi-drive Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Multi-drive Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Multi-drive Disable
    P31 OFFSET(31) NUMBITS(1) []
],
MDSR [
    /// Multi-drive Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Multi-drive Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Multi-drive Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Multi-drive Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Multi-drive Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Multi-drive Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Multi-drive Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Multi-drive Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Multi-drive Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Multi-drive Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Multi-drive Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Multi-drive Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Multi-drive Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Multi-drive Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Multi-drive Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Multi-drive Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Multi-drive Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Multi-drive Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Multi-drive Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Multi-drive Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Multi-drive Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Multi-drive Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Multi-drive Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Multi-drive Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Multi-drive Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Multi-drive Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Multi-drive Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Multi-drive Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Multi-drive Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Multi-drive Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Multi-drive Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Multi-drive Status
    P31 OFFSET(31) NUMBITS(1) []
],
PUDR [
    /// Pull-Up Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Pull-Up Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Pull-Up Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Pull-Up Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Pull-Up Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Pull-Up Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Pull-Up Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Pull-Up Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Pull-Up Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Pull-Up Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Pull-Up Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Pull-Up Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Pull-Up Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Pull-Up Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Pull-Up Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Pull-Up Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Pull-Up Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Pull-Up Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Pull-Up Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Pull-Up Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Pull-Up Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Pull-Up Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Pull-Up Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Pull-Up Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Pull-Up Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Pull-Up Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Pull-Up Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Pull-Up Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Pull-Up Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Pull-Up Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Pull-Up Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Pull-Up Disable
    P31 OFFSET(31) NUMBITS(1) []
],
PUER [
    /// Pull-Up Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Pull-Up Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Pull-Up Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Pull-Up Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Pull-Up Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Pull-Up Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Pull-Up Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Pull-Up Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Pull-Up Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Pull-Up Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Pull-Up Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Pull-Up Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Pull-Up Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Pull-Up Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Pull-Up Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Pull-Up Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Pull-Up Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Pull-Up Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Pull-Up Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Pull-Up Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Pull-Up Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Pull-Up Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Pull-Up Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Pull-Up Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Pull-Up Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Pull-Up Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Pull-Up Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Pull-Up Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Pull-Up Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Pull-Up Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Pull-Up Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Pull-Up Enable
    P31 OFFSET(31) NUMBITS(1) []
],
PUSR [
    /// Pull-Up Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Pull-Up Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Pull-Up Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Pull-Up Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Pull-Up Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Pull-Up Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Pull-Up Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Pull-Up Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Pull-Up Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Pull-Up Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Pull-Up Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Pull-Up Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Pull-Up Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Pull-Up Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Pull-Up Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Pull-Up Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Pull-Up Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Pull-Up Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Pull-Up Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Pull-Up Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Pull-Up Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Pull-Up Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Pull-Up Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Pull-Up Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Pull-Up Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Pull-Up Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Pull-Up Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Pull-Up Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Pull-Up Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Pull-Up Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Pull-Up Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Pull-Up Status
    P31 OFFSET(31) NUMBITS(1) []
],
IFSCDR [
    /// Peripheral Clock Glitch Filtering Select
    P0 OFFSET(0) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P1 OFFSET(1) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P2 OFFSET(2) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P3 OFFSET(3) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P4 OFFSET(4) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P5 OFFSET(5) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P6 OFFSET(6) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P7 OFFSET(7) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P8 OFFSET(8) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P9 OFFSET(9) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P10 OFFSET(10) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P11 OFFSET(11) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P12 OFFSET(12) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P13 OFFSET(13) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P14 OFFSET(14) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P15 OFFSET(15) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P16 OFFSET(16) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P17 OFFSET(17) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P18 OFFSET(18) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P19 OFFSET(19) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P20 OFFSET(20) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P21 OFFSET(21) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P22 OFFSET(22) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P23 OFFSET(23) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P24 OFFSET(24) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P25 OFFSET(25) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P26 OFFSET(26) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P27 OFFSET(27) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P28 OFFSET(28) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P29 OFFSET(29) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P30 OFFSET(30) NUMBITS(1) [],
    /// Peripheral Clock Glitch Filtering Select
    P31 OFFSET(31) NUMBITS(1) []
],
IFSCER [
    /// Slow Clock Debouncing Filtering Select
    P0 OFFSET(0) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P1 OFFSET(1) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P2 OFFSET(2) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P3 OFFSET(3) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P4 OFFSET(4) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P5 OFFSET(5) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P6 OFFSET(6) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P7 OFFSET(7) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P8 OFFSET(8) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P9 OFFSET(9) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P10 OFFSET(10) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P11 OFFSET(11) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P12 OFFSET(12) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P13 OFFSET(13) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P14 OFFSET(14) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P15 OFFSET(15) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P16 OFFSET(16) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P17 OFFSET(17) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P18 OFFSET(18) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P19 OFFSET(19) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P20 OFFSET(20) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P21 OFFSET(21) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P22 OFFSET(22) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P23 OFFSET(23) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P24 OFFSET(24) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P25 OFFSET(25) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P26 OFFSET(26) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P27 OFFSET(27) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P28 OFFSET(28) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P29 OFFSET(29) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P30 OFFSET(30) NUMBITS(1) [],
    /// Slow Clock Debouncing Filtering Select
    P31 OFFSET(31) NUMBITS(1) []
],
IFSCSR [
    /// Glitch or Debouncing Filter Selection Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Glitch or Debouncing Filter Selection Status
    P31 OFFSET(31) NUMBITS(1) []
],
SCDR [
    /// Slow Clock Divider Selection for Debouncing
    DIV OFFSET(0) NUMBITS(14) []
],
PPDDR [
    /// Pull-Down Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Pull-Down Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Pull-Down Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Pull-Down Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Pull-Down Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Pull-Down Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Pull-Down Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Pull-Down Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Pull-Down Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Pull-Down Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Pull-Down Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Pull-Down Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Pull-Down Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Pull-Down Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Pull-Down Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Pull-Down Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Pull-Down Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Pull-Down Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Pull-Down Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Pull-Down Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Pull-Down Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Pull-Down Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Pull-Down Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Pull-Down Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Pull-Down Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Pull-Down Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Pull-Down Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Pull-Down Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Pull-Down Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Pull-Down Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Pull-Down Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Pull-Down Disable
    P31 OFFSET(31) NUMBITS(1) []
],
PPDER [
    /// Pull-Down Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Pull-Down Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Pull-Down Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Pull-Down Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Pull-Down Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Pull-Down Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Pull-Down Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Pull-Down Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Pull-Down Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Pull-Down Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Pull-Down Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Pull-Down Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Pull-Down Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Pull-Down Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Pull-Down Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Pull-Down Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Pull-Down Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Pull-Down Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Pull-Down Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Pull-Down Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Pull-Down Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Pull-Down Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Pull-Down Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Pull-Down Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Pull-Down Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Pull-Down Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Pull-Down Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Pull-Down Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Pull-Down Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Pull-Down Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Pull-Down Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Pull-Down Enable
    P31 OFFSET(31) NUMBITS(1) []
],
PPDSR [
    /// Pull-Down Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Pull-Down Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Pull-Down Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Pull-Down Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Pull-Down Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Pull-Down Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Pull-Down Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Pull-Down Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Pull-Down Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Pull-Down Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Pull-Down Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Pull-Down Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Pull-Down Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Pull-Down Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Pull-Down Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Pull-Down Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Pull-Down Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Pull-Down Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Pull-Down Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Pull-Down Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Pull-Down Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Pull-Down Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Pull-Down Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Pull-Down Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Pull-Down Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Pull-Down Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Pull-Down Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Pull-Down Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Pull-Down Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Pull-Down Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Pull-Down Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Pull-Down Status
    P31 OFFSET(31) NUMBITS(1) []
],
OWER [
    /// Output Write Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Write Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Write Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Write Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Write Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Write Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Write Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Write Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Write Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Write Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Write Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Write Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Write Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Write Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Write Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Write Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Write Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Write Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Write Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Write Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Write Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Write Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Write Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Write Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Write Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Write Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Write Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Write Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Write Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Write Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Write Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Write Enable
    P31 OFFSET(31) NUMBITS(1) []
],
OWDR [
    /// Output Write Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Write Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Write Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Write Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Write Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Write Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Write Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Write Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Write Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Write Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Write Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Write Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Write Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Write Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Write Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Write Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Write Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Write Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Write Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Write Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Write Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Write Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Write Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Write Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Write Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Write Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Write Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Write Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Write Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Write Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Write Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Write Disable
    P31 OFFSET(31) NUMBITS(1) []
],
OWSR [
    /// Output Write Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Output Write Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Output Write Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Output Write Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Output Write Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Output Write Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Output Write Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Output Write Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Output Write Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Output Write Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Output Write Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Output Write Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Output Write Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Output Write Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Output Write Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Output Write Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Output Write Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Output Write Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Output Write Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Output Write Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Output Write Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Output Write Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Output Write Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Output Write Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Output Write Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Output Write Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Output Write Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Output Write Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Output Write Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Output Write Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Output Write Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Output Write Status
    P31 OFFSET(31) NUMBITS(1) []
],
AIMER [
    /// Additional Interrupt Modes Enable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Additional Interrupt Modes Enable
    P31 OFFSET(31) NUMBITS(1) []
],
AIMDR [
    /// Additional Interrupt Modes Disable
    P0 OFFSET(0) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P1 OFFSET(1) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P2 OFFSET(2) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P3 OFFSET(3) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P4 OFFSET(4) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P5 OFFSET(5) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P6 OFFSET(6) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P7 OFFSET(7) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P8 OFFSET(8) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P9 OFFSET(9) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P10 OFFSET(10) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P11 OFFSET(11) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P12 OFFSET(12) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P13 OFFSET(13) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P14 OFFSET(14) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P15 OFFSET(15) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P16 OFFSET(16) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P17 OFFSET(17) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P18 OFFSET(18) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P19 OFFSET(19) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P20 OFFSET(20) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P21 OFFSET(21) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P22 OFFSET(22) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P23 OFFSET(23) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P24 OFFSET(24) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P25 OFFSET(25) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P26 OFFSET(26) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P27 OFFSET(27) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P28 OFFSET(28) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P29 OFFSET(29) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P30 OFFSET(30) NUMBITS(1) [],
    /// Additional Interrupt Modes Disable
    P31 OFFSET(31) NUMBITS(1) []
],
AIMMR [
    /// IO Line Index
    P0 OFFSET(0) NUMBITS(1) [],
    /// IO Line Index
    P1 OFFSET(1) NUMBITS(1) [],
    /// IO Line Index
    P2 OFFSET(2) NUMBITS(1) [],
    /// IO Line Index
    P3 OFFSET(3) NUMBITS(1) [],
    /// IO Line Index
    P4 OFFSET(4) NUMBITS(1) [],
    /// IO Line Index
    P5 OFFSET(5) NUMBITS(1) [],
    /// IO Line Index
    P6 OFFSET(6) NUMBITS(1) [],
    /// IO Line Index
    P7 OFFSET(7) NUMBITS(1) [],
    /// IO Line Index
    P8 OFFSET(8) NUMBITS(1) [],
    /// IO Line Index
    P9 OFFSET(9) NUMBITS(1) [],
    /// IO Line Index
    P10 OFFSET(10) NUMBITS(1) [],
    /// IO Line Index
    P11 OFFSET(11) NUMBITS(1) [],
    /// IO Line Index
    P12 OFFSET(12) NUMBITS(1) [],
    /// IO Line Index
    P13 OFFSET(13) NUMBITS(1) [],
    /// IO Line Index
    P14 OFFSET(14) NUMBITS(1) [],
    /// IO Line Index
    P15 OFFSET(15) NUMBITS(1) [],
    /// IO Line Index
    P16 OFFSET(16) NUMBITS(1) [],
    /// IO Line Index
    P17 OFFSET(17) NUMBITS(1) [],
    /// IO Line Index
    P18 OFFSET(18) NUMBITS(1) [],
    /// IO Line Index
    P19 OFFSET(19) NUMBITS(1) [],
    /// IO Line Index
    P20 OFFSET(20) NUMBITS(1) [],
    /// IO Line Index
    P21 OFFSET(21) NUMBITS(1) [],
    /// IO Line Index
    P22 OFFSET(22) NUMBITS(1) [],
    /// IO Line Index
    P23 OFFSET(23) NUMBITS(1) [],
    /// IO Line Index
    P24 OFFSET(24) NUMBITS(1) [],
    /// IO Line Index
    P25 OFFSET(25) NUMBITS(1) [],
    /// IO Line Index
    P26 OFFSET(26) NUMBITS(1) [],
    /// IO Line Index
    P27 OFFSET(27) NUMBITS(1) [],
    /// IO Line Index
    P28 OFFSET(28) NUMBITS(1) [],
    /// IO Line Index
    P29 OFFSET(29) NUMBITS(1) [],
    /// IO Line Index
    P30 OFFSET(30) NUMBITS(1) [],
    /// IO Line Index
    P31 OFFSET(31) NUMBITS(1) []
],
ESR [
    /// Edge Interrupt Selection
    P0 OFFSET(0) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P1 OFFSET(1) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P2 OFFSET(2) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P3 OFFSET(3) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P4 OFFSET(4) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P5 OFFSET(5) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P6 OFFSET(6) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P7 OFFSET(7) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P8 OFFSET(8) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P9 OFFSET(9) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P10 OFFSET(10) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P11 OFFSET(11) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P12 OFFSET(12) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P13 OFFSET(13) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P14 OFFSET(14) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P15 OFFSET(15) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P16 OFFSET(16) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P17 OFFSET(17) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P18 OFFSET(18) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P19 OFFSET(19) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P20 OFFSET(20) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P21 OFFSET(21) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P22 OFFSET(22) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P23 OFFSET(23) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P24 OFFSET(24) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P25 OFFSET(25) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P26 OFFSET(26) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P27 OFFSET(27) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P28 OFFSET(28) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P29 OFFSET(29) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P30 OFFSET(30) NUMBITS(1) [],
    /// Edge Interrupt Selection
    P31 OFFSET(31) NUMBITS(1) []
],
LSR [
    /// Level Interrupt Selection
    P0 OFFSET(0) NUMBITS(1) [],
    /// Level Interrupt Selection
    P1 OFFSET(1) NUMBITS(1) [],
    /// Level Interrupt Selection
    P2 OFFSET(2) NUMBITS(1) [],
    /// Level Interrupt Selection
    P3 OFFSET(3) NUMBITS(1) [],
    /// Level Interrupt Selection
    P4 OFFSET(4) NUMBITS(1) [],
    /// Level Interrupt Selection
    P5 OFFSET(5) NUMBITS(1) [],
    /// Level Interrupt Selection
    P6 OFFSET(6) NUMBITS(1) [],
    /// Level Interrupt Selection
    P7 OFFSET(7) NUMBITS(1) [],
    /// Level Interrupt Selection
    P8 OFFSET(8) NUMBITS(1) [],
    /// Level Interrupt Selection
    P9 OFFSET(9) NUMBITS(1) [],
    /// Level Interrupt Selection
    P10 OFFSET(10) NUMBITS(1) [],
    /// Level Interrupt Selection
    P11 OFFSET(11) NUMBITS(1) [],
    /// Level Interrupt Selection
    P12 OFFSET(12) NUMBITS(1) [],
    /// Level Interrupt Selection
    P13 OFFSET(13) NUMBITS(1) [],
    /// Level Interrupt Selection
    P14 OFFSET(14) NUMBITS(1) [],
    /// Level Interrupt Selection
    P15 OFFSET(15) NUMBITS(1) [],
    /// Level Interrupt Selection
    P16 OFFSET(16) NUMBITS(1) [],
    /// Level Interrupt Selection
    P17 OFFSET(17) NUMBITS(1) [],
    /// Level Interrupt Selection
    P18 OFFSET(18) NUMBITS(1) [],
    /// Level Interrupt Selection
    P19 OFFSET(19) NUMBITS(1) [],
    /// Level Interrupt Selection
    P20 OFFSET(20) NUMBITS(1) [],
    /// Level Interrupt Selection
    P21 OFFSET(21) NUMBITS(1) [],
    /// Level Interrupt Selection
    P22 OFFSET(22) NUMBITS(1) [],
    /// Level Interrupt Selection
    P23 OFFSET(23) NUMBITS(1) [],
    /// Level Interrupt Selection
    P24 OFFSET(24) NUMBITS(1) [],
    /// Level Interrupt Selection
    P25 OFFSET(25) NUMBITS(1) [],
    /// Level Interrupt Selection
    P26 OFFSET(26) NUMBITS(1) [],
    /// Level Interrupt Selection
    P27 OFFSET(27) NUMBITS(1) [],
    /// Level Interrupt Selection
    P28 OFFSET(28) NUMBITS(1) [],
    /// Level Interrupt Selection
    P29 OFFSET(29) NUMBITS(1) [],
    /// Level Interrupt Selection
    P30 OFFSET(30) NUMBITS(1) [],
    /// Level Interrupt Selection
    P31 OFFSET(31) NUMBITS(1) []
],
ELSR [
    /// Edge/Level Interrupt Source Selection
    P0 OFFSET(0) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P1 OFFSET(1) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P2 OFFSET(2) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P3 OFFSET(3) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P4 OFFSET(4) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P5 OFFSET(5) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P6 OFFSET(6) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P7 OFFSET(7) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P8 OFFSET(8) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P9 OFFSET(9) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P10 OFFSET(10) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P11 OFFSET(11) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P12 OFFSET(12) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P13 OFFSET(13) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P14 OFFSET(14) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P15 OFFSET(15) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P16 OFFSET(16) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P17 OFFSET(17) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P18 OFFSET(18) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P19 OFFSET(19) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P20 OFFSET(20) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P21 OFFSET(21) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P22 OFFSET(22) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P23 OFFSET(23) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P24 OFFSET(24) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P25 OFFSET(25) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P26 OFFSET(26) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P27 OFFSET(27) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P28 OFFSET(28) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P29 OFFSET(29) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P30 OFFSET(30) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P31 OFFSET(31) NUMBITS(1) []
],
FELLSR [
    /// Falling Edge/Low-Level Interrupt Selection
    P0 OFFSET(0) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P1 OFFSET(1) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P2 OFFSET(2) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P3 OFFSET(3) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P4 OFFSET(4) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P5 OFFSET(5) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P6 OFFSET(6) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P7 OFFSET(7) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P8 OFFSET(8) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P9 OFFSET(9) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P10 OFFSET(10) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P11 OFFSET(11) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P12 OFFSET(12) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P13 OFFSET(13) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P14 OFFSET(14) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P15 OFFSET(15) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P16 OFFSET(16) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P17 OFFSET(17) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P18 OFFSET(18) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P19 OFFSET(19) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P20 OFFSET(20) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P21 OFFSET(21) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P22 OFFSET(22) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P23 OFFSET(23) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P24 OFFSET(24) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P25 OFFSET(25) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P26 OFFSET(26) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P27 OFFSET(27) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P28 OFFSET(28) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P29 OFFSET(29) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P30 OFFSET(30) NUMBITS(1) [],
    /// Falling Edge/Low-Level Interrupt Selection
    P31 OFFSET(31) NUMBITS(1) []
],
REHLSR [
    /// Rising Edge/High-Level Interrupt Selection
    P0 OFFSET(0) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P1 OFFSET(1) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P2 OFFSET(2) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P3 OFFSET(3) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P4 OFFSET(4) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P5 OFFSET(5) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P6 OFFSET(6) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P7 OFFSET(7) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P8 OFFSET(8) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P9 OFFSET(9) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P10 OFFSET(10) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P11 OFFSET(11) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P12 OFFSET(12) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P13 OFFSET(13) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P14 OFFSET(14) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P15 OFFSET(15) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P16 OFFSET(16) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P17 OFFSET(17) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P18 OFFSET(18) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P19 OFFSET(19) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P20 OFFSET(20) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P21 OFFSET(21) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P22 OFFSET(22) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P23 OFFSET(23) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P24 OFFSET(24) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P25 OFFSET(25) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P26 OFFSET(26) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P27 OFFSET(27) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P28 OFFSET(28) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P29 OFFSET(29) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P30 OFFSET(30) NUMBITS(1) [],
    /// Rising Edge/High-Level Interrupt Selection
    P31 OFFSET(31) NUMBITS(1) []
],
FRLHSR [
    /// Edge/Level Interrupt Source Selection
    P0 OFFSET(0) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P1 OFFSET(1) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P2 OFFSET(2) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P3 OFFSET(3) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P4 OFFSET(4) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P5 OFFSET(5) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P6 OFFSET(6) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P7 OFFSET(7) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P8 OFFSET(8) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P9 OFFSET(9) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P10 OFFSET(10) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P11 OFFSET(11) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P12 OFFSET(12) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P13 OFFSET(13) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P14 OFFSET(14) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P15 OFFSET(15) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P16 OFFSET(16) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P17 OFFSET(17) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P18 OFFSET(18) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P19 OFFSET(19) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P20 OFFSET(20) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P21 OFFSET(21) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P22 OFFSET(22) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P23 OFFSET(23) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P24 OFFSET(24) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P25 OFFSET(25) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P26 OFFSET(26) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P27 OFFSET(27) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P28 OFFSET(28) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P29 OFFSET(29) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P30 OFFSET(30) NUMBITS(1) [],
    /// Edge/Level Interrupt Source Selection
    P31 OFFSET(31) NUMBITS(1) []
],
LOCKSR [
    /// Lock Status
    P0 OFFSET(0) NUMBITS(1) [],
    /// Lock Status
    P1 OFFSET(1) NUMBITS(1) [],
    /// Lock Status
    P2 OFFSET(2) NUMBITS(1) [],
    /// Lock Status
    P3 OFFSET(3) NUMBITS(1) [],
    /// Lock Status
    P4 OFFSET(4) NUMBITS(1) [],
    /// Lock Status
    P5 OFFSET(5) NUMBITS(1) [],
    /// Lock Status
    P6 OFFSET(6) NUMBITS(1) [],
    /// Lock Status
    P7 OFFSET(7) NUMBITS(1) [],
    /// Lock Status
    P8 OFFSET(8) NUMBITS(1) [],
    /// Lock Status
    P9 OFFSET(9) NUMBITS(1) [],
    /// Lock Status
    P10 OFFSET(10) NUMBITS(1) [],
    /// Lock Status
    P11 OFFSET(11) NUMBITS(1) [],
    /// Lock Status
    P12 OFFSET(12) NUMBITS(1) [],
    /// Lock Status
    P13 OFFSET(13) NUMBITS(1) [],
    /// Lock Status
    P14 OFFSET(14) NUMBITS(1) [],
    /// Lock Status
    P15 OFFSET(15) NUMBITS(1) [],
    /// Lock Status
    P16 OFFSET(16) NUMBITS(1) [],
    /// Lock Status
    P17 OFFSET(17) NUMBITS(1) [],
    /// Lock Status
    P18 OFFSET(18) NUMBITS(1) [],
    /// Lock Status
    P19 OFFSET(19) NUMBITS(1) [],
    /// Lock Status
    P20 OFFSET(20) NUMBITS(1) [],
    /// Lock Status
    P21 OFFSET(21) NUMBITS(1) [],
    /// Lock Status
    P22 OFFSET(22) NUMBITS(1) [],
    /// Lock Status
    P23 OFFSET(23) NUMBITS(1) [],
    /// Lock Status
    P24 OFFSET(24) NUMBITS(1) [],
    /// Lock Status
    P25 OFFSET(25) NUMBITS(1) [],
    /// Lock Status
    P26 OFFSET(26) NUMBITS(1) [],
    /// Lock Status
    P27 OFFSET(27) NUMBITS(1) [],
    /// Lock Status
    P28 OFFSET(28) NUMBITS(1) [],
    /// Lock Status
    P29 OFFSET(29) NUMBITS(1) [],
    /// Lock Status
    P30 OFFSET(30) NUMBITS(1) [],
    /// Lock Status
    P31 OFFSET(31) NUMBITS(1) []
],
WPMR [
    /// Write Protection Enable
    WPEN OFFSET(0) NUMBITS(1) [],
    /// Write Protection Key
    WPKEY OFFSET(8) NUMBITS(24) [
        /// Writing any other value in this field aborts the write operation of the WPEN bit.Always reads as 0.
        PASSWD = 5261647
    ]
],
WPSR [
    /// Write Protection Violation Status
    WPVS OFFSET(0) NUMBITS(1) [],
    /// Write Protection Violation Source
    WPVSRC OFFSET(8) NUMBITS(16) []
],
SCHMITT [
    /// Schmitt Trigger Control
    SCHMITT0 OFFSET(0) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT1 OFFSET(1) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT2 OFFSET(2) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT3 OFFSET(3) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT4 OFFSET(4) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT5 OFFSET(5) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT6 OFFSET(6) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT7 OFFSET(7) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT8 OFFSET(8) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT9 OFFSET(9) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT10 OFFSET(10) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT11 OFFSET(11) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT12 OFFSET(12) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT13 OFFSET(13) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT14 OFFSET(14) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT15 OFFSET(15) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT16 OFFSET(16) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT17 OFFSET(17) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT18 OFFSET(18) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT19 OFFSET(19) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT20 OFFSET(20) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT21 OFFSET(21) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT22 OFFSET(22) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT23 OFFSET(23) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT24 OFFSET(24) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT25 OFFSET(25) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT26 OFFSET(26) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT27 OFFSET(27) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT28 OFFSET(28) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT29 OFFSET(29) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT30 OFFSET(30) NUMBITS(1) [],
    /// Schmitt Trigger Control
    SCHMITT31 OFFSET(31) NUMBITS(1) []
],
DRIVER [
    /// Drive of PIO Line 0
    LINE0 OFFSET(0) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 1
    LINE1 OFFSET(1) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 2
    LINE2 OFFSET(2) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 3
    LINE3 OFFSET(3) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 4
    LINE4 OFFSET(4) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 5
    LINE5 OFFSET(5) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 6
    LINE6 OFFSET(6) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 7
    LINE7 OFFSET(7) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 8
    LINE8 OFFSET(8) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 9
    LINE9 OFFSET(9) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 10
    LINE10 OFFSET(10) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 11
    LINE11 OFFSET(11) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 12
    LINE12 OFFSET(12) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 13
    LINE13 OFFSET(13) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 14
    LINE14 OFFSET(14) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 15
    LINE15 OFFSET(15) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 16
    LINE16 OFFSET(16) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 17
    LINE17 OFFSET(17) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 18
    LINE18 OFFSET(18) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 19
    LINE19 OFFSET(19) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 20
    LINE20 OFFSET(20) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 21
    LINE21 OFFSET(21) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 22
    LINE22 OFFSET(22) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 23
    LINE23 OFFSET(23) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 24
    LINE24 OFFSET(24) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 25
    LINE25 OFFSET(25) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 26
    LINE26 OFFSET(26) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 27
    LINE27 OFFSET(27) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 28
    LINE28 OFFSET(28) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 29
    LINE29 OFFSET(29) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 30
    LINE30 OFFSET(30) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ],
    /// Drive of PIO Line 31
    LINE31 OFFSET(31) NUMBITS(1) [
        /// Lowest drive
        LowestDrive = 0,
        /// Highest drive
        HighestDrive = 1
    ]
],
PCMR [
    /// Parallel Capture Mode Enable
    PCEN OFFSET(0) NUMBITS(1) [],
    /// Parallel Capture Mode Data Size
    DSIZE OFFSET(4) NUMBITS(2) [
        /// The reception data in the PIO_PCRHR is a byte (8-bit)
        TheReceptionDataInThePIO_PCRHRIsAByte8Bit = 0,
        /// The reception data in the PIO_PCRHR is a half-word (16-bit)
        TheReceptionDataInThePIO_PCRHRIsAHalfWord16Bit = 1,
        /// The reception data in the PIO_PCRHR is a word (32-bit)
        TheReceptionDataInThePIO_PCRHRIsAWord32Bit = 2
    ],
    /// Parallel Capture Mode Always Sampling
    ALWYS OFFSET(9) NUMBITS(1) [],
    /// Parallel Capture Mode Half Sampling
    HALFS OFFSET(10) NUMBITS(1) [],
    /// Parallel Capture Mode First Sample
    FRSTS OFFSET(11) NUMBITS(1) []
],
PCIER [
    /// Parallel Capture Mode Data Ready Interrupt Enable
    DRDY OFFSET(0) NUMBITS(1) [],
    /// Parallel Capture Mode Overrun Error Interrupt Enable
    OVRE OFFSET(1) NUMBITS(1) [],
    /// End of Reception Transfer Interrupt Enable
    ENDRX OFFSET(2) NUMBITS(1) [],
    /// Reception Buffer Full Interrupt Enable
    RXBUFF OFFSET(3) NUMBITS(1) []
],
PCIDR [
    /// Parallel Capture Mode Data Ready Interrupt Disable
    DRDY OFFSET(0) NUMBITS(1) [],
    /// Parallel Capture Mode Overrun Error Interrupt Disable
    OVRE OFFSET(1) NUMBITS(1) [],
    /// End of Reception Transfer Interrupt Disable
    ENDRX OFFSET(2) NUMBITS(1) [],
    /// Reception Buffer Full Interrupt Disable
    RXBUFF OFFSET(3) NUMBITS(1) []
],
PCIMR [
    /// Parallel Capture Mode Data Ready Interrupt Mask
    DRDY OFFSET(0) NUMBITS(1) [],
    /// Parallel Capture Mode Overrun Error Interrupt Mask
    OVRE OFFSET(1) NUMBITS(1) [],
    /// End of Reception Transfer Interrupt Mask
    ENDRX OFFSET(2) NUMBITS(1) [],
    /// Reception Buffer Full Interrupt Mask
    RXBUFF OFFSET(3) NUMBITS(1) []
],
PCISR [
    /// Parallel Capture Mode Data Ready
    DRDY OFFSET(0) NUMBITS(1) [],
    /// Parallel Capture Mode Overrun Error
    OVRE OFFSET(1) NUMBITS(1) []
],
PCRHR [
    /// Parallel Capture Mode Reception Data
    RDATA OFFSET(0) NUMBITS(32) []
],
ABCDSR0 [
    /// Peripheral Select
    P0 OFFSET(0) NUMBITS(1) [],
    /// Peripheral Select
    P1 OFFSET(1) NUMBITS(1) [],
    /// Peripheral Select
    P2 OFFSET(2) NUMBITS(1) [],
    /// Peripheral Select
    P3 OFFSET(3) NUMBITS(1) [],
    /// Peripheral Select
    P4 OFFSET(4) NUMBITS(1) [],
    /// Peripheral Select
    P5 OFFSET(5) NUMBITS(1) [],
    /// Peripheral Select
    P6 OFFSET(6) NUMBITS(1) [],
    /// Peripheral Select
    P7 OFFSET(7) NUMBITS(1) [],
    /// Peripheral Select
    P8 OFFSET(8) NUMBITS(1) [],
    /// Peripheral Select
    P9 OFFSET(9) NUMBITS(1) [],
    /// Peripheral Select
    P10 OFFSET(10) NUMBITS(1) [],
    /// Peripheral Select
    P11 OFFSET(11) NUMBITS(1) [],
    /// Peripheral Select
    P12 OFFSET(12) NUMBITS(1) [],
    /// Peripheral Select
    P13 OFFSET(13) NUMBITS(1) [],
    /// Peripheral Select
    P14 OFFSET(14) NUMBITS(1) [],
    /// Peripheral Select
    P15 OFFSET(15) NUMBITS(1) [],
    /// Peripheral Select
    P16 OFFSET(16) NUMBITS(1) [],
    /// Peripheral Select
    P17 OFFSET(17) NUMBITS(1) [],
    /// Peripheral Select
    P18 OFFSET(18) NUMBITS(1) [],
    /// Peripheral Select
    P19 OFFSET(19) NUMBITS(1) [],
    /// Peripheral Select
    P20 OFFSET(20) NUMBITS(1) [],
    /// Peripheral Select
    P21 OFFSET(21) NUMBITS(1) [],
    /// Peripheral Select
    P22 OFFSET(22) NUMBITS(1) [],
    /// Peripheral Select
    P23 OFFSET(23) NUMBITS(1) [],
    /// Peripheral Select
    P24 OFFSET(24) NUMBITS(1) [],
    /// Peripheral Select
    P25 OFFSET(25) NUMBITS(1) [],
    /// Peripheral Select
    P26 OFFSET(26) NUMBITS(1) [],
    /// Peripheral Select
    P27 OFFSET(27) NUMBITS(1) [],
    /// Peripheral Select
    P28 OFFSET(28) NUMBITS(1) [],
    /// Peripheral Select
    P29 OFFSET(29) NUMBITS(1) [],
    /// Peripheral Select
    P30 OFFSET(30) NUMBITS(1) [],
    /// Peripheral Select
    P31 OFFSET(31) NUMBITS(1) []
],
ABCDSR1 [
    /// Peripheral Select
    P0 OFFSET(0) NUMBITS(1) [],
    /// Peripheral Select
    P1 OFFSET(1) NUMBITS(1) [],
    /// Peripheral Select
    P2 OFFSET(2) NUMBITS(1) [],
    /// Peripheral Select
    P3 OFFSET(3) NUMBITS(1) [],
    /// Peripheral Select
    P4 OFFSET(4) NUMBITS(1) [],
    /// Peripheral Select
    P5 OFFSET(5) NUMBITS(1) [],
    /// Peripheral Select
    P6 OFFSET(6) NUMBITS(1) [],
    /// Peripheral Select
    P7 OFFSET(7) NUMBITS(1) [],
    /// Peripheral Select
    P8 OFFSET(8) NUMBITS(1) [],
    /// Peripheral Select
    P9 OFFSET(9) NUMBITS(1) [],
    /// Peripheral Select
    P10 OFFSET(10) NUMBITS(1) [],
    /// Peripheral Select
    P11 OFFSET(11) NUMBITS(1) [],
    /// Peripheral Select
    P12 OFFSET(12) NUMBITS(1) [],
    /// Peripheral Select
    P13 OFFSET(13) NUMBITS(1) [],
    /// Peripheral Select
    P14 OFFSET(14) NUMBITS(1) [],
    /// Peripheral Select
    P15 OFFSET(15) NUMBITS(1) [],
    /// Peripheral Select
    P16 OFFSET(16) NUMBITS(1) [],
    /// Peripheral Select
    P17 OFFSET(17) NUMBITS(1) [],
    /// Peripheral Select
    P18 OFFSET(18) NUMBITS(1) [],
    /// Peripheral Select
    P19 OFFSET(19) NUMBITS(1) [],
    /// Peripheral Select
    P20 OFFSET(20) NUMBITS(1) [],
    /// Peripheral Select
    P21 OFFSET(21) NUMBITS(1) [],
    /// Peripheral Select
    P22 OFFSET(22) NUMBITS(1) [],
    /// Peripheral Select
    P23 OFFSET(23) NUMBITS(1) [],
    /// Peripheral Select
    P24 OFFSET(24) NUMBITS(1) [],
    /// Peripheral Select
    P25 OFFSET(25) NUMBITS(1) [],
    /// Peripheral Select
    P26 OFFSET(26) NUMBITS(1) [],
    /// Peripheral Select
    P27 OFFSET(27) NUMBITS(1) [],
    /// Peripheral Select
    P28 OFFSET(28) NUMBITS(1) [],
    /// Peripheral Select
    P29 OFFSET(29) NUMBITS(1) [],
    /// Peripheral Select
    P30 OFFSET(30) NUMBITS(1) [],
    /// Peripheral Select
    P31 OFFSET(31) NUMBITS(1) []
]
];

/// Peripheral functions that may be assigned to a `GPIOPin`.
///
/// GPIO pins on the SAMv71 may serve multiple functions. In addition to the
/// default functionality, each pin can be assigned up to four different
/// peripheral functions. The various functions for each pin are described in
/// "Peripheral Signal Multiplexing on I/O Lines" section of the SAMV71 datasheet[^1].
///
/// [^1]: Section 14.2, pages 68-69
#[derive(Copy, Clone)]
pub enum PeripheralFunction {
    A,
    B,
    C,
    D,
}

/// Reference count for the number of GPIO interrupts currently active.
///
/// This is used to determine if it's possible for the SAM4L to go into
/// WAIT/RETENTION mode, since those modes will not be woken up by GPIO
/// interrupts.
///
/// This is an `AtomicUsize` because it has to be a `Sync` type to live in a
/// global---Rust has no way of knowing we're not going to use it across
/// threads. Use `Ordering::Relaxed` when reading/writing the value to get LLVM
/// to just use plain loads and stores instead of atomic operations.
pub static INTERRUPT_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Name of the GPIO pin on the SAMV71.
///
/// The "Package and Pinout" section[^1] of the SAMV71 datasheet shows the
/// mapping between these names and hardware pins on different chip packages.
///
/// [^1]: Section 6.2, pages 28-35
#[derive(Copy,Clone)]
#[rustfmt::skip]
pub enum Pin {
    PA00, PA01, PA02, PA03, PA04, PA05, PA06, PA07,
    PA08, PA09, PA10, PA11, PA12, PA13, PA14, PA15,
    PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23,
    PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31,
 
    PB00, PB01, PB02, PB03, PB04, PB05, PB06, PB07,
    PB08, PB09, PB12, PB13, 

    PC00, PC01, PC02, PC03, PC04, PC05, PC06, PC07,
    PC08, PC09, PC10, PC11, PC12, PC13, PC14, PC15,
    PC16, PC17, PC18, PC19, PC20, PC21, PC22, PC23,
    PC24, PC25, PC26, PC27, PC28, PC29, PC30, PC31,

    PD00, PD01, PD02, PD03, PD04, PD05, PD06, PD07,
    PD08, PD09, PD10, PD11, PD12, PD13, PD14, PD15,
    PD16, PD17, PD18, PD19, PD20, PD21, PD22, PD23,
    PD24, PD25, PD26, PD27, PD28, PD29, PD30, PD31,

    PE00, PE01, PE02, PE03, PE04, PE05,
}

/// GPIO port that manages a variable number of pins to support the B
/// and E ports which have less than the standard 32 pins.
///
/// The SAMV71 divides GPIOs into _ports_ that each manage a group of 32
/// individual pins. There are up to five ports, depending particular chip
/// (see[^1]).
///
/// In general, the kernel and applications should care about individual
/// [GPIOPin](struct.GPIOPin.html)s. However, mirroring the hardware grouping in
/// Rust is useful, internally, for correctly handling and dispatching
/// interrupts.
///
/// The port itself is a set of 32-bit memory-mapped I/O registers. Each
/// register has a bit for each pin in the port. Pins are, thus, named by their
/// port and offset bit in each register that controls is. For example, the
/// first port has pins called "PA00" thru "PA31".
///
pub struct Port<const N: usize> {
    registers: StaticRef<GpioRegisters>,
    pins: [Pin; N],
}

pub type PortA = Port<32>;
pub type PortB = Port<12>;
pub type PortC = Port<32>;
pub type PortD = Port<32>;
pub type PortE = Port<6>;

impl PortA {
    pub const fn new() -> Self {
        Self {
            registers: unsafe { StaticRef::new(BASE_ADDRESS as *const GpioRegisters) },
            pins: [
                Pin::PA00,
                Pin::PA01,
                Pin::PA02,
                Pin::PA03,
                Pin::PA04,
                Pin::PA05,
                Pin::PA06,
                Pin::PA07,
                Pin::PA08,
                Pin::PA09,
                Pin::PA10,
                Pin::PA11,
                Pin::PA12,
                Pin::PA13,
                Pin::PA14,
                Pin::PA15,
                Pin::PA16,
                Pin::PA17,
                Pin::PA18,
                Pin::PA19,
                Pin::PA20,
                Pin::PA21,
                Pin::PA22,
                Pin::PA23,
                Pin::PA24,
                Pin::PA25,
                Pin::PA26,
                Pin::PA27,
                Pin::PA28,
                Pin::PA29,
                Pin::PA30,
                Pin::PA31,
            ],
        }
    }
}

impl PortB {
    pub const fn new() -> Self {
        Self {
            registers: unsafe { StaticRef::new((BASE_ADDRESS + 1 * SIZE) as *const GpioRegisters) },
            pins: [
                Pin::PB00,
                Pin::PB01,
                Pin::PB02,
                Pin::PB03,
                Pin::PB04,
                Pin::PB05,
                Pin::PB06,
                Pin::PB07,
                Pin::PB08,
                Pin::PB09,
                Pin::PB12,
                Pin::PB13,
            ],
        }
    }
}

impl PortC {
    pub const fn new() -> Self {
        Self {
            registers: unsafe { StaticRef::new((BASE_ADDRESS + 2 * SIZE) as *const GpioRegisters) },
            pins: [
                Pin::PC00,
                Pin::PC01,
                Pin::PC02,
                Pin::PC03,
                Pin::PC04,
                Pin::PC05,
                Pin::PC06,
                Pin::PC07,
                Pin::PC08,
                Pin::PC09,
                Pin::PC10,
                Pin::PC11,
                Pin::PC12,
                Pin::PC13,
                Pin::PC14,
                Pin::PC15,
                Pin::PC16,
                Pin::PC17,
                Pin::PC18,
                Pin::PC19,
                Pin::PC20,
                Pin::PC21,
                Pin::PC22,
                Pin::PC23,
                Pin::PC24,
                Pin::PC25,
                Pin::PC26,
                Pin::PC27,
                Pin::PC28,
                Pin::PC29,
                Pin::PC30,
                Pin::PC31,
            ],
        }
    }
}

impl PortD {
    pub const fn new() -> Self {
        Self {
            registers: unsafe { StaticRef::new((BASE_ADDRESS + 3 * SIZE) as *const GpioRegisters) },
            pins: [
                Pin::PD00,
                Pin::PD01,
                Pin::PD02,
                Pin::PD03,
                Pin::PD04,
                Pin::PD05,
                Pin::PD06,
                Pin::PD07,
                Pin::PD08,
                Pin::PD09,
                Pin::PD10,
                Pin::PD11,
                Pin::PD12,
                Pin::PD13,
                Pin::PD14,
                Pin::PD15,
                Pin::PD16,
                Pin::PD17,
                Pin::PD18,
                Pin::PD19,
                Pin::PD20,
                Pin::PD21,
                Pin::PD22,
                Pin::PD23,
                Pin::PD24,
                Pin::PD25,
                Pin::PD26,
                Pin::PD27,
                Pin::PD28,
                Pin::PD29,
                Pin::PD30,
                Pin::PD31,
            ],
        }
    }
}

impl PortE {
    pub const fn new() -> Self {
        Self {
            registers: unsafe { StaticRef::new((BASE_ADDRESS + 4 * SIZE) as *const GpioRegisters) },
            pins: [
                Pin::PE00,
                Pin::PE01,
                Pin::PE02,
                Pin::PE03,
                Pin::PE04,
                Pin::PE05,
            ],
        }
    }
}

impl<const N: usize> Port<N> {
    pub fn handle_interrupt(&self) {
        let port: &GpioRegisters = &self.registers;

        // Interrupt Flag Register (IFR) bits are only valid if the same bits
        // are enabled in Interrupt Enabled Register (IER).
        let mut fired = port.ifr.val.get() & port.ier.val.get();
        loop {
            let pin = fired.trailing_zeros() as usize;
            if pin < self.pins.len() {
                fired &= !(1 << pin);
                self.pins[pin].handle_interrupt();
                port.ifr.clear.set(1 << pin);
            } else {
                break;
            }
        }
    }
}

pub struct GPIOPin<'a> {
    port: StaticRef<GpioRegisters>,
    pin_mask: u32,
    client: OptionalCell<&'a dyn hil::gpio::Client>,
}

impl<'a> GPIOPin<'a> {
    pub const fn new(pin: Pin) -> GPIOPin<'a> {
        GPIOPin {
            port: unsafe {
                StaticRef::new(
                    (BASE_ADDRESS + ((pin as usize) / 32) * SIZE) as *const GpioRegisters,
                )
            },
            pin_mask: 1 << ((pin as u32) % 32),
            client: OptionalCell::empty(),
        }
    }

    pub fn set_client(&self, client: &'a dyn gpio::Client) {
        self.client.set(client);
    }

    pub fn select_peripheral(&self, function: PeripheralFunction) {
        let f = function as u32;
        let (bit0, bit1, bit2) = (f & 0b1, (f & 0b10) >> 1, (f & 0b100) >> 2);
        let port: &GpioRegisters = &self.port;

        // clear GPIO enable for pin
        port.gper.clear.set(self.pin_mask);

        // Set PMR0-2 according to passed in peripheral
        if bit0 == 0 {
            port.pmr0.clear.set(self.pin_mask);
        } else {
            port.pmr0.set.set(self.pin_mask);
        }
        if bit1 == 0 {
            port.pmr1.clear.set(self.pin_mask);
        } else {
            port.pmr1.set.set(self.pin_mask);
        }
        if bit2 == 0 {
            port.pmr2.clear.set(self.pin_mask);
        } else {
            port.pmr2.set.set(self.pin_mask);
        }
    }

    pub fn enable(&self) {
        let port: &GpioRegisters = &self.port;
        port.gper.set.set(self.pin_mask);
    }

    pub fn disable(&self) {
        let port: &GpioRegisters = &self.port;
        port.gper.clear.set(self.pin_mask);
    }

    pub fn is_pending(&self) -> bool {
        let port: &GpioRegisters = &self.port;
        (port.ifr.val.get() & self.pin_mask) != 0
    }

    pub fn enable_output(&self) {
        let port: &GpioRegisters = &self.port;
        port.oder.set.set(self.pin_mask);
    }

    pub fn disable_output(&self) {
        let port: &GpioRegisters = &self.port;
        port.oder.clear.set(self.pin_mask);
    }

    pub fn enable_pull_down(&self) {
        let port: &GpioRegisters = &self.port;
        port.pder.set.set(self.pin_mask);
    }

    pub fn disable_pull_down(&self) {
        let port: &GpioRegisters = &self.port;
        port.pder.clear.set(self.pin_mask);
    }

    pub fn enable_pull_up(&self) {
        let port: &GpioRegisters = &self.port;
        port.puer.set.set(self.pin_mask);
    }

    pub fn disable_pull_up(&self) {
        let port: &GpioRegisters = &self.port;
        port.puer.clear.set(self.pin_mask);
    }

    /// Sets the interrupt mode registers. Interrupts may fire on the rising or
    /// falling edge of the pin or on both.
    ///
    /// The mode is a two-bit value based on the mapping from section 23.7.13 of
    /// the SAM4L datasheet (page 563):
    ///
    /// | `mode` value | Interrupt Mode |
    /// | ------------ | -------------- |
    /// | 0b00         | Pin change     |
    /// | 0b01         | Rising edge    |
    /// | 0b10         | Falling edge   |
    ///
    pub fn set_interrupt_mode(&self, mode: u8) {
        let port: &GpioRegisters = &self.port;
        if mode & 0b01 != 0 {
            port.imr0.set.set(self.pin_mask);
        } else {
            port.imr0.clear.set(self.pin_mask);
        }

        if mode & 0b10 != 0 {
            port.imr1.set.set(self.pin_mask);
        } else {
            port.imr1.clear.set(self.pin_mask);
        }
    }

    pub fn enable_interrupt(&self) {
        let port: &GpioRegisters = &self.port;
        if port.ier.val.get() & self.pin_mask == 0 {
            INTERRUPT_COUNT.fetch_add(1, Ordering::Relaxed);
            port.ier.set.set(self.pin_mask);
        }
    }

    pub fn disable_interrupt(&self) {
        let port: &GpioRegisters = &self.port;
        if port.ier.val.get() & self.pin_mask != 0 {
            INTERRUPT_COUNT.fetch_sub(1, Ordering::Relaxed);
            port.ier.clear.set(self.pin_mask);
        }
    }

    pub fn handle_interrupt(&self) {
        self.client.map(|client| {
            client.fired();
        });
    }

    pub fn disable_schmidtt_trigger(&self) {
        let port: &GpioRegisters = &self.port;
        port.ster.clear.set(self.pin_mask);
    }

    pub fn enable_schmidtt_trigger(&self) {
        let port: &GpioRegisters = &self.port;
        port.ster.set.set(self.pin_mask);
    }

    pub fn read(&self) -> bool {
        let port: &GpioRegisters = &self.port;
        (port.pvr.get() & self.pin_mask) > 0
    }

    pub fn toggle(&self) -> bool {
        let port: &GpioRegisters = &self.port;
        port.ovr.toggle.set(self.pin_mask);
        self.read()
    }

    pub fn set(&self) {
        let port: &GpioRegisters = &self.port;
        port.ovr.set.set(self.pin_mask);
    }

    pub fn clear(&self) {
        let port: &GpioRegisters = &self.port;
        port.ovr.clear.set(self.pin_mask);
    }
}

impl hil::Controller for GPIOPin<'_> {
    type Config = Option<PeripheralFunction>;

    fn configure(&self, config: Self::Config) {
        match config {
            Some(c) => self.select_peripheral(c),
            None => self.enable(),
        }
    }
}

impl gpio::Configure for GPIOPin<'_> {
    fn set_floating_state(&self, mode: gpio::FloatingState) {
        match mode {
            gpio::FloatingState::PullUp => {
                self.disable_pull_down();
                self.enable_pull_up();
            }
            gpio::FloatingState::PullDown => {
                self.disable_pull_up();
                self.enable_pull_down();
            }
            gpio::FloatingState::PullNone => {
                self.disable_pull_up();
                self.disable_pull_down();
            }
        }
    }

    fn deactivate_to_low_power(&self) {
        GPIOPin::disable(self);
    }

    fn make_output(&self) -> gpio::Configuration {
        self.enable();
        GPIOPin::enable_output(self);
        self.disable_schmidtt_trigger();
        gpio::Configuration::Output
    }

    fn make_input(&self) -> gpio::Configuration {
        self.enable();
        GPIOPin::disable_output(self);
        self.enable_schmidtt_trigger();
        gpio::Configuration::Input
    }

    fn disable_output(&self) -> gpio::Configuration {
        let port: &GpioRegisters = &self.port;
        port.oder.clear.set(self.pin_mask);
        self.configuration()
    }

    fn disable_input(&self) -> gpio::Configuration {
        self.configuration()
    }

    fn is_input(&self) -> bool {
        let port: &GpioRegisters = &self.port;
        port.gper.val.get() & self.pin_mask != 0
    }

    fn is_output(&self) -> bool {
        let port: &GpioRegisters = &self.port;
        port.oder.val.get() & self.pin_mask != 0
    }

    fn floating_state(&self) -> gpio::FloatingState {
        let port: &GpioRegisters = &self.port;
        let down = (port.pder.val.get() & self.pin_mask) != 0;
        let up = (port.puer.val.get() & self.pin_mask) != 0;
        if down {
            gpio::FloatingState::PullDown
        } else if up {
            gpio::FloatingState::PullUp
        } else {
            gpio::FloatingState::PullNone
        }
    }

    fn configuration(&self) -> gpio::Configuration {
        let port: &GpioRegisters = &self.port;
        let input = self.is_input();
        let output = self.is_output();
        let gpio = (port.gper.val.get() & self.pin_mask) == 1;
        let config = (gpio, input, output);
        match config {
            (false, _, _) => gpio::Configuration::Function,
            (true, false, false) => gpio::Configuration::Other,
            (true, false, true) => gpio::Configuration::Output,
            (true, true, false) => gpio::Configuration::Input,
            (true, true, true) => gpio::Configuration::InputOutput,
        }
    }
}

impl gpio::Input for GPIOPin<'_> {
    fn read(&self) -> bool {
        GPIOPin::read(self)
    }
}

impl gpio::Output for GPIOPin<'_> {
    fn toggle(&self) -> bool {
        GPIOPin::toggle(self)
    }

    fn set(&self) {
        GPIOPin::set(self);
    }

    fn clear(&self) {
        GPIOPin::clear(self);
    }
}

impl<'a> gpio::Interrupt<'a> for GPIOPin<'a> {
    fn enable_interrupts(&self, mode: gpio::InterruptEdge) {
        let mode_bits = match mode {
            hil::gpio::InterruptEdge::EitherEdge => 0b00,
            hil::gpio::InterruptEdge::RisingEdge => 0b01,
            hil::gpio::InterruptEdge::FallingEdge => 0b10,
        };
        GPIOPin::set_interrupt_mode(self, mode_bits);
        GPIOPin::enable_interrupt(self);
    }

    fn disable_interrupts(&self) {
        GPIOPin::disable_interrupt(self);
    }

    fn set_client(&self, client: &'a dyn gpio::Client) {
        GPIOPin::set_client(self, client);
    }

    fn is_pending(&self) -> bool {
        GPIOPin::is_pending(self)
    }
}
