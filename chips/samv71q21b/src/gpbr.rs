use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadWrite};
use kernel::utilities::StaticRef;

const GPBR_BASE: StaticRef<GpbrRegisters> =
    unsafe { StaticRef::new(0x400E1890 as *const GpbrRegisters) };

register_structs! {
    GpbrRegisters {
        (0x000 => gpbr0: ReadWrite<u32, GPBR0::Register>),
        (0x004 => gpbr1: ReadWrite<u32, GPBR1::Register>),
        (0x008 => gpbr2: ReadWrite<u32, GPBR2::Register>),
        (0x00C => gpbr3: ReadWrite<u32, GPBR3::Register>),
        (0x010 => gpbr4: ReadWrite<u32, GPBR4::Register>),
        (0x014 => gpbr5: ReadWrite<u32, GPBR5::Register>),
        (0x018 => gpbr6: ReadWrite<u32, GPBR6::Register>),
        (0x01C => gpbr7: ReadWrite<u32, GPBR7::Register>),
        (0x020 => @END),
    }
}
register_bitfields![u32,
GPBR0 [
    /// General Purpose Backup Data
    GPBR_VALUE0 OFFSET(0) NUMBITS(32) []
],
GPBR1 [
    /// General Purpose Backup Data
    GPBR_VALUE1 OFFSET(0) NUMBITS(32) []
],
GPBR2 [
    /// General Purpose Backup Data
    GPBR_VALUE2 OFFSET(0) NUMBITS(32) []
],
GPBR3 [
    /// General Purpose Backup Data
    GPBR_VALUE3 OFFSET(0) NUMBITS(32) []
],
GPBR4 [
    /// General Purpose Backup Data
    GPBR_VALUE4 OFFSET(0) NUMBITS(32) []
],
GPBR5 [
    /// General Purpose Backup Data
    GPBR_VALUE5 OFFSET(0) NUMBITS(32) []
],
GPBR6 [
    /// General Purpose Backup Data
    GPBR_VALUE6 OFFSET(0) NUMBITS(32) []
],
GPBR7 [
    /// General Purpose Backup Data
    GPBR_VALUE7 OFFSET(0) NUMBITS(32) []
]
];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GpbrIndex {
    Gpbr0 = 0,
    Gpbr1 = 1,
    Gpbr2 = 2,
    Gpbr3 = 3,
    Gpbr4 = 4,
    Gpbr5 = 5,
    Gpbr6 = 6,
    Gpbr7 = 7,
}

pub struct Gpbr {
    registers: StaticRef<GpbrRegisters>,
}

impl Gpbr {
    pub const fn new() -> Self {
        Gpbr {
            registers: GPBR_BASE,
        }
    }

    pub fn set(&self, index: GpbrIndex, value: u32) {
        // No panic needed — exhaustive match
        match index {
            GpbrIndex::Gpbr0 => self.registers.gpbr0.set(value),
            GpbrIndex::Gpbr1 => self.registers.gpbr1.set(value),
            GpbrIndex::Gpbr2 => self.registers.gpbr2.set(value),
            GpbrIndex::Gpbr3 => self.registers.gpbr3.set(value),
            GpbrIndex::Gpbr4 => self.registers.gpbr4.set(value),
            GpbrIndex::Gpbr5 => self.registers.gpbr5.set(value),
            GpbrIndex::Gpbr6 => self.registers.gpbr6.set(value),
            GpbrIndex::Gpbr7 => self.registers.gpbr7.set(value),
        }
    }

    pub fn get(&self, index: GpbrIndex) -> u32 {
        match index {
            GpbrIndex::Gpbr0 => self.registers.gpbr0.get(),
            GpbrIndex::Gpbr1 => self.registers.gpbr1.get(),
            GpbrIndex::Gpbr2 => self.registers.gpbr2.get(),
            GpbrIndex::Gpbr3 => self.registers.gpbr3.get(),
            GpbrIndex::Gpbr4 => self.registers.gpbr4.get(),
            GpbrIndex::Gpbr5 => self.registers.gpbr5.get(),
            GpbrIndex::Gpbr6 => self.registers.gpbr6.get(),
            GpbrIndex::Gpbr7 => self.registers.gpbr7.get(),
        }
    }
}
