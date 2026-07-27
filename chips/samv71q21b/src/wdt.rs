//! Watchdog timer.
//!
//! # WDT_MR is write-once
//!
//! The mode register can be written **once** after a reset, and every later
//! write is ignored. That single fact shapes how this is used:
//!
//! * Whoever writes it first decides for everyone. The bootloader jumps to the
//!   kernel without an intervening reset, so the kernel inherits whatever the
//!   bootloader chose and cannot change it. A kernel-side "disable the
//!   watchdog" write after a bootloader jump is a no-op, and a misleading one.
//! * There is no enabling it later. Deciding to disable at startup and arm it
//!   nearer the kernel is not possible; the choice has to be made before
//!   anything long-running happens.
//!
//! # Why idle and debug halt matter
//!
//! The counter is clocked at SLCK/128, about 256 Hz, and WDV is 12 bits, so
//! the longest period is roughly 16 seconds.
//!
//! `WDIDLEHLT` stops the counter while the core is idle, and `WDDBGHLT` stops
//! it while a debugger has the core halted. Both are essential here rather
//! than optional:
//!
//! * The bootloader sleeps indefinitely waiting for a host command. Without
//!   `WDIDLEHLT` a board sitting in the bootloader would reset every few
//!   seconds, which is precisely when someone is trying to recover it.
//! * Without `WDDBGHLT` a breakpoint would reset the board, making the
//!   watchdog and the debugger mutually exclusive.
//!
//! Together they give the semantics wanted: a core that is *spinning* gets
//! reset, a core that is legitimately waiting does not.

use kernel::platform::watchdog;
use kernel::utilities::registers::interfaces::Writeable;
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;

const WDT_BASE: StaticRef<WdtRegisters> =
    unsafe { StaticRef::new(0x400E1850 as *const WdtRegisters) };

register_structs! {
    WdtRegisters {
        (0x00 => cr: WriteOnly<u32, CR::Register>),
        (0x04 => mr: ReadWrite<u32, MR::Register>),
        (0x08 => sr: ReadOnly<u32>),
        (0x0C => @END),
    }
}

register_bitfields![u32,
CR [
    /// Restart the watchdog. Must be written with the password.
    WDRSTT OFFSET(0) NUMBITS(1) [],
    /// 0xA5, or the write is ignored.
    KEY OFFSET(24) NUMBITS(8) []
],
MR [
    /// Counter value, in units of SLCK/128 (~3.9 ms).
    WDV OFFSET(0) NUMBITS(12) [],
    /// Fault interrupt enable.
    WDFIEN OFFSET(12) NUMBITS(1) [],
    /// Reset the chip when the counter underflows.
    WDRSTEN OFFSET(13) NUMBITS(1) [],
    /// Reset the processor only, rather than the whole chip.
    WDRPROC OFFSET(14) NUMBITS(1) [],
    /// Disable the watchdog entirely.
    WDDIS OFFSET(15) NUMBITS(1) [],
    /// Permitted window: restarts before this are themselves a fault.
    WDD OFFSET(16) NUMBITS(12) [],
    /// Halt the counter while a debugger has the core halted.
    WDDBGHLT OFFSET(28) NUMBITS(1) [],
    /// Halt the counter while the core is idle.
    WDIDLEHLT OFFSET(29) NUMBITS(1) []
]
];

/// Password for WDT_CR, from the datasheet.
const KEY: u32 = 0xA5;

/// Counter ticks per second: SLCK (32768 Hz) / 128.
const TICKS_PER_SECOND: u32 = 256;

/// Longest period WDV can express.
pub const MAX_PERIOD_MS: u32 = 4095 * 1000 / TICKS_PER_SECOND;

pub struct Wdt {
    registers: StaticRef<WdtRegisters>,
}

impl Wdt {
    pub const fn new() -> Wdt {
        Wdt {
            registers: WDT_BASE,
        }
    }

    /// Arm the watchdog with the given period, and never reset while idle or
    /// halted in a debugger.
    ///
    /// Writes WDT_MR, so this may be called only once after a reset and only
    /// if nothing has disabled the watchdog first. `period_ms` is clamped to
    /// what the counter can express.
    pub fn start(&self, period_ms: u32) {
        let ticks = (period_ms.min(MAX_PERIOD_MS) * TICKS_PER_SECOND / 1000).max(1);

        self.registers.mr.write(
            MR::WDV.val(ticks)
                // No forbidden window: a restart is always allowed. A window
                // would catch code petting the watchdog *too often*, which is
                // a real fault class, but it also turns a harmless extra pet
                // into a reset -- not a trade worth making on a development
                // board.
                + MR::WDD.val(ticks)
                + MR::WDRSTEN::SET
                + MR::WDDBGHLT::SET
                + MR::WDIDLEHLT::SET,
        );
    }

    /// Disable the watchdog. Also a write to WDT_MR, so it is equally final.
    pub fn disable(&self) {
        self.registers.mr.write(MR::WDDIS::SET);
    }

    /// Restart the counter.
    ///
    /// Safe to call at any time, including from a long-running loop that has
    /// not returned to the kernel's main loop -- which is the case that needs
    /// it, since a multi-page flash erase never yields.
    pub fn pet(&self) {
        self.registers
            .cr
            .write(CR::KEY.val(KEY) + CR::WDRSTT::SET);
    }
}

impl watchdog::WatchDog for Wdt {
    fn setup(&self) {
        // Deliberately not `start()`. WDT_MR has already been written by the
        // bootloader, which is the only place that can choose the period, so
        // writing it here would be ignored -- and if this kernel were ever
        // booted without a bootloader, arming it here would apply a period
        // chosen for a different context.
        self.pet();
    }

    fn tickle(&self) {
        self.pet();
    }

    fn suspend(&self) {
        // Nothing to do: `WDIDLEHLT` stops the counter in hardware for exactly
        // the sleep this precedes. Petting here instead would be worse than
        // useless -- it would say "still alive" on the way into a sleep that
        // might never end.
    }

    fn resume(&self) {
        self.pet();
    }
}
