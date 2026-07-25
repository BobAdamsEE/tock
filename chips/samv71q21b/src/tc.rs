// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! TC0 Channel 0 timer driver for SAMV71Q21B.
//!
//! Runs in WAVEFORM UP mode on TIMER_CLOCK5 (SLCK ≈ 32,768 Hz).
//! A software overflow counter extends the 16-bit hardware counter to 32 bits,
//! giving a range of ~2M seconds at Freq32KHz resolution.
//!
//! Implements `hil::time::Counter` and `hil::time::Alarm`.

use core::cell::Cell;

use kernel::hil::time::{self, Ticks, Ticks32};
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

// ---------------------------------------------------------------------------
// Register layout – one TC channel block (offset within TC0 base)
// ---------------------------------------------------------------------------

register_structs! {
    TcChannelRegisters {
        (0x000 => ccr:  WriteOnly<u32, Ccr::Register>),
        (0x004 => cmr:  ReadWrite<u32, Cmr::Register>),
        (0x008 => _smmr),
        (0x00C => _rab),
        (0x010 => cv:   ReadOnly<u32>),
        (0x014 => ra:   ReadWrite<u32>),
        (0x018 => rb:   ReadWrite<u32>),
        (0x01C => rc:   ReadWrite<u32>),
        (0x020 => sr:   ReadOnly<u32,  Sr::Register>),
        (0x024 => ier:  WriteOnly<u32, Ir::Register>),
        (0x028 => idr:  WriteOnly<u32, Ir::Register>),
        (0x02C => imr:  ReadOnly<u32,  Ir::Register>),
        (0x030 => _emr),
        (0x034 => @END),
    }
}

register_bitfields![u32,
    Ccr [
        CLKEN  OFFSET(0) NUMBITS(1) [],
        CLKDIS OFFSET(1) NUMBITS(1) [],
        SWTRG  OFFSET(2) NUMBITS(1) [],
    ],
    Cmr [
        TCCLKS  OFFSET(0)  NUMBITS(3) [
            Clock1 = 0,  // MCK/2
            Clock2 = 1,  // MCK/8
            Clock3 = 2,  // MCK/32
            Clock4 = 3,  // MCK/128
            Clock5 = 4,  // SLCK ≈ 32 kHz
        ],
        WAVE    OFFSET(15) NUMBITS(1) [],
        WAVSEL  OFFSET(13) NUMBITS(2) [
            Up         = 0,  // free-running 0→0xFFFF
            UpAuto     = 2,  // auto-reset at RC
        ],
    ],
    Sr [
        COVFS OFFSET(0) NUMBITS(1) [],
        CPCS  OFFSET(4) NUMBITS(1) [],
    ],
    Ir [
        COVFS OFFSET(0) NUMBITS(1) [],
        CPCS  OFFSET(4) NUMBITS(1) [],
    ],
];

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// TC0 Channel 0 base address.
const TC0_CH0_BASE: StaticRef<TcChannelRegisters> =
    unsafe { StaticRef::new(0x4000_C000 as *const TcChannelRegisters) };

/// TC0 Channel 0 peripheral ID (for PMC clock enable).
pub const TC0_CH0_PID: u32 = 23;

pub struct Tc<'a> {
    regs: StaticRef<TcChannelRegisters>,
    overflow: Cell<u32>,
    alarm_target: Cell<u32>,
    armed: Cell<bool>,
    client: OptionalCell<&'a dyn time::AlarmClient>,
    running: Cell<bool>,
}

impl<'a> Tc<'a> {
    pub const fn new() -> Self {
        Tc {
            regs: TC0_CH0_BASE,
            overflow: Cell::new(0),
            alarm_target: Cell::new(0),
            armed: Cell::new(false),
            client: OptionalCell::empty(),
            running: Cell::new(false),
        }
    }

    /// Call from the NVIC interrupt handler for TC0 CH0.
    pub fn handle_interrupt(&self) {
        // Reading SR clears all flags.
        let sr = self.regs.sr.extract();

        if sr.is_set(Sr::COVFS) {
            // Overflow: advance the 32-bit virtual counter high half.
            let new_high = self.overflow.get().wrapping_add(1);
            self.overflow.set(new_high);

            // If we're armed but waiting for the high half to reach the target,
            // check whether it's time to load RC.
            if self.armed.get() {
                let target = self.alarm_target.get();
                let target_high = target >> 16;
                if new_high == target_high {
                    let target_low = target & 0xFFFF;
                    self.regs.rc.set(target_low);
                    self.regs.ier.write(Ir::CPCS::SET);
                }
            }
        }

        if sr.is_set(Sr::CPCS) && self.armed.get() {
            // Alarm fired.
            self.regs.idr.write(Ir::CPCS::SET);
            self.armed.set(false);
            self.client.map(|c| c.alarm());
        }
    }

    /// Read the current 32-bit virtual tick count, handling a potential
    /// overflow race between reading high and low halves.
    fn now_u32(&self) -> u32 {
        loop {
            let high = self.overflow.get();
            let low = self.regs.cv.get();
            // If overflow_count didn't change while we read cv, we're safe.
            if self.overflow.get() == high {
                return (high << 16) | (low & 0xFFFF);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// HIL: Time
// ---------------------------------------------------------------------------

impl<'a> time::Time for Tc<'a> {
    type Frequency = time::Freq32KHz;
    type Ticks = Ticks32;

    fn now(&self) -> Ticks32 {
        Ticks32::from(self.now_u32())
    }
}

// ---------------------------------------------------------------------------
// HIL: Counter
// ---------------------------------------------------------------------------

impl<'a> time::Counter<'a> for Tc<'a> {
    fn set_overflow_client(&self, _client: &'a dyn time::OverflowClient) {
        // Not used by bootloader; overflow is tracked internally.
    }

    fn start(&self) -> Result<(), ErrorCode> {
        if self.running.get() {
            return Ok(());
        }
        // WAVEFORM, UP (free-running), TIMER_CLOCK5 (SLCK).
        self.regs.cmr.write(Cmr::WAVE::SET + Cmr::WAVSEL::Up + Cmr::TCCLKS::Clock5);
        // Enable COVFS so we can track overflows.
        self.regs.ier.write(Ir::COVFS::SET);
        // Enable clock and trigger.
        self.regs.ccr.write(Ccr::CLKEN::SET + Ccr::SWTRG::SET);
        self.running.set(true);
        Ok(())
    }

    fn stop(&self) -> Result<(), ErrorCode> {
        self.regs.idr.write(Ir::COVFS::SET + Ir::CPCS::SET);
        self.regs.ccr.write(Ccr::CLKDIS::SET);
        self.running.set(false);
        Ok(())
    }

    fn reset(&self) -> Result<(), ErrorCode> {
        self.overflow.set(0);
        self.regs.ccr.write(Ccr::SWTRG::SET);
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.running.get()
    }
}

// ---------------------------------------------------------------------------
// HIL: Alarm
// ---------------------------------------------------------------------------

impl<'a> time::Alarm<'a> for Tc<'a> {
    fn set_alarm_client(&self, client: &'a dyn time::AlarmClient) {
        self.client.set(client);
    }

    fn set_alarm(&self, reference: Ticks32, dt: Ticks32) {
        let target = reference.wrapping_add(dt);
        self.alarm_target.set(target.into_u32());
        self.armed.set(true);

        let now = self.now_u32();
        let target_val = target.into_u32();
        let target_high = target_val >> 16;
        let now_high = now >> 16;

        // If the high half already matches, load RC and arm CPCS immediately.
        // Otherwise COVFS handler will do it when overflow count reaches target_high.
        if now_high == target_high {
            let target_low = target_val & 0xFFFF;
            self.regs.rc.set(target_low);
            self.regs.ier.write(Ir::CPCS::SET);
        }
        // COVFS interrupt stays enabled from start() for overflow tracking.
    }

    fn get_alarm(&self) -> Ticks32 {
        Ticks32::from(self.alarm_target.get())
    }

    fn disarm(&self) -> Result<(), ErrorCode> {
        self.armed.set(false);
        self.regs.idr.write(Ir::CPCS::SET);
        Ok(())
    }

    fn is_armed(&self) -> bool {
        self.armed.get()
    }

    fn minimum_dt(&self) -> Ticks32 {
        // At 32,768 Hz one tick ≈ 30 µs; require at least 4 ticks for safety.
        Ticks32::from(4)
    }
}
