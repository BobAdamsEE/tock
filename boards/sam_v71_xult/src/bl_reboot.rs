//! Reboot into the bootloader, for one authorised application only.
//!
//! This is the transition step of the reflashing architecture: the UDS app
//! receives `DiagnosticSessionControl` programming session over CAN, answers
//! it, and then asks the kernel to restart into the bootloader so the actual
//! reprogramming can happen. Without this the bootloader can only be reached
//! by a physical reset.
//!
//! It also offers a plain restart (command 3), which is what the same app's
//! UDS `ECUReset` needs: the board resets and comes back up in the kernel.
//! Both are the same privilege -- a reset is disruptive whichever side of the
//! jump it lands on.
//!
//! # Who is allowed
//!
//! The caller must present a specific [`ShortId`], which the board's AppID
//! assigner grants only to a process whose signature credential was *accepted*
//! (see `credentials.rs`). Unsigned processes are assigned
//! `ShortId::LocallyUnique`, and:
//!
//! ```text
//! (ShortId::Fixed(a), ShortId::Fixed(b)) => a == b,
//! _ => false,
//! ```
//!
//! so they can never match, whatever they do. The check fails closed.
//!
//! Note what this does and does not defend against. It stops *other software
//! on this device* from bouncing the board into the bootloader. It does
//! nothing about a hostile tester on the CAN bus -- that is the UDS app's
//! SecurityAccess, and ultimately signature verification of the image before
//! the bootloader jumps to it.

use kernel::process::ShortId;
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::{ErrorCode, ProcessId};

use samv71q21b::gpbr::{Gpbr, GpbrIndex};

/// Vendor-specific driver number.
pub const DRIVER_NUM: usize = 0x9_0001;

/// Value the bootloader looks for in GPBR7 to stay resident. Must match
/// `DFU_MAGIC_TOCK_BOOTLOADER1` in the bootloader's entry check.
const DFU_MAGIC: u32 = 0x90;

pub struct BootloaderReboot<'a> {
    gpbr: &'a Gpbr,
    /// The only identity permitted to trigger a reboot.
    allowed: ShortId,
}

impl<'a> BootloaderReboot<'a> {
    pub fn new(gpbr: &'a Gpbr, allowed: ShortId) -> BootloaderReboot<'a> {
        BootloaderReboot { gpbr, allowed }
    }

    fn permitted(&self, processid: ProcessId) -> bool {
        // `ShortId::LocallyUnique` compares unequal to everything, so an
        // unsigned process is refused here without a special case.
        processid.short_app_id() == self.allowed
    }
}

impl SyscallDriver for BootloaderReboot<'_> {
    fn command(
        &self,
        command_num: usize,
        _arg1: usize,
        _arg2: usize,
        processid: ProcessId,
    ) -> CommandReturn {
        match command_num {
            // Driver existence check, deliberately open to everyone: refusing
            // it would leak nothing useful and only complicate callers.
            0 => CommandReturn::success(),

            // Is this process allowed to reboot into the bootloader? Lets an
            // app find out without attempting it.
            1 => {
                if self.permitted(processid) {
                    CommandReturn::success_u32(1)
                } else {
                    CommandReturn::success_u32(0)
                }
            }

            // Reboot into the bootloader. Does not return on success.
            2 => {
                if !self.permitted(processid) {
                    return CommandReturn::failure(ErrorCode::NOSUPPORT);
                }
                self.gpbr.set(GpbrIndex::Gpbr7, DFU_MAGIC);
                // GPBR survives a software reset; the bootloader's entry check
                // sees the magic and stays resident instead of jumping to the
                // kernel.
                unsafe {
                    cortexm7::scb::reset();
                }
                #[allow(unreachable_code)]
                CommandReturn::failure(ErrorCode::FAIL)
            }

            // Restart into the kernel: the same reset, without the magic. This
            // is what UDS `ECUReset` means -- come back up running -- whereas
            // command 2 is what `DiagnosticSessionControl` programming session
            // means. Gated identically: a reset is disruptive however it ends,
            // and the caller is the same app either way.
            3 => {
                if !self.permitted(processid) {
                    return CommandReturn::failure(ErrorCode::NOSUPPORT);
                }
                // Clear rather than leave alone. Nothing should be able to set
                // GPBR7 behind our back, but if something did, an ECUReset
                // would land in the bootloader and look like a hang.
                self.gpbr.set(GpbrIndex::Gpbr7, 0);
                unsafe {
                    cortexm7::scb::reset();
                }
                #[allow(unreachable_code)]
                CommandReturn::failure(ErrorCode::FAIL)
            }

            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, _processid: ProcessId) -> Result<(), kernel::process::Error> {
        Ok(())
    }
}
