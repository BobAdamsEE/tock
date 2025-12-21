#![no_std]
#![no_main]

use core::arch::asm;

use atsamv71q21b::gpio::{GPIOPin, PortA};

// Import the Tock GPIO HIL traits
use kernel::hil::gpio::{Configure, Output};

#[no_mangle]
pub unsafe fn reset_handler() {
    // Create PortA instance
    let porta = PortA::new_port_a();

    // Get PA23
    let pa23 = porta.pin(23);

    // Now the methods are available
    pa23.make_output(); // Configure as output
    pa23.clear(); // Active-low: clear = turn LED ON

    loop {
        pa23.toggle();

        // Rough delay — tune if needed (post-reset clock is slow, ~4-8 MHz)
        for _ in 0..3_000_000 {
            asm!("nop");
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
