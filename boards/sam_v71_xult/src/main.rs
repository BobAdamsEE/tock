#![no_std]
#![no_main]

use core::arch::asm;

use panic_halt as _;

use atsamv71q21b::gpio::PortA;
use kernel::hil::gpio::Configure; // This fixes make_output

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let porta = PortA::new_port_a();

    let mut pa23 = porta.pin(23);

    pa23.make_output(); // Now compiles
    pa23.clear(); // Active-low: turns LED ON

    loop {
        pa23.toggle();

        for _ in 0..20_000_000 {
            unsafe {
                asm!("nop");
            }
        }
    }
}
