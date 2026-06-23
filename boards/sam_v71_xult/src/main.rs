//! Tock kernel for the SAMV71 Xplained Ultra evaluation board.
//!
//! - ATSAMV71Q21B, Cortex-M7, 300 MHz / 150 MHz MCK
//! - Console: USART1 via EDBG CDC (PA21 RXD, PB4 TXD), 115200 baud
//! - LED0: PA23, LED1: PC9 (active-low)
//! - Alarm: TC0 channel 0, SLCK @ 32 kHz

#![no_std]
#![no_main]
#![deny(missing_docs)]

use core::ptr::{addr_of, addr_of_mut};

use kernel::capabilities;
use kernel::component::Component;
use kernel::hil;
use kernel::hil::time::Counter;
use kernel::platform::{KernelResources, SyscallDriverLookup};
#[allow(unused_imports)]
use kernel::{create_capability, debug, static_init};

use capsules_system::scheduler::round_robin::RoundRobinSched;

use samv71q21b::chip::{Atsamv71q21b, Atsamv71q21bDefaultPeripherals};
use samv71q21b::gpio::PeripheralFunction;
use samv71q21b::pmc;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const NUM_PROCS: u8 = 4;
const NUM_PROCS_USIZE: usize = NUM_PROCS as usize;

/// Stack for the kernel (8 KB).
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

static mut CHIP: Option<&'static Atsamv71q21b<Atsamv71q21bDefaultPeripherals>> = None;

// ---------------------------------------------------------------------------
// Platform
// ---------------------------------------------------------------------------

/// Board platform struct.
struct SamV71Xult {
    /// Console driver.
    console: &'static capsules_core::console::Console<'static>,
    /// LED driver (LED0=PA23, LED1=PC9).
    led: &'static capsules_core::led::LedDriver<
        'static,
        kernel::hil::led::LedLow<'static, samv71q21b::gpio::GPIOPin<'static>>,
        2,
    >,
    /// Alarm driver for userspace.
    alarm: &'static capsules_core::alarm::AlarmDriver<
        'static,
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<
            'static,
            samv71q21b::tc::Tc<'static>,
        >,
    >,
    /// Scheduler.
    scheduler: &'static RoundRobinSched<'static>,
    /// SysTick for preemptive scheduling.
    systick: cortexm7::systick::SysTick,
}

impl SyscallDriverLookup for SamV71Xult {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::led::DRIVER_NUM => f(Some(self.led)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            _ => f(None),
        }
    }
}

impl KernelResources<Atsamv71q21b<Atsamv71q21bDefaultPeripherals>> for SamV71Xult {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm7::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup { self }
    fn syscall_filter(&self) -> &Self::SyscallFilter { &() }
    fn process_fault(&self) -> &Self::ProcessFault { &() }
    fn scheduler(&self) -> &Self::Scheduler { self.scheduler }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer { &self.systick }
    fn watchdog(&self) -> &Self::WatchDog { &() }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback { &() }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Main kernel entry point.
#[no_mangle]
pub unsafe fn main() {
    // Disable watchdog (write-once register).
    core::ptr::write_volatile(0x400E_1854 as *mut u32, 0x0000_8000);

    // Early PMC enables: PIOA(10), PIOB(11), PIOC(12), USART1(14), TC0_CH0(23).
    core::ptr::write_volatile(
        0x400E_0610 as *mut u32,
        (1u32 << 10) | (1u32 << 11) | (1u32 << 12) | (1u32 << 14),
    );

    // Flash wait states for 150 MHz.
    core::ptr::write_volatile(0x400E_0C00 as *mut u32, 0x0400_0600);

    samv71q21b::init();

    // Deferred call state (must be early, before any component that uses it).
    kernel::deferred_call::initialize_deferred_call_state_unsafe::<
        cortexm7::thread_id::CortexMThreadIdProvider,
    >();

    pmc::PMC.setup_clocks();

    let peripherals = static_init!(
        Atsamv71q21bDefaultPeripherals,
        Atsamv71q21bDefaultPeripherals::new()
    );
    peripherals.efc.init();

    // PMC enables via HAL.
    pmc::PMC.enable_peripheral_clock(samv71q21b::uart::USART1_PID);
    pmc::PMC.enable_peripheral_clock(10); // PIOA
    pmc::PMC.enable_peripheral_clock(11); // PIOB
    pmc::PMC.enable_peripheral_clock(12); // PIOC
    pmc::PMC.enable_peripheral_clock(samv71q21b::tc::TC0_CH0_PID);

    // Pin mux: USART1 EDBG CDC.
    {
        let ccfg_sysio = 0x4008_8114 as *mut u32;
        core::ptr::write_volatile(
            ccfg_sysio,
            core::ptr::read_volatile(ccfg_sysio) | (1u32 << 4),
        );
    }
    peripherals.pa.pin(21).select_peripheral(PeripheralFunction::A);
    peripherals.pb.pin(4).select_peripheral(PeripheralFunction::D);

    // Process array.
    let processes = components::process_array::ProcessArrayComponent::new()
        .finalize(components::process_array_component_static!(NUM_PROCS_USIZE));

    let board_kernel = static_init!(
        kernel::Kernel,
        kernel::Kernel::new(processes.as_slice())
    );

    let chip = static_init!(
        Atsamv71q21b<Atsamv71q21bDefaultPeripherals>,
        Atsamv71q21b::new(peripherals)
    );
    CHIP = Some(chip);

    // NVIC: enable only the interrupts we handle.
    cortexm7::nvic::Nvic::new(14).enable(); // USART1
    cortexm7::nvic::Nvic::new(6).enable();  // EFC
    cortexm7::nvic::Nvic::new(23).enable(); // TC0_CH0
    cortexm7::nvic::Nvic::new(10).enable(); // PIOA
    cortexm7::nvic::Nvic::new(11).enable(); // PIOB
    cortexm7::nvic::Nvic::new(12).enable(); // PIOC

    // -----------------------------------------------------------------------
    // Alarm (TC0 @ 32 kHz SLCK)
    // -----------------------------------------------------------------------
    let _ = peripherals.tc0.start();

    let mux_alarm = components::alarm::AlarmMuxComponent::new(&peripherals.tc0)
        .finalize(components::alarm_mux_component_static!(samv71q21b::tc::Tc));

    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules_core::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_static!(samv71q21b::tc::Tc));

    // -----------------------------------------------------------------------
    // Console
    // -----------------------------------------------------------------------
    let uart_mux = components::console::UartMuxComponent::new(&peripherals.usart1, 115200)
        .finalize(components::uart_mux_component_static!());

    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());

    hil::uart::Transmit::set_transmit_client(&peripherals.usart1, uart_mux);
    hil::uart::Receive::set_receive_client(&peripherals.usart1, uart_mux);

    // Debug writer for kernel debug!() output over USART1.
    components::debug_writer::DebugWriterComponent::new::<
        cortexm7::thread_id::CortexMThreadIdProvider,
    >(
        uart_mux,
        create_capability!(capabilities::SetDebugWriterCapability),
    )
    .finalize(components::debug_writer_component_static!());

    // -----------------------------------------------------------------------
    // LED
    // -----------------------------------------------------------------------
    let led = components::led::LedsComponent::new().finalize(components::led_component_static!(
        kernel::hil::led::LedLow<'static, samv71q21b::gpio::GPIOPin<'static>>,
        kernel::hil::led::LedLow::new(peripherals.pa.pin(23)), // LED0
        kernel::hil::led::LedLow::new(peripherals.pc.pin(9)),  // LED1
    ));

    // -----------------------------------------------------------------------
    // Scheduler
    // -----------------------------------------------------------------------
    let scheduler =
        components::sched::round_robin::RoundRobinComponent::new(processes)
            .finalize(components::round_robin_component_static!(NUM_PROCS_USIZE));

    let platform = SamV71Xult {
        console,
        led,
        alarm,
        scheduler,
        systick: cortexm7::systick::SysTick::new_with_calibration(300_000_000),
    };

    // -----------------------------------------------------------------------
    // Process loading
    // -----------------------------------------------------------------------
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            addr_of!(_sapps),
            addr_of!(_eapps) as usize - addr_of!(_sapps) as usize,
        ),
        core::slice::from_raw_parts_mut(
            addr_of_mut!(_sappmem),
            addr_of!(_eappmem) as usize - addr_of!(_sappmem) as usize,
        ),
        &capsules_system::process_policies::PanicFaultPolicy {},
        &process_management_capability,
    )
    .unwrap_or_else(|_err| {});

    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    board_kernel.kernel_loop(
        &platform,
        chip,
        None::<&kernel::ipc::IPC<{ NUM_PROCS }>>,
        &main_loop_capability,
    );
}

#[cfg(not(test))]
#[panic_handler]
/// Panic handler.
pub unsafe fn panic_fmt(_pi: &core::panic::PanicInfo) -> ! {
    loop {}
}
