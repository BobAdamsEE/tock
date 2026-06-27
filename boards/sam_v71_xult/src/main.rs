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
use samv71q21b::mcan;
use samv71q21b::pmc;
use samv71q21b::twihs;

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
    /// CAN driver (MCAN1 via ATA6561 transceiver).
    can: &'static capsules_extra::can::CanCapsule<'static, mcan::Mcan>,
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
            capsules_extra::can::DRIVER_NUM => f(Some(self.can)),
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
// Boot-time EEPROM debug client — prints MAC address on first read
// ---------------------------------------------------------------------------

/// Prints the AT24MAC402 EUI-48 MAC address and serial number via debug!().
struct EepromDebugClient;

impl capsules_extra::at24mac402::At24Mac402Client for EepromDebugClient {
    fn mac_read_complete(&self, mac: &[u8; 6], status: Result<(), kernel::ErrorCode>) {
        if status.is_ok() {
            debug!(
                "AT24MAC402 MAC: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
            );
        } else {
            debug!("AT24MAC402 MAC read failed");
        }
    }

    fn serial_read_complete(&self, serial: &[u8; 16], status: Result<(), kernel::ErrorCode>) {
        if status.is_ok() {
            debug!(
                "AT24MAC402 serial: {:02X}{:02X}{:02X}{:02X}-{:02X}{:02X}{:02X}{:02X}-{:02X}{:02X}{:02X}{:02X}-{:02X}{:02X}{:02X}{:02X}",
                serial[0], serial[1], serial[2], serial[3],
                serial[4], serial[5], serial[6], serial[7],
                serial[8], serial[9], serial[10], serial[11],
                serial[12], serial[13], serial[14], serial[15],
            );
        } else {
            debug!("AT24MAC402 serial read failed");
        }
    }
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

    // Flash wait states for 150 MHz (FWS=6). CLOE intentionally OFF.
    core::ptr::write_volatile(0x400E_0C00 as *mut u32, 0x0000_0600);

    samv71q21b::init();

    // Deferred call state (must be early, before any component that uses it).
    kernel::deferred_call::initialize_deferred_call_state_unsafe::<
        cortexm7::thread_id::CortexMThreadIdProvider,
    >();

    pmc::PMC.setup_clocks();

    let mcan1_msg_ram = static_init!(mcan::MessageRam, mcan::MessageRam::new());

    let peripherals = static_init!(
        Atsamv71q21bDefaultPeripherals,
        Atsamv71q21bDefaultPeripherals::new(mcan1_msg_ram)
    );
    peripherals.efc.init();

    // PMC enables via HAL.
    pmc::PMC.enable_peripheral_clock(samv71q21b::uart::USART1_PID);
    pmc::PMC.enable_peripheral_clock(10); // PIOA
    pmc::PMC.enable_peripheral_clock(11); // PIOB
    pmc::PMC.enable_peripheral_clock(12); // PIOC
    pmc::PMC.enable_peripheral_clock(samv71q21b::tc::TC0_CH0_PID);
    pmc::PMC.enable_peripheral_clock(twihs::TWIHS0_PID);

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

    // Pin mux: TWIHS0 (I2C0) — PA3 = TWD0/SDA, PA4 = TWCK0/SCL.
    peripherals.pa.pin(3).select_peripheral(PeripheralFunction::A);
    peripherals.pa.pin(4).select_peripheral(PeripheralFunction::A);

    // MCAN1: enable peripheral clock + PCK5 as CAN core clock.
    // SAMV71 MCAN uses PCK5 (not GCLK). PCK5 = PLLA / (14+1) = 20 MHz.
    // 20 MHz gives exact 87.5% sample point at 500 kbps (40 TQ per bit).
    pmc::PMC.enable_peripheral_clock(mcan::MCAN1_PID);
    pmc::PMC.configure_pck(5, 2, 14); // PCK5: CSS=2 (PLLA 300 MHz), PRES=14 → 20 MHz

    // Set MCAN DMA base address (CCFG_CAN0 in Matrix).
    // The MCAN controller forms message RAM addresses as:
    //   {CCFG_CAN0.CAN0DMABA[15:0], register_field[13:0], 2'b00}
    // CAN0DMABA must be the upper 16 bits of the SRAM base (0x2040).
    {
        let ccfg_can0 = 0x4008_8110 as *mut u32;
        core::ptr::write_volatile(ccfg_can0, 0x2040_0000u32);
    }

    // Pin mux: MCAN1 — PC12 = RX (Peripheral C), PC14 = TX (Peripheral C)
    peripherals.pc.pin(12).select_peripheral(PeripheralFunction::C);
    peripherals.pc.pin(14).select_peripheral(PeripheralFunction::C);

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
    cortexm7::nvic::Nvic::new(twihs::TWIHS0_PID).enable(); // TWIHS0
    cortexm7::nvic::Nvic::new(mcan::MCAN1_PID).enable();  // MCAN1 INT0
    cortexm7::nvic::Nvic::new(38).enable();                // MCAN1 INT1

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
    // I2C bus (TWIHS0) + AT24MAC402 EEPROM
    // -----------------------------------------------------------------------
    let mux_i2c = components::i2c::I2CMuxComponent::new(&peripherals.twihs0, None)
        .finalize(components::i2c_mux_component_static!(samv71q21b::twihs::Twihs));

    // User EEPROM (256 bytes, R/W) at 0x57.
    let eeprom_i2c = components::i2c::I2CComponent::new(mux_i2c, 0x57)
        .finalize(components::i2c_component_static!(samv71q21b::twihs::Twihs));

    // Extended block (MAC + serial, R/O) at 0x5F.
    let eeprom_i2c_ext = components::i2c::I2CComponent::new(mux_i2c, 0x5F)
        .finalize(components::i2c_component_static!(samv71q21b::twihs::Twihs));

    let eeprom_buf = static_init!([u8; 18], [0; 18]);
    let eeprom = static_init!(
        capsules_extra::at24mac402::At24Mac402<'static>,
        capsules_extra::at24mac402::At24Mac402::new(eeprom_i2c, eeprom_i2c_ext, eeprom_buf)
    );
    eeprom_i2c.set_client(eeprom);
    eeprom_i2c_ext.set_client(eeprom);

    // Boot-time debug: read and print EUI-48 MAC address.
    let eeprom_debug = static_init!(EepromDebugClient, EepromDebugClient);
    eeprom.set_meta_client(eeprom_debug);
    let _ = eeprom.read_mac_address();

    // -----------------------------------------------------------------------
    // CAN (MCAN1 @ 500 kbps via ATA6561 transceiver)
    // -----------------------------------------------------------------------
    let can = components::can::CanComponent::new(
        board_kernel,
        capsules_extra::can::DRIVER_NUM,
        &peripherals.mcan1,
    )
    .finalize(components::can_component_static!(mcan::Mcan));

    kernel::deferred_call::DeferredCallClient::register(&peripherals.mcan1);

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
        can,
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
