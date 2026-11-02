//! Tock kernel for the Chameleon board (PCA-0001-01).
//!
//! DRAFT, written before the first board exists: it builds, but nothing here
//! has run on Chameleon hardware. See README.md for what is not done yet.
//!
//! - ATSAMV71Q21B, Cortex-M7, 300 MHz / 150 MHz MCK from a 20 MHz crystal
//! - Console: SEGGER RTT only. USART1's pins (PA21, PB4) are LIN A here.
//! - Alarm: TC0 channel 0, SLCK @ 32 kHz
//! - CAN B: MCAN1 (PC12 RX, PC14 TX) via TJA1057. CAN A (MCAN0) is idle.
//! - SD card: HSMCI slot A, read-only, on a switched supply (PA24), card
//!   detect PA29

#![no_std]
#![no_main]
#![deny(missing_docs)]

use core::ptr::{addr_of, addr_of_mut};

use kernel::capabilities;
use kernel::component::Component;
use kernel::hil;
use kernel::hil::time::Counter;
use kernel::platform::chip::Chip;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::{create_capability, static_init};

use capsules_system::scheduler::round_robin::RoundRobinSched;

use samv71q21b::chip::{Atsamv71q21b, Atsamv71q21bDefaultPeripherals};
use samv71q21b::gpio::{GPIOPin, PeripheralFunction};
use samv71q21b::mcan;
use samv71q21b::pmc;
use segger::rtt::SeggerRttMemory;

#[path = "../../sam_v71_xult/src/bl_reboot.rs"]
mod bl_reboot;
#[path = "../../sam_v71_xult/src/credentials.rs"]
mod credentials;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Y1. The Xplained Ultra's is 12 MHz; assuming that here would put PLLA at
/// 500 MHz.
const CRYSTAL_HZ: u32 = 20_000_000;

/// Package name of the one application permitted to reboot the board into the
/// bootloader. See the sam_v71_xult board for why trusting the name is sound.
const PRIVILEGED_APP_NAME: &str = "uds";

const NUM_PROCS: u8 = 4;
const NUM_PROCS_USIZE: usize = NUM_PROCS as usize;

/// Stack for the kernel (8 KB).
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

/// The SD host, for the type of the userspace driver.
type SdHost = samv71q21b::hsmci::Hsmci<
    'static,
    capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, samv71q21b::tc::Tc<'static>>,
>;

type ChipHw = Atsamv71q21b<Atsamv71q21bDefaultPeripherals>;

static mut CHIP: Option<&'static ChipHw> = None;

/// The RTT control block, for the panic handler. The `SeggerRtt` driver owns
/// the block in normal running.
static mut RTT_MEMORY: Option<*const SeggerRttMemory<'static>> = None;

// ---------------------------------------------------------------------------
// Platform
// ---------------------------------------------------------------------------

/// Board platform struct.
struct Chameleon {
    /// Internal watchdog, armed by the bootloader. The external window
    /// watchdog (U17) is separate; see `safe_state`.
    watchdog: &'static samv71q21b::wdt::Wdt,
    /// Console driver.
    console: &'static capsules_core::console::Console<'static>,
    /// Alarm driver for userspace.
    alarm: &'static capsules_core::alarm::AlarmDriver<
        'static,
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<
            'static,
            samv71q21b::tc::Tc<'static>,
        >,
    >,
    /// CAN B (MCAN1), one client of the CAN mux.
    can: &'static capsules_extra::can::CanCapsule<
        'static,
        capsules_core::virtualizers::virtual_can::CanDevice<'static, mcan::Mcan>,
    >,
    /// SD card, read-only (HSMCI slot A).
    sdmmc: &'static capsules_extra::sdmmc::SdmmcDriver<'static, SdHost>,
    /// Reboot-into-bootloader driver, restricted to the signed UDS app.
    bl_reboot: &'static bl_reboot::BootloaderReboot<'static>,
    /// Scheduler.
    scheduler: &'static RoundRobinSched<'static>,
    /// SysTick for preemptive scheduling.
    systick: cortexm7::systick::SysTick,
}

impl SyscallDriverLookup for Chameleon {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules_extra::can::DRIVER_NUM => f(Some(self.can)),
            capsules_extra::sdmmc::DRIVER_NUM => f(Some(self.sdmmc)),
            bl_reboot::DRIVER_NUM => f(Some(self.bl_reboot)),
            _ => f(None),
        }
    }
}

impl KernelResources<ChipHw> for Chameleon {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm7::systick::SysTick;
    type WatchDog = samv71q21b::wdt::Wdt;
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup { self }
    fn syscall_filter(&self) -> &Self::SyscallFilter { &() }
    fn process_fault(&self) -> &Self::ProcessFault { &() }
    fn scheduler(&self) -> &Self::Scheduler { self.scheduler }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer { &self.systick }
    fn watchdog(&self) -> &Self::WatchDog { self.watchdog }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback { &() }
}

// ---------------------------------------------------------------------------
// Pin setup
// ---------------------------------------------------------------------------

/// Make `pin` a push-pull output at `high`, without its reset-default pull-up.
/// The level is set first so the pin never glitches through the other state.
fn drive(pin: &GPIOPin, high: bool) {
    if high {
        pin.set();
    } else {
        pin.clear();
    }
    pin.enable();
    pin.enable_output();
    pin.disable_pull_up();
}

/// Put every pin that controls something outside the MCU in a known state.
/// Runs before the clock setup, which can take 62 ms on the crystal alone.
fn safe_state(p: &Atsamv71q21bDefaultPeripherals) {
    // Keep ourselves powered once CAN or IGN has woken us.
    drive(p.pc.pin(1), true); // MAIN_HOLD

    // U17 is a window watchdog and SET0 = SET1 = 0 (R109, R110) leaves it
    // running from power-up, wanting a WDI falling edge 10.35-92.7 ms after
    // the last, or WDO cycles the +3V3 buck. SET0 = 1, SET1 = 0 disables it.
    // It stays disabled until something feeds it at a paced interval: the
    // kernel's WatchDog::tickle runs every main-loop pass, which would break
    // the lower bound of the window.
    drive(p.pc.pin(17), true); // WD_SET0
    drive(p.pc.pin(18), false); // WD_SET1
    drive(p.pc.pin(16), false); // WDI: must not float while disabled

    // Everything that drives a load stays off.
    drive(p.pc.pin(19), false); // VEXT_EN
    for pin in [10, 11, 13] {
        drive(p.pa.pin(pin), false); // HSO-DIAG-EN, HSO0-uC, HSO1-uC
    }
    for pin in [0, 2, 4, 6, 24, 25, 26, 27] {
        drive(p.pd.pin(pin), false); // LSO0-3 ENABLE and IN
    }

    // Transceivers. LIN stays disabled, as nothing here drives LIN yet.
    drive(p.pa.pin(22), false); // LINA EN
    drive(p.pd.pin(17), false); // LINB EN
    drive(p.pc.pin(11), false); // CANB_SILENT: normal mode
    drive(p.pd.pin(19), true); // CANA_EN and ~CANA_STB: normal mode, and
    drive(p.pd.pin(20), true); // PB2 is left recessive by its pull-up

    drive(p.pa.pin(24), false); // SD_PWR_EN: card off until set up below
    drive(p.pa.pin(5), true); // EEPROM ~WC: write-protected
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Main kernel entry point.
#[no_mangle]
pub unsafe fn main() {
    // PMC PCER0: PIOA(10), PIOB(11), PIOC(12), PIOD(16), PIOE(17).
    core::ptr::write_volatile(
        0x400E_0610 as *mut u32,
        (1u32 << 10) | (1u32 << 11) | (1u32 << 12) | (1u32 << 16) | (1u32 << 17),
    );

    // Flash wait states for 150 MHz (FWS=6). CLOE intentionally OFF.
    core::ptr::write_volatile(0x400E_0C00 as *mut u32, 0x0000_0600);

    ChipHw::init();

    // Deferred call state (must be early, before any component that uses it).
    kernel::deferred_call::initialize_deferred_call_state_unsafe::<
        cortexm7::thread_id::CortexMThreadIdProvider,
    >();

    let mcan1_msg_ram = static_init!(mcan::MessageRam, mcan::MessageRam::new());

    let peripherals = static_init!(
        Atsamv71q21bDefaultPeripherals,
        Atsamv71q21bDefaultPeripherals::new(mcan1_msg_ram)
    );

    safe_state(peripherals);

    pmc::PMC.setup_clocks(CRYSTAL_HZ);

    peripherals.efc.init();

    pmc::PMC.enable_peripheral_clock(samv71q21b::tc::TC0_CH0_PID);

    // MCAN1: enable peripheral clock + PCK5 as CAN core clock.
    // SAMV71 MCAN uses PCK5 (not GCLK). PCK5 = PLLA / (14+1) = 20 MHz.
    pmc::PMC.enable_peripheral_clock(mcan::MCAN1_PID);
    pmc::PMC.configure_pck(5, 2, 14); // PCK5: CSS=2 (PLLA 300 MHz), PRES=14 → 20 MHz

    // Set MCAN DMA base address (CCFG_CAN0 in Matrix); see sam_v71_xult.
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

    let chip = static_init!(ChipHw, Atsamv71q21b::new(peripherals));
    CHIP = Some(chip);

    // NVIC: enable only the interrupts we handle.
    cortexm7::nvic::Nvic::new(6).enable();  // EFC
    cortexm7::nvic::Nvic::new(23).enable(); // TC0_CH0
    cortexm7::nvic::Nvic::new(mcan::MCAN1_PID).enable(); // MCAN1 INT0
    cortexm7::nvic::Nvic::new(38).enable();              // MCAN1 INT1

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
        create_capability!(capabilities::MemoryAllocationCapability),
    )
    .finalize(components::alarm_component_static!(samv71q21b::tc::Tc));

    // -----------------------------------------------------------------------
    // Console (RTT only)
    // -----------------------------------------------------------------------
    let rtt_memory = components::segger_rtt::SeggerRttMemoryComponent::new()
        .finalize(components::segger_rtt_memory_component_static!());
    RTT_MEMORY = Some(core::ptr::from_ref(&*rtt_memory.rtt_memory));
    let rtt = components::segger_rtt::SeggerRttComponent::new(mux_alarm, rtt_memory)
        .finalize(components::segger_rtt_component_static!(samv71q21b::tc::Tc));

    let uart_mux = components::console::UartMuxComponent::new(rtt, 115_200)
        .finalize(components::uart_mux_component_static!());

    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
        create_capability!(capabilities::MemoryAllocationCapability),
    )
    .finalize(components::console_component_static!());

    components::debug_writer::DebugWriterComponent::new::<
        cortexm7::thread_id::CortexMThreadIdProvider,
    >(
        uart_mux,
        create_capability!(capabilities::SetDebugWriterCapability),
    )
    .finalize(components::debug_writer_component_static!());

    // -----------------------------------------------------------------------
    // SD card (HSMCI slot A), read-only
    // -----------------------------------------------------------------------
    // PA25 = MCCK (Peripheral D); PA28 = MCCDA, PA30/PA31/PA26/PA27 = MCDA0-3
    // (Peripheral C). CMD and DAT have 10k pull-ups on +3V3_SD, so the internal
    // ones go off: they would feed the rail while the card is switched off.
    // PA29 = card detect, active low, R93 to +3V3. Powering the card here,
    // long before an app can initialize it, leaves the LDO time to settle.
    pmc::PMC.enable_peripheral_clock(samv71q21b::hsmci::HSMCI_PID);
    peripherals.pa.pin(25).select_peripheral(PeripheralFunction::D);
    peripherals.pa.pin(25).disable_pull_up();
    for pin in [26, 27, 28, 30, 31] {
        peripherals.pa.pin(pin).disable_pull_up();
        peripherals.pa.pin(pin).select_peripheral(PeripheralFunction::C);
    }
    let sd_detect = peripherals.pa.pin(29);
    sd_detect.enable();
    sd_detect.disable_output();
    sd_detect.disable_pull_up();
    peripherals.pa.pin(24).set(); // SD_PWR_EN

    let sd_alarm = static_init!(
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, samv71q21b::tc::Tc>,
        capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );
    sd_alarm.setup();
    let hsmci = static_init!(
        SdHost,
        samv71q21b::hsmci::Hsmci::new(sd_alarm, Some(sd_detect))
    );
    hil::time::Alarm::set_alarm_client(sd_alarm, hsmci);
    kernel::deferred_call::DeferredCallClient::register(hsmci);

    let sd_buf = static_init!([u8; 64 * hil::sdmmc::BLOCK_SIZE], [0; 64 * hil::sdmmc::BLOCK_SIZE]);
    let sdmmc = static_init!(
        capsules_extra::sdmmc::SdmmcDriver<'static, SdHost>,
        capsules_extra::sdmmc::SdmmcDriver::new(
            hsmci,
            sd_buf,
            board_kernel.create_grant(
                capsules_extra::sdmmc::DRIVER_NUM,
                &create_capability!(capabilities::MemoryAllocationCapability)
            )
        )
    );
    hil::sdmmc::Sdmmc::set_client(hsmci, sdmmc);

    // -----------------------------------------------------------------------
    // CAN B (MCAN1 @ 500 kbps via TJA1057)
    // -----------------------------------------------------------------------
    // Automatic retransmission must be set while the peripheral is Disabled;
    // see sam_v71_xult for why a scan tool needs it.
    use kernel::hil::can::Configure;
    peripherals
        .mcan1
        .set_automatic_retransmission(true)
        .expect("MCAN1 must be Disabled during board setup");

    let can_mux = components::can::CanMuxComponent::new(&peripherals.mcan1)
        .finalize(components::can_mux_component_static!(mcan::Mcan));

    let can_device = components::can::CanDeviceComponent::new(can_mux)
        .finalize(components::can_device_component_static!(mcan::Mcan));

    peripherals
        .mcan1
        .set_bitrate(500_000)
        .expect("MCAN1 bitrate 500 kbps");
    peripherals
        .mcan1
        .set_operation_mode(kernel::hil::can::OperationMode::Normal)
        .expect("MCAN1 normal mode");

    let can = components::can::CanComponent::new(
        board_kernel,
        capsules_extra::can::DRIVER_NUM,
        can_device,
        create_capability!(capabilities::MemoryAllocationCapability),
    )
    .with_subscriber(can_device)
    .finalize(components::can_component_static!(
        capsules_core::virtualizers::virtual_can::CanDevice<'static, mcan::Mcan>
    ));

    kernel::deferred_call::DeferredCallClient::register(&peripherals.mcan1);

    // -----------------------------------------------------------------------
    // Reboot-into-bootloader capsule
    // -----------------------------------------------------------------------
    let gpbr = static_init!(samv71q21b::gpbr::Gpbr, samv71q21b::gpbr::Gpbr::new());
    let bl_reboot = static_init!(
        bl_reboot::BootloaderReboot<'static>,
        bl_reboot::BootloaderReboot::new(
            gpbr,
            core::num::NonZeroU32::new(credentials::SignedAppIdAssigner::name_id(
                PRIVILEGED_APP_NAME
            ))
            .into(),
        )
    );

    // -----------------------------------------------------------------------
    // Scheduler
    // -----------------------------------------------------------------------
    let scheduler =
        components::sched::round_robin::RoundRobinComponent::new(processes)
            .finalize(components::round_robin_component_static!(NUM_PROCS_USIZE));

    let watchdog = static_init!(samv71q21b::wdt::Wdt, samv71q21b::wdt::Wdt::new());

    let platform = Chameleon {
        watchdog,
        console,
        alarm,
        can,
        sdmmc,
        bl_reboot,
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

    // -----------------------------------------------------------------------
    // Credential checking
    // -----------------------------------------------------------------------
    // Same policy and key as sam_v71_xult: signatures are optional, and all
    // one buys is the identity that unlocks the reboot driver.
    let sha = components::sha::ShaSoftware256Component::new()
        .finalize(components::sha_software_256_component_static!());

    let verifying_key = static_init!(
        [u8; 64],
        [
            0xb9, 0xe2, 0x2e, 0xf0, 0x5f, 0xea, 0xfc, 0xaa,
            0x4b, 0xa5, 0x9b, 0x66, 0xae, 0x93, 0xab, 0x64,
            0x19, 0x63, 0x98, 0x8b, 0x47, 0xe8, 0xb9, 0x66,
            0xad, 0x2d, 0x8c, 0xf3, 0x64, 0xd0, 0x12, 0x61,
            0x39, 0x0f, 0x32, 0x83, 0xaa, 0x01, 0x58, 0xf3,
            0xd5, 0x77, 0x8b, 0xe0, 0x8a, 0x62, 0x96, 0xe2,
            0x74, 0xe3, 0x12, 0x6b, 0x8c, 0x15, 0x53, 0x9a,
            0x3a, 0x2a, 0x08, 0x38, 0x43, 0x96, 0x5c, 0x53,
        ]
    );
    let verifying_keys = static_init!([&'static mut [u8; 64]; 1], [verifying_key]);

    let ecdsa_scratch = static_init!([u8; 64], [0; 64]);
    let ecdsa_verifier = static_init!(
        ecdsa_sw::p256_verifier::EcdsaP256SignatureVerifier<'static>,
        ecdsa_sw::p256_verifier::EcdsaP256SignatureVerifier::new(ecdsa_scratch)
    );
    kernel::deferred_call::DeferredCallClient::register(ecdsa_verifier);

    let verifier_keys =
        components::signature_verify_in_memory_keys::SignatureVerifyInMemoryKeysComponent::new(
            ecdsa_verifier,
            verifying_keys,
        )
        .finalize(
            components::signature_verify_in_memory_keys_component_static!(
                ecdsa_sw::p256_verifier::EcdsaP256SignatureVerifier<'static>,
                1,
                64,
                32,
                64,
            ),
        );

    let signature_policy = components::appid::checker_signature::AppCheckerSignatureComponent::new(
        sha,
        verifier_keys,
        tock_tbf::types::TbfFooterV2CredentialsType::EcdsaNistP256,
    )
    .finalize(components::app_checker_signature_component_static!(
        capsules_extra::signature_verify_in_memory_keys::SignatureVerifyInMemoryKeys<
            'static,
            ecdsa_sw::p256_verifier::EcdsaP256SignatureVerifier<'static>,
            1,
            64,
            32,
            64,
        >,
        capsules_extra::sha256::Sha256Software<'static>,
        32,
        64,
    ));

    let checking_policy = static_init!(
        credentials::OptionalCredentials<'static>,
        credentials::OptionalCredentials::new(signature_policy)
    );
    checking_policy.setup();

    let assigner = static_init!(
        credentials::SignedAppIdAssigner,
        credentials::SignedAppIdAssigner::new()
    );

    let checker = components::appid::checker::ProcessCheckerMachineComponent::new(checking_policy)
        .finalize(components::process_checker_machine_component_static!());

    let storage_permissions_policy =
        components::storage_permissions::null::StoragePermissionsNullComponent::new().finalize(
            components::storage_permissions_null_component_static!(
                ChipHw,
                kernel::process::ProcessStandardDebugFull,
            ),
        );

    static FAULT_RESPONSE: capsules_system::process_policies::PanicFaultPolicy =
        capsules_system::process_policies::PanicFaultPolicy {};

    let app_flash = core::slice::from_raw_parts(
        addr_of!(_sapps),
        addr_of!(_eapps) as usize - addr_of!(_sapps) as usize,
    );
    let app_memory = core::slice::from_raw_parts_mut(
        addr_of_mut!(_sappmem),
        addr_of!(_eappmem) as usize - addr_of!(_sappmem) as usize,
    );

    let _loader = components::loader::sequential::ProcessLoaderSequentialComponent::new(
        checker,
        board_kernel,
        chip,
        &FAULT_RESPONSE,
        assigner,
        storage_permissions_policy,
        app_flash,
        app_memory,
        create_capability!(capabilities::ProcessManagementCapability),
    )
    .finalize(components::process_loader_sequential_component_static!(
        ChipHw,
        kernel::process::ProcessStandardDebugFull,
        NUM_PROCS_USIZE
    ));

    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    // Tell the bootloader this kernel is healthy; see sam_v71_xult. GPBR6
    // mirrors BOOT_ATTEMPT_INDEX in `bootloader_entry_gpbr.rs`.
    gpbr.set(samv71q21b::gpbr::GpbrIndex::Gpbr6, 0);

    board_kernel.kernel_loop(
        &platform,
        chip,
        None::<&kernel::ipc::IPC<{ NUM_PROCS }>>,
        &main_loop_capability,
    );
}

#[cfg(not(test))]
#[panic_handler]
/// Panic handler: the message and fault registers go to RTT, then halt.
///
/// `write_sync` waits for room in the ring, so with a J-Link reading it this
/// shows everything; with none attached it stops when the ring fills, which
/// is a halt either way.
pub unsafe fn panic_fmt(pi: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    struct SyncRtt(&'static SeggerRttMemory<'static>);
    impl Write for SyncRtt {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            self.0.write_sync(s.as_bytes());
            Ok(())
        }
    }

    if let Some(memory) = RTT_MEMORY {
        let mut rtt = SyncRtt(&*memory);
        let _ = write!(rtt, "\r\n\r\n!!! KERNEL PANIC !!!\r\n{}\r\n", pi);

        // [CCR, CFSR, HFSR, MMFAR, BFAR], saved when a process faulted.
        let scb = addr_of!(cortexm::syscall::SCB_REGISTERS) as *const [u32; 5];
        let _ = write!(
            rtt,
            "CFSR=0x{:08X} HFSR=0x{:08X} MMFAR=0x{:08X} BFAR=0x{:08X}\r\n",
            (*scb)[1], (*scb)[2], (*scb)[3], (*scb)[4],
        );
        let _ = write!(rtt, "!!! HALTED !!!\r\n");
    }

    loop {}
}
