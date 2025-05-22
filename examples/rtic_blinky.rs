#![no_std]
#![no_main]

// RTIC monotonics
use rtic_monotonics::rp235x::prelude::*;
rp235x_timer_monotonic!(Mono);

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;
use hal::{
    clocks,
    gpio::{
        self,
        bank0::Gpio25,
        FunctionSio, PullNone, SioOutput,
    },
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Some things we need
use embedded_hal::digital::OutputPin;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Type aliases
type Led = gpio::Pin<Gpio25, FunctionSio<SioOutput>, PullNone>;

#[rtic::app(device = pac)]
mod app {
    // bring in all use statements
    use super::*;

    // shared variables
    #[shared]
    struct Shared {}

    // local variables
    #[local]
    struct Local {
        // do a single LED for now
        led: Led,
    }

    #[init]
    fn init (mut ctx: init::Context) -> (Shared, Local) {
        // configure clocks, watchdog and monotonic
        Mono::start(ctx.device.TIMER0, &mut ctx.device.RESETS);
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let _clocks = clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // single-cycle I/O glock contros our GPIO pins
        let sio = Sio::new(ctx.device.SIO);

        // initiaize pins to their default state
        let pins = gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // configure GPIO25 as an output pin (for the LED)
        let mut led = pins.gpio25
                .into_pull_type::<PullNone>()
                .into_push_pull_output();
        led.set_low().unwrap(); // initial state

        // spawn heartbeat task (blinky)
        heartbeat::spawn().ok();

        // return resources and timer
        (Shared {}, Local {led})

    }

    #[task(local = [led])]
    async fn heartbeat(ctx: heartbeat::Context) {
        // toggle led
        loop {
            ctx.local.led.set_high().unwrap();
            Mono::delay(1000.millis()).await;
            ctx.local.led.set_low().unwrap();
            Mono::delay(1000.millis()).await;
        }
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"RTIC Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
