#![no_std]
#![no_main]
#![allow(unused_imports)]
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;
use hal::{
    clocks,
    gpio::{
        self,
        bank0::{
            Gpio25, // led
            Gpio0,  // encoder in
            Gpio1,
        },
        Interrupt,
        FunctionSio, SioOutput, SioInput,
        PullDown, PullUp, PullNone,
    },
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Some things we need
// use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::digital::{OutputPin, InputPin};
// use fugit::Duration;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// RTIC monotonics
use rtic_monotonics::rp235x::prelude::*;
rp235x_timer_monotonic!(Mono);

#[rtic::app(device = pac, dispatchers = [UART1_IRQ])]
mod app {
    use rp235x_hal::pwm::{self, Channel, FreeRunning, Pwm4, Slice};

    // bring in all use statements
    use super::*;

    // shared variables
    #[shared]
    struct Shared {}

    // local variables
    #[local]
    struct Local {
        // do a single LED for now
        led: Channel<Slice<Pwm4, FreeRunning>, pwm::B>,
        // led: Slices,
        enc_a: gpio::Pin<Gpio1, FunctionSio<SioInput>, PullDown>,
        enc_b: gpio::Pin<Gpio0, FunctionSio<SioInput>, PullDown>,
        brightness: u32,
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

        // Init PWMs
        let pwm_slices = hal::pwm::Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        // Configure led as PWM 4b
        let mut pwm4 = pwm_slices.pwm4; // take pwm4
        pwm4.set_ph_correct();
        pwm4.enable();
        let mut led = pwm4.channel_b; // take channel_b
        led.output_to(pins.gpio25);

        // configure GPIO0 as input pin for encoder reading
        let enc_a = pins.gpio1.into_pull_down_input();
        let enc_b = pins.gpio0.into_pull_down_input();
        let brightness = 0u32;

        // enable interrupt on rising edge
        enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // spawn heartbeat task (blinky)
        // heartbeat::spawn().ok();

        // return resources and timer
        (Shared {}, Local {led, enc_a, enc_b, brightness})

    }

    #[idle]
    // note that idle is defined as priority 0 by default.
    fn idle(_: idle::Context) -> ! {
        loop {
            hal::arch::wfi(); // low power sleep when idle
        }
    }

    // #[task(local = [led, enc_a, enc_b, brightness], priority = 1)]
    // async fn heartbeat(ctx: heartbeat::Context) {
    //     // let enc_a = ctx.local.enc_a;
    //     // let enc_b = ctx.local.enc_b;
    //     let led = ctx.local.led;
    //
    //     // read encoder input and set led vaue accordingly
    //     loop {
    //         for i in 0..=20_000 {
    //             Mono::delay(10.micros()).await;
    //             led.set_duty_cycle(i).unwrap();
    //         }
    //         for i in (0..=20_000).rev() {
    //             Mono::delay(10.micros()).await;
    //             led.set_duty_cycle(i).unwrap();
    //         }
    //         Mono::delay(500.millis()).await;
    //     }
    // }

    #[task(binds = IO_IRQ_BANK0, local = [led, enc_b], priority = 1)]
    fn heartbeat(ctx: heartbeat::Context) {
        let enc = ctx.local.enc_b;
        let led = ctx.local.led;

        // read encoder input and set led vaue accordingly
        if enc.is_high().unwrap() {
            led.set_duty_cycle(led.max_duty_cycle()).unwrap();
        } else {
            led.set_duty_cycle(0).unwrap();
        }

        // clear interrupt flag
        enc.clear_interrupt(Interrupt::EdgeHigh);
        enc.clear_interrupt(Interrupt::EdgeLow);
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
