//! # PWM Blink Example
//!
//! If you have an LED connected to pin 25, it will fade the LED using the PWM
//! peripheral.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
// use embedded_hal::digital::{InputPin, OutputPin};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 8_000;

/// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 20_000;

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then fades the LED in an
/// infinite loop.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM0
    let pwm0 = &mut pwm_slices.pwm0;
    pwm0.set_ph_correct();
    pwm0.enable();

    // configure PWM4
    let pwm4 = &mut pwm_slices.pwm4;
    pwm4.set_ph_correct();
    pwm4.enable();

    // motor has two pwm channels :)
    let m1_a = &mut pwm0.channel_b;
    let m1_b = &mut pwm0.channel_a;
    m1_a.output_to(pins.gpio17);
    m1_b.output_to(pins.gpio16);

    // invert the outputs!
    m1_a.set_inverted();
    m1_b.set_inverted();

    // led is on pwm4 channel b on gpio 25
    let led = &mut pwm4.channel_b;
    led.output_to(pins.gpio25);

    // Infinite loop, fading LED up and down
    loop {
        // Ramp brightness up
        for i in LOW..=HIGH {
            delay.delay_us(200);
            let _ = led.set_duty_cycle(i);
            let _ = m1_b.set_duty_cycle(0); // equivlent to output high
            let _ = m1_a.set_duty_cycle(i);
        }

        // Ramp brightness down
        for i in (LOW..=HIGH).rev() {
            delay.delay_us(200);
            let _ = led.set_duty_cycle(i);
            let _ = m1_a.set_duty_cycle(0); // equivlent to output high
            let _ = m1_b.set_duty_cycle(i);
        }
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"PWM Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
