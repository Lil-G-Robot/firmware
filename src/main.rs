#![no_std]
#![no_main]
#![allow(unused_imports)]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use defmt_rtt as _;
use panic_probe as _;

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
            Gpio20,
            Gpio12,
            Gpio13,
        },
        Interrupt,
        FunctionSio, SioOutput, SioInput,
        PullDown, PullUp, PullNone,
    },
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// temporary will remove after types are moved to respective file
use rp235x_hal::pwm::{self, Channel, FreeRunning, Pwm4, Slice};

// import local drivers
use motor_driver::*;
use motor::Motor;
use rp235x_pkg::*;
use motion_control::motion;
use motion::{Motion, RotDirection, MoveDirection};

// IMU driver
use icm20948_driver::icm20948;

// Some things we need
// use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::digital::{OutputPin, InputPin};
use fugit::RateExtU32;
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

#[rtic::app(device = pac, dispatchers = [UART1_IRQ, UART0_IRQ, SPI0_IRQ, SPI1_IRQ])]
mod app {

    // bring in all use statements
    use super::*;
    use defmt::info;
    use embedded_hal::delay::DelayNs;
    use rp235x_hal::{gpio::{FunctionI2c, Pin}, i2c, pac::I2C0, timer::{CopyableTimer1, Timer}, I2C};

    // shared variables
    #[shared]
    struct Shared {
        // motor instance
        m: Motion,
    }

    // local variables
    #[local]
    struct Local {
        // do a single LED for now
        led: Channel<Slice<Pwm4, FreeRunning>, pwm::B>,
   }

    #[init]
    fn init (mut ctx: init::Context) -> (Shared, Local) {
        // configure clocks, watchdog and monotonic
        Mono::start(ctx.device.TIMER0, &mut ctx.device.RESETS);
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
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

        // Debug print
        info!("Initializing peripherals");

        // timer 1
        let mut timer = hal::Timer::new_timer1(ctx.device.TIMER1, &mut ctx.device.RESETS, &clocks);
        timer.delay_ms(200);

        // I2C
        let sda_pin = pins.gpio12.reconfigure();
        let scl_pin = pins.gpio13.reconfigure();
        let i2c = hal::I2C::i2c0(
            ctx.device.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut ctx.device.RESETS,
            &clocks.system_clock,
        );

        // setting up IMU
        info!("Setting up IMU");
        let mut imu = icm20948::i2c::IcmImu::new(i2c, 0x69).unwrap();

        let wai = imu.wai().unwrap();
        info!("Who Am I?: {}", wai);

        imu.set_gyro_sen(icm20948::GyroSensitivity::Sen500dps).unwrap();
        imu.set_acc_sen(icm20948::AccSensitivity::Sen4g).unwrap();

        // Init PWMs
        let pwm_slices = hal::pwm::Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        // Configure led as PWM 4b
        let mut pwm4 = pwm_slices.pwm4; // take pwm4
        pwm4.set_ph_correct();
        pwm4.enable();
        let mut led = pwm4.channel_b; // take channel_b
        led.output_to(pins.gpio25);

        // Configure Motor PWM Pins //
        // Motor 1
        let mut pwm0 = pwm_slices.pwm0;
        pwm0.set_ph_correct();
        pwm0.enable();
        let mut m1_pwm_a = pwm0.channel_b;
        let mut m1_pwm_b = pwm0.channel_a;
        m1_pwm_a.output_to(pins.gpio17);
        m1_pwm_b.output_to(pins.gpio16);
        m1_pwm_a.set_inverted();
        m1_pwm_b.set_inverted();

        // Motor 2
        let mut pwm5 = pwm_slices.pwm5;
        let mut pwm2 = pwm_slices.pwm2;
        pwm5.set_ph_correct();
        pwm2.set_ph_correct();
        pwm5.enable();
        pwm2.enable();
        let mut m2_pwm_a = pwm2.channel_a;
        let mut m2_pwm_b = pwm5.channel_a;
        m2_pwm_a.output_to(pins.gpio4);
        m2_pwm_b.output_to(pins.gpio26);
        m2_pwm_a.set_inverted();
        m2_pwm_b.set_inverted();

        // Motor 3
        let mut pwm1 = pwm_slices.pwm1;
        let mut pwm7 = pwm_slices.pwm7;
        pwm1.set_ph_correct();
        pwm7.set_ph_correct();
        pwm1.enable();
        pwm7.enable();
        let mut m3_pwm_a = pwm7.channel_a;
        let mut m3_pwm_b = pwm1.channel_b;
        m3_pwm_a.output_to(pins.gpio14);
        m3_pwm_b.output_to(pins.gpio19);
        m3_pwm_a.set_inverted();
        m3_pwm_b.set_inverted();

        // Motor 4
        let mut m4_pwm_a = pwm1.channel_a;
        let mut m4_pwm_b = pwm7.channel_b;
        m4_pwm_a.output_to(pins.gpio18);
        m4_pwm_b.output_to(pins.gpio15);
        m4_pwm_a.set_inverted();
        m4_pwm_b.set_inverted();

        // Configure Motor Encoder Pins //
        // Motor 1
        let m1_enc_a = pins.gpio22.into_pull_down_input();
        let m1_enc_b = pins.gpio21.into_pull_down_input();
        m1_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m1_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m1_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m1_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // Motor 2
        let m2_enc_a = pins.gpio1.into_pull_down_input();
        let m2_enc_b = pins.gpio0.into_pull_down_input();
        m2_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m2_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m2_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m2_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // Motor 3
        let m3_enc_a = pins.gpio2.into_pull_down_input();
        let m3_enc_b = pins.gpio28.into_pull_down_input();
        m3_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m3_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m3_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m3_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // Motor 4
        let m4_enc_a = pins.gpio3.into_pull_down_input();
        let m4_enc_b = pins.gpio27.into_pull_down_input();
        m4_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m4_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m4_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m4_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // instantiate motors
        let m1 = Motor::new(m1_pwm_a, m1_pwm_b, m1_enc_a, m1_enc_b).configure();
        let m2 = Motor::new(m2_pwm_a, m2_pwm_b, m2_enc_a, m2_enc_b).configure();
        let m3 = Motor::new(m3_pwm_a, m3_pwm_b, m3_enc_a, m3_enc_b).configure();
        let m4 = Motor::new(m4_pwm_a, m4_pwm_b, m4_enc_a, m4_enc_b).configure();

        // motion control instance
        let mut m = Motion::new(m1, m2, m3, m4, imu, timer);
        m.configure(); // set some defaults
 
        // spawn heartbeat task (blinky)
        heartbeat::spawn().ok();

        spin::spawn().ok();
        setpoint::spawn().ok();
        // printEnc::spawn().ok();
        // imu_test::spawn().ok();

        // return resources and timer
        (Shared {m}, Local {led})

    }

    #[idle]
    // note that idle is defined as priority 0 by default.
    fn idle(_: idle::Context) -> ! {
        loop {
            hal::arch::wfi(); // low power sleep when idle
        }
    }

    #[task(shared = [m], priority = 2)]
    async fn printEnc(ctx: printEnc::Context) {
        let mut m = ctx.shared.m;

        // read encoder input and set led vaue accordingly
        loop {
            let (mut enc1_t, mut enc2_t, mut enc3_t, mut enc4_t) = (0f32,0f32,0f32,0f32);

            m.lock(|m| {
                let (enc1, enc2, enc3, enc4) = m.get_encoder_ticks();
                (enc1_t, enc2_t, enc3_t, enc4_t) = (enc1 as f32, enc2 as f32, enc3 as f32, enc4 as f32);
                m.reset_encoder_ticks();
            });

            enc1_t = (enc1_t * 6.28 * 6.0) / (12.0 * 0.2);
            enc2_t = (enc2_t * 6.28 * 6.0) / (12.0 * 0.2);
            enc3_t = (enc3_t * 6.28 * 6.0) / (12.0 * 0.2);
            enc4_t = (enc4_t * 6.28 * 6.0) / (12.0 * 0.2);

            info!(
            "Motor 1 Speed (rot/s): {}\n\
                Motor 2 Speed (rot/s): {}\n\
                Motor 3 Speed (rot/s): {}\n\
                Motor 4 Speed (rot/s): {}\
            ", enc1_t, enc2_t, enc3_t, enc4_t);
            Mono::delay(200.millis()).await;
        }
    }

    #[task(shared = [m], priority = 1)]
    async fn spin(ctx: spin::Context) {
        let mut m = ctx.shared.m;
        let speed: f32 = 20.0;
        let fast = 30.0;

        // m.lock(|m| {
        //     m.move_in_dir_speed(speed, MoveDirection::Forward);
        // });

        loop {
            m.lock(|m| {
                m.move_in_dir_speed_diag(speed, MoveDirection::Forward);
            });
            Mono::delay(5000.millis()).await;

            // right
            m.lock(|m| {
                m.move_in_dir_speed_diag(speed, MoveDirection::Right);
            });
            Mono::delay(5000.millis()).await;

            // backward
            m.lock(|m| {
                m.move_in_dir_speed_diag(speed, MoveDirection::Backward);
            });
            Mono::delay(5000.millis()).await;

            // backward
            m.lock(|m| {
                m.move_in_dir_speed_diag(speed, MoveDirection::Left);
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.rotate_in_dir_speed(speed, RotDirection::CW);
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.rotate_in_dir_speed(fast, RotDirection::CW);
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.rotate_in_dir_speed(speed, RotDirection::CCW);
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.rotate_in_dir_speed(fast, RotDirection::CCW);
            });
            Mono::delay(5000.millis()).await;
        }
    }

    #[task(shared = [m], priority = 2)]
    async fn imu_test(ctx: imu_test::Context) {
        let mut m = ctx.shared.m;

        loop {
            m.lock(|m| {
                m.test_gyro();
            });
            Mono::delay(50.millis()).await;
        }
    }

    #[task(shared = [m], priority = 2)]
    async fn setpoint(ctx: setpoint::Context) {
        let mut m = ctx.shared.m;

        loop {
            // forward
            m.lock(|m| {
                m.poll_all_setpoints(0.05);
            });
            Mono::delay(50.millis()).await;
       }
    }

    #[task(binds = IO_IRQ_BANK0, shared = [m], priority = 3)]
    fn encUpdate(ctx: encUpdate::Context) {
        let mut m = ctx.shared.m;

        // debug
        // info!("Toggled Enc Update!");

        m.lock(|m| {
            m.update_encoder_tick_all_motors();
            m.clear_all_encoder_interrupt(); // safety
        });
    }

    #[task(local = [led], priority = 1)]
    async fn heartbeat(ctx: heartbeat::Context) {
        let led = ctx.local.led;

        // read encoder input and set led vaue accordingly
        loop {
            led.set_duty_cycle(led.max_duty_cycle()).unwrap();
            Mono::delay(500.millis()).await;
            led.set_duty_cycle(0).unwrap();
            Mono::delay(500.millis()).await;
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
