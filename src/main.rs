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
use rp235x_hal::pwm::{self, Channel, FreeRunning, Pwm4, Pwm5, Slice};

// import local drivers
use motor_driver::*;
use motor::Motor;
use rp235x_pkg::*;
use motion_control::motion;
use motion::{Motion, RotDirection, MoveDirection};
mod status;
use status::Status;

// IMU driver
// use icm20948_driver::icm20948;
use lsm6dso::Lsm6dso;
use nalgebra::base::*;

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
        // shared timer
        timer: MotionTimer,
    }

    // local variables
    #[local]
    struct Local {
        // do a single LED for now
        status: Status,
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
        info!("Initializing...");

        // timer 1
        let mut timer = hal::Timer::new_timer1(ctx.device.TIMER1, &mut ctx.device.RESETS, &clocks);
        timer.delay_ms(200);

        ///////////////
        // IMU Setup //
        ///////////////
        // I2C
        let sda_pin = pins.gpio2.reconfigure();
        let scl_pin = pins.gpio3.reconfigure();
        let i2c = hal::I2C::i2c1(
            ctx.device.I2C1,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut ctx.device.RESETS,
            &clocks.system_clock,
        );

        // imu instance
        let mut imu = Lsm6dso::new(i2c, 0x6A);

        // acceleromter setup
        imu.set_accelerometer_output(lsm6dso::AccelerometerOutput::Rate833).unwrap();
        imu.set_accelerometer_scale(lsm6dso::AccelerometerScale::G02).unwrap();
        imu.set_accelerometer_low_pass(Some(lsm6dso::Bandwidth::OdrDiv10)).unwrap();

        // gyro setup
        imu.set_gyroscope_output(lsm6dso::GyroscopeOutput::Rate833).unwrap();
        imu.set_gyroscope_scale(lsm6dso::GyroscopeFullScale::Dps125).unwrap();

        // make sure we can communicate with IMU
        match imu.check() {
            Ok(_) => info!("[INFO] IMU Initialized correctly"),
            Err(_) => info!("[ERROR] Cannot initialized IMU")
        };

        /////////////////
        // Status LEDs //
        /////////////////
        let led_1 = pins.gpio11.into_push_pull_output();
        let led_2 = pins.gpio10.into_push_pull_output();
        let led_3 = pins.gpio9.into_push_pull_output();
        let led_4 = pins.gpio8.into_push_pull_output();
        let mut status = Status::new(led_1, led_2, led_3, led_4);
        status.reset_all();

        //////////////////////////////
        // Configure Motor PWM Pins //
        //////////////////////////////
        // Init PWMs
        let pwm_slices = hal::pwm::Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);
        // Motor 1
        let m1_pwm = pwm_slices.pwm4;
        let m1_pin_a = pins.gpio24;
        let m1_pin_b = pins.gpio25;

        // Motor 2
        let m2_pwm = pwm_slices.pwm2;
        let m2_pin_a = pins.gpio20;
        let m2_pin_b = pins.gpio21;

        // Motor 3
        let m3_pwm = pwm_slices.pwm0;
        let m3_pin_a = pins.gpio16;
        let m3_pin_b = pins.gpio17;

        // Motor 4
        let m4_pwm = pwm_slices.pwm6;
        let m4_pin_a = pins.gpio12;
        let m4_pin_b = pins.gpio13;

        //////////////////////////////////
        // Configure Motor Encoder Pins //
        //////////////////////////////////
        // Motor 1
        let m1_enc_a = pins.gpio26.into_pull_down_input();
        let m1_enc_b = pins.gpio27.into_pull_down_input();
        m1_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m1_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m1_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m1_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // Motor 2
        let m2_enc_a = pins.gpio22.into_pull_down_input();
        let m2_enc_b = pins.gpio23.into_pull_down_input();
        m2_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m2_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m2_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m2_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // Motor 3
        let m3_enc_a = pins.gpio18.into_pull_down_input();
        let m3_enc_b = pins.gpio19.into_pull_down_input();
        m3_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m3_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m3_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m3_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // Motor 4
        let m4_enc_a = pins.gpio14.into_pull_down_input();
        let m4_enc_b = pins.gpio15.into_pull_down_input();
        m4_enc_a.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m4_enc_a.set_interrupt_enabled(Interrupt::EdgeLow, true);
        m4_enc_b.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        m4_enc_b.set_interrupt_enabled(Interrupt::EdgeLow, true);

        // instantiate motors
        let m1 = Motor::new(m1_pwm, m1_enc_a, m1_enc_b).configure(m1_pin_a, m1_pin_b);
        let m2 = Motor::new(m2_pwm, m2_enc_a, m2_enc_b).configure(m2_pin_a, m2_pin_b);
        let m3 = Motor::new(m3_pwm, m3_enc_a, m3_enc_b).configure(m3_pin_a, m3_pin_b);
        let m4 = Motor::new(m4_pwm, m4_enc_a, m4_enc_b).configure(m4_pin_a, m4_pin_b);

        ////////////////////
        // Motion Control //
        ////////////////////
        let mut m = Motion::new(m1, m2, m3, m4, imu, &timer);
        m.configure(); // set some defaults
 
        // spawn heartbeat task (blinky)
        heartbeat::spawn().ok();

        basic_move::spawn().ok();
        // complex_move::spawn().ok();
        setpoint::spawn().ok();
        // printEnc::spawn().ok();
        // imu_test::spawn().ok();

        // return resources and timer
        (Shared {m, timer}, Local {status})

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
    async fn basic_move(ctx: basic_move::Context) {
        let mut m = ctx.shared.m;
        let speed: f32 = 20.0;
        let fast = 30.0;

        // m.lock(|m| {
        //     m.move_in_dir_speed(speed, MoveDirection::Forward);
        // });

        loop {
            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, speed, 0.0));
            });
            Mono::delay(5000.millis()).await;

            // right
            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(speed, 0.0, 0.0));
            });
            Mono::delay(5000.millis()).await;

            // backward
            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, -speed, 0.0));
            });
            Mono::delay(5000.millis()).await;

            // left
            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(-speed, 0.0, 0.0));
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, 0.0, -speed/5.0));
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, 0.0, -fast/5.0));
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, 0.0, speed/5.0));
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, 0.0, fast/5.0));
            });
            Mono::delay(5000.millis()).await;
        }
    }

    #[task(shared = [m], priority = 1)]
    async fn complex_move(ctx: complex_move::Context) {
        let mut m = ctx.shared.m;
        let speed = 20.0f32;

        loop {
            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(speed/2.0, speed/2.0, 0.0));
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(speed, 0.0, speed/10.0));
            });
            Mono::delay(5000.millis()).await;

            m.lock(|m| {
                m.move_body_to_wheel(Vector3::new(0.0, speed/2.0, speed/5.0));
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
                m.test_accel();
                m.test_temp();
            });
            Mono::delay(50.millis()).await;
        }
    }

    #[task(shared = [m, timer], priority = 2)]
    async fn setpoint(ctx: setpoint::Context) {
        let mut m = ctx.shared.m;
        let mut timer = ctx.shared.timer;

        loop {
            // forward
            (&mut m, &mut timer).lock(|m, timer| {
                m.poll_all_setpoints(0.2, &timer);
            });
            Mono::delay(200.millis()).await;
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

    #[task(local = [status], priority = 1)]
    async fn heartbeat(ctx: heartbeat::Context) {
        let status = ctx.local.status;

        // read encoder input and set led vaue accordingly
        loop {
            status.set_all();
            Mono::delay(500.millis()).await;
            status.reset_all();
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
