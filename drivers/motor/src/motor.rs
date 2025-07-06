//!
//! Driver for Lil-G's motor system
//!

use core::convert::Infallible;
use core::f32::consts::PI;

// embedded hal traits
use embedded_hal::digital::InputPin;
use embedded_hal::pwm::SetDutyCycle;

use rp235x_hal::gpio::AnyPin;
// rp235x structs and traits
// use defmt::info;
use rp235x_hal::pwm::{Slice, SliceId, ValidSliceMode};
use rp235x_hal::pwm::{ValidPwmOutputPin, A, B};

// constants
pub const TICKS_PER_ROTATION: f32 = 12.0;   // encoder ticks
pub const WHEEL_DIAMETER: f32 = 6.0;        // 60 mm => 6 cm
pub const MOTOR_GEAR_RATIO: f32 = 29.86;    // pololu motors gr

const SPEED_HISTORY_LENGTH: usize = 2;  // number of recorded speed samples
const CORRECTION_FACTOR_DIFF: f32 = 1.5;
const CORRECTION_OFFSET: u16 = 10;
const MAX_ALLOWED_DUTY: u16 = 40_000;

// enum
#[derive(Clone, Copy)]
pub enum WheelRotDir {
    CW,
    CCW
}

#[derive(Clone, Copy)]
pub(crate) enum EncState {
    A,
    B,
    C,
    D,
}

impl EncState {
    pub(crate) fn tick_lut(&self, new_state: &EncState) -> i64 {
        match self {
            EncState::A => {
                match new_state {
                    EncState::A => 0,
                    EncState::B => -1,
                    EncState::C => 0,
                    EncState::D => 1,
                }
            },
            EncState::B => {
                match new_state {
                    EncState::A => 1,
                    EncState::B => 0,
                    EncState::C => -1,
                    EncState::D => 0,
                }
            },
            EncState::D => {
                match new_state {
                    EncState::A => 0,
                    EncState::B => 1,
                    EncState::C => 0,
                    EncState::D => -1,
                }
            },
            EncState::C => {
                match new_state {
                    EncState::A => -1,
                    EncState::B => 0,
                    EncState::C => 1,
                    EncState::D => 0,
                }
            },
        }
    }
}

pub(crate) fn to_state(a: bool, b: bool) -> EncState {
    if a {
        if b {
            EncState::C
        } else {
            EncState::D
        }
    } else {
        if b {
            EncState::B
        } else {
            EncState::A
        }
    }
}

pub(crate) fn get_correction_factor(curr_speed:&f32, new_speed: &f32) -> f32 {
    // correction factor will be proportional to the difference in speed magnitude
    let mut difference = curr_speed - new_speed;
    if difference < 0.0 {
        difference = -difference;
    }

    // correction factor returns  correction if our difference is 1 cm/s
    // consider changing this function to be non-linear
    let mut correction = difference * CORRECTION_FACTOR_DIFF;
    if correction > 50.0 {
        correction = 50.0; // never be greater than 100%
    }
    correction
}

pub(crate) fn get_weighted_speed(history: &[f32;SPEED_HISTORY_LENGTH], curr_speed:&f32) -> f32 {
    // takes into account the history of the speed and averages it
    // alongside the new calculated speed
    (curr_speed * 0.6) + (history[1] * 0.25) + (history[0] * 0.15)
}

pub(crate) fn get_updated_duty(curr: u16, correction: f32) -> u16 {
    // This is the updated duty function: for now we'll use a linear function
    // of the form ax + b. Where, a = 1 +/- correction factor and b is some linear offset

    let scaled_duty = curr as f32 * correction;
    let mut new_duty = scaled_duty as u16 + CORRECTION_OFFSET;

    if new_duty > MAX_ALLOWED_DUTY {
        new_duty = MAX_ALLOWED_DUTY;
    }

    new_duty
}

///
/// Structure that represents an instance of a Motor Driver.
///
/// For Allegro's A4950E DMOS full-bridge brushed DC motor driver IC.
///
pub struct Motor <I, M, E1, E2>
where
    I: SliceId,             // channel ID
    M: ValidSliceMode<I>,   // pwm channel
    E1: InputPin,           // input pin
    E2: InputPin,           // input pin
{
    // two PWM channels (A and B)
    pwm: Slice<I, M>,

    // two encoders input pins
    pub enc_a: E1,
    pub enc_b: E2,

    // encoder state
    enc_state: EncState,

    // rotations counter
    enc_ticks: i64, // a lot of ticks :)

    // speed setpoint
    //      The motor handles it's own speed setpoint by using encoder readings
    //      the frequency/response time is determined in main
    speed_setpoint: f32,

    // bookeeping
    duty_cycle: u16,    // curr duty cycle
    dir: WheelRotDir,   // curr direction
    speed_history: [f32; SPEED_HISTORY_LENGTH],


}

impl <I, M, E1, E2> Motor<I, M, E1, E2>
where
    I: SliceId,         // pwm channel
    M: ValidSliceMode<I>,  // pwm channel
    E1: InputPin,        // input pin
    E2: InputPin,        // input pin
{
    /// Create a motor driver instance and configure pins accordingly
    pub fn new(
        pwm: Slice<I, M>,
        enc_a: E1, enc_b: E2,
    ) -> Self {
        // Motor instance
        Motor {
            pwm, enc_a, enc_b,
            enc_state: EncState::A, // initialize some random state
            enc_ticks: 0,           // initial ticks is 0
            speed_setpoint: 0.0,   // initial speed setpoint is 0
            duty_cycle: 0,
            dir: WheelRotDir::CW,
            speed_history: [0.0; SPEED_HISTORY_LENGTH],
        }
    }

    /// Configures the motor to the right state and perihperal values
    pub fn configure <P1: AnyPin, P2: AnyPin> (mut self, pin_a: P1, pin_b: P2) -> Self
    where
        P1::Id: ValidPwmOutputPin<I, A>,
        P2::Id: ValidPwmOutputPin<I, B>,
    {
        // initialize encoder pins and state
        let (a, b) = self.read_encoder();
        self.enc_state = to_state(a, b);

        // just some debug prints
        // info!("Max Allowed Duty A: {}", self.pwm_a.max_duty_cycle());
        // info!("Max Allowed Duty B: {}", self.pwm_b.max_duty_cycle());

        // setup pwm slices
        self.pwm.set_ph_correct();
        self.pwm.enable();
        self.pwm.channel_a.output_to(pin_a);
        self.pwm.channel_b.output_to(pin_b);
        self.pwm.channel_a.set_inverted();
        self.pwm.channel_b.set_inverted();

        // this is cool
        self
    }

    /// Assign motor speed
    /// params:
    ///     - duty: speed as percentage [0:100] of max duty
    ///     - direction: CW or CCW encoded as a a bool
    /// note:
    ///     direction is determined by value of direction:
    ///     - true = CW
    ///     - false = CCW
    pub fn set_duty_percent(&mut self, duty_percent: u16, direction: WheelRotDir) -> Result<(), Infallible>{
        // we first scale the passed in duty (as percentage) based on the max duty
        let max_duty_a = self.pwm.channel_a.max_duty_cycle() as f32;
        let max_duty_b = self.pwm.channel_b.max_duty_cycle() as f32;

        // now assign the duty to each channel based on direction
        match direction {
            WheelRotDir::CW => {
                // scale duty for A
                let scaled_duty = ((duty_percent as f32 / 100f32) * max_duty_a as f32) as u16;

                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm.channel_a.set_duty_cycle(scaled_duty).unwrap();    // rp235x-hal always returns Ok(())
                self.pwm.channel_b.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
            },
            WheelRotDir::CCW => {
                // scale duty for B
                let scaled_duty = ((duty_percent as f32 / 100f32) * max_duty_b as f32) as u16;

                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm.channel_a.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
                self.pwm.channel_b.set_duty_cycle(scaled_duty).unwrap();    // rp235x-hal always returns Ok(())
            },
        }

        Ok(())
    }

    pub fn set_duty(&mut self, duty: u16, direction: WheelRotDir) -> Result<(), Infallible>{
        // update duty
        self.duty_cycle = duty;
        self.dir = direction;

        // now assign the duty to each channel based on direction
        match direction {
            WheelRotDir::CW => {
                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm.channel_a.set_duty_cycle(duty).unwrap();    // rp235x-hal always returns Ok(())
                self.pwm.channel_b.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
            },
            WheelRotDir::CCW => {
                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm.channel_a.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
                self.pwm.channel_b.set_duty_cycle(duty).unwrap();    // rp235x-hal always returns Ok(())
            },
        }

        Ok(())
    }

    pub fn stop(&mut self) {
        self.pwm.channel_a.set_duty_cycle(0).unwrap();
        self.pwm.channel_b.set_duty_cycle(0).unwrap();
    }

    // calibration routine to determine the best speed to duty factor
    pub fn calibrate(&mut self) {
        // 1. run wheels at multiple speed targets
        // 2. use intial conversion estimate
        //      - fixed
        //      - random init
        //      - contrained random init
        // 3. obtain encoder readings (in cm/s)
        // 4. calculate loss
        //      - squared mean loss
        //      - some other loss function?
        // 5. adjust conversion constant based on loss
        // 6. repeat until convergence or timeout
        //      - number of cycles (arg vs fixed)
        //
        // For now we can model the conversion as:
        //      (A * x) + B
        //  A: speed_to_duty_factor
        //  B: speed_to_duty_offset
    }

    // speed to duty converts (cm/s) into duty cycles
    // this function uses pre-calibrated constants
    fn speed_to_duty(&mut self, speed: f32) -> u16 {
        0
    }

    fn duty_to_speed(&mut self, duty: u16) -> f32 {
        0f32
    }

    pub fn set_speed_setpoint(&mut self, speed: f32) {
        self.speed_setpoint = speed;
        if speed < 0.0 {
            self.dir = WheelRotDir::CCW;
            self.duty_cycle = ((-1.0 * speed) as u16) * 1_000;
        } else {
            self.dir = WheelRotDir::CW;
            self.duty_cycle = speed as u16 * 1_000;
        }
        // initial guess
        self.set_duty(self.duty_cycle, self.dir).unwrap();
    }

    pub fn update_duty_to_setpoint(&mut self, delta: f32) -> f32{
        // get motor speed (enable lpf)
        let weighted_speed = self.get_encoder_speed(delta, true);

        // info!("Current Speed: {} | Setpoint {}", weighted_speed, self.speed_setpoint);

        // get correction as percentage and convert to decimal
        let mut correction = get_correction_factor(&weighted_speed, &self.speed_setpoint) / 100.0;

        // update duty depending on curr_speed - setpoint
        match self.dir {
            WheelRotDir::CCW => {
                if weighted_speed < self.speed_setpoint {
                    correction = 1.0 - correction;
                    self.set_duty(get_updated_duty(self.duty_cycle, correction), self.dir).unwrap();
                } else if weighted_speed > self.speed_setpoint {
                    correction = 1.0 + correction;
                    self.set_duty(get_updated_duty(self.duty_cycle, correction), self.dir).unwrap();
                }
            },
            WheelRotDir::CW => {
                if weighted_speed > self.speed_setpoint {
                    correction = 1.0 - correction;
                    self.set_duty(get_updated_duty(self.duty_cycle, correction), self.dir).unwrap();
                } else if weighted_speed < self.speed_setpoint {
                    correction = 1.0 + correction;
                    self.set_duty(get_updated_duty(self.duty_cycle, correction), self.dir).unwrap();
                }
            },
        };

        // buffer for low pass filter
        self.push_speed_history(&weighted_speed);

        weighted_speed
    }

    /// Helper function that reads and returns the encoder values as a tuple
    /// Return: (bool, bool) rerpresenting if a is high and b is high
    pub fn read_encoder (&mut self) -> (bool, bool) {
        // read the encoder values
        let read_a = self.enc_a.is_high().unwrap();
        let read_b = self.enc_b.is_high().unwrap();

        // info!("EncA: {}", read_a);
        // info!("EncB: {}", read_b);

        // return as tuple
        (read_a, read_b)
    }

    /// Updates encoder tick count by using a LUT to determine +/- enc tick value
    pub fn update_encoder_tick(&mut self) {
        // get new state by reading encoders
        let (a, b) = self.read_encoder();
        let new_state = to_state(a, b);

        // determine +/- tick count using LUT
        self.enc_ticks += self.enc_state.tick_lut(&new_state);

        // update state
        self.enc_state = new_state;
    }

    /// Read encoder ticks and resets if asserterd
    pub fn get_encoder_ticks(&mut self, reset: bool) -> i64 {
        let ticks = self.enc_ticks;
        if reset {
            self.enc_ticks = 0;
        }
        ticks
    }

    pub fn reset_encoder_ticks(&mut self) {
        self.enc_ticks = 0;
    }

    /// Converts encoder readings into speed
    /// args:
    ///     delta: time step
    ///     lpf: low-pass filter enable
    pub fn get_encoder_speed(&mut self, delta: f32, lpf: bool) -> f32 {
        // result
        let res: f32;

        // calculate motor's speed
        let rotations = self.get_encoder_ticks(true) as f32 / (TICKS_PER_ROTATION * MOTOR_GEAR_RATIO);
        let rot_per_sec = rotations / delta;
        let speed = rot_per_sec * 2.0 * PI * WHEEL_DIAMETER;

        if lpf {
            res = get_weighted_speed(&self.speed_history, &speed);
        } else {
            res = speed;
        }

        res
    }

    pub(crate) fn push_speed_history(&mut self, new_speed: &f32) {
        // push the new value into the history and overwrite the oldest value
        for i in 0..(SPEED_HISTORY_LENGTH-2) {
            self.speed_history[i] = self.speed_history[i + 1];
        }

        self.speed_history[SPEED_HISTORY_LENGTH - 1] = *new_speed;
    }
}
