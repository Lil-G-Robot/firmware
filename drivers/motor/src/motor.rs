//!
//! Driver for Lil-G's motor system
//!

use core::convert::Infallible;
use core::f32::consts::PI;

// embedded hal traits
use embedded_hal::digital::InputPin;
use embedded_hal::pwm::SetDutyCycle;

// rp235x structs and traits
use defmt::info;

// constants
const TICKS_PER_ROTATION: f32 = 12.0;
const WHEEL_DIAMETER: f32 = 6.0; // 60 mm => 6 cm
const MOTOR_GEAR_RATIO: f32 = 29.86;
const SPEED_HISTORY_LENGTH: usize = 2;
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

    // correction factor returns 10& correction if our difference is 1 cm/s
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
/// Based on Allegro's A4950E DMOS full-bridge brushed DC motor driver IC.
///
pub struct Motor <C1, C2, E1, E2>
where
    C1: SetDutyCycle,    // pwm channel
    C2: SetDutyCycle,    // pwm channel
    E1: InputPin,        // input pin
    E2: InputPin,        // input pin
{
    // two PWM channels (A and B)
    pwm_a: C1,
    pwm_b: C2,

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
    duty_cycle: u16,
    dir: WheelRotDir,
    speed_history: [f32; SPEED_HISTORY_LENGTH],

}

impl <C1, C2, E1, E2> Motor<C1, C2, E1, E2>
where
    C1: SetDutyCycle,    // pwm channel
    C2: SetDutyCycle,    // pwm channel
    E1: InputPin,        // input pin
    E2: InputPin,        // input pin
{
    /// Create a motor driver instance and configure pins accordingly
    pub fn new(
        pwm_a: C1, pwm_b: C2,
        enc_a: E1, enc_b: E2,
    ) -> Self {
        // Motor instance
        Motor {
            pwm_a, pwm_b, enc_a, enc_b,
            enc_state: EncState::A, // initialize some random state
            enc_ticks: 0,           // initial ticks is 0
            speed_setpoint: 0.0,   // initial speed setpoint is 0
            duty_cycle: 0,
            dir: WheelRotDir::CW,
            speed_history: [0.0; SPEED_HISTORY_LENGTH],
        }
    }

    /// Configures the motor to the right state and perihperal values
    pub fn configure (mut self) -> Self {
        // initialize encoder state
        let (a, b) = self.read_encoder();
        self.enc_state = to_state(a, b);

        // just some debug prints
        // info!("Max Allowed Duty A: {}", self.pwm_a.max_duty_cycle());
        // info!("Max Allowed Duty B: {}", self.pwm_b.max_duty_cycle());

        // future work: setup pwm slices
        //  for now motors don't share the same pwm slice :(
        //  therefore we need to configure them in main
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
        let max_duty_a = self.pwm_a.max_duty_cycle() as f32;
        let max_duty_b = self.pwm_b.max_duty_cycle() as f32;

        // now assign the duty to each channel based on direction
        match direction {
            WheelRotDir::CW => {
                // scale duty for A
                let scaled_duty = ((duty_percent as f32 / 100f32) * max_duty_a as f32) as u16;

                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm_a.set_duty_cycle(scaled_duty).unwrap();    // rp235x-hal always returns Ok(())
                self.pwm_b.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
            },
            WheelRotDir::CCW => {
                // scale duty for B
                let scaled_duty = ((duty_percent as f32 / 100f32) * max_duty_b as f32) as u16;

                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm_a.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
                self.pwm_b.set_duty_cycle(scaled_duty).unwrap();    // rp235x-hal always returns Ok(())
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
                self.pwm_a.set_duty_cycle(duty).unwrap();    // rp235x-hal always returns Ok(())
                self.pwm_b.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
            },
            WheelRotDir::CCW => {
                // rotate cw by setting channel a to duty speed and b to 0
                self.pwm_a.set_duty_cycle(0).unwrap();        // rp235x-hal always returns Ok(())
                self.pwm_b.set_duty_cycle(duty).unwrap();    // rp235x-hal always returns Ok(())
            },
        }

        Ok(())
    }

    pub fn stop(&mut self) {
        self.pwm_a.set_duty_cycle(0).unwrap();
        self.pwm_b.set_duty_cycle(0).unwrap();
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
        // calculate motor's speed
        let rotations = self.get_encoder_ticks(true) as f32 / (TICKS_PER_ROTATION * MOTOR_GEAR_RATIO);
        let rotation_per_second = rotations / delta;
        let curr_speed = rotation_per_second * 2.0 * PI * WHEEL_DIAMETER;
        let weighted_speed = get_weighted_speed(&self.speed_history, &curr_speed);
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

    pub(crate) fn push_speed_history(&mut self, new_speed: &f32) {
        // push the new value into the history and overwrite the oldest value
        for i in 0..(SPEED_HISTORY_LENGTH-2) {
            self.speed_history[i] = self.speed_history[i + 1];
        }

        self.speed_history[SPEED_HISTORY_LENGTH - 1] = *new_speed;
    }
}
