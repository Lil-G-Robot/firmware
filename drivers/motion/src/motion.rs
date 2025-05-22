#![allow(unused)]

use core::f32::consts::{SQRT_2, PI};

use motor_driver::*;
use motor::{Motor, WheelRotDir};
use icm20948_driver::icm20948::i2c::IcmImu;
use rp235x_hal::{
        fugit::Duration, gpio::Interrupt, timer::Instant
    };
use rp235x_pkg::*; // ensures pins are right
use defmt::info;

// enum for directions
#[derive(Debug)]
pub enum MoveDirection {
    Forward,
    Right,
    Backward,
    Left,
}

#[derive(Debug)]
pub enum RotDirection {
    CW,
    CCW,
}

pub enum MotorNum {
    M1,
    M2,
    M3,
    M4,
}

pub struct Motion {
    // motor instances
    m1: Motor<M1PwmA, M1PwmB, M1EncA, M1EncB>,
    m2: Motor<M2PwmA, M2PwmB, M2EncA, M2EncB>,
    m3: Motor<M3PwmA, M3PwmB, M3EncA, M3EncB>,
    m4: Motor<M4PwmA, M4PwmB, M4EncA, M4EncB>,

    // add imu instance
    imu: IcmImu<ImuI2C>,

    // timer for instants
    timer: MotionTimer,

    // add motion control state variables
    last_timer_count: Instant,
}

impl Motion {
    pub fn new (
                m1: Motor<M1PwmA, M1PwmB, M1EncA, M1EncB>,
                m2: Motor<M2PwmA, M2PwmB, M2EncA, M2EncB>,
                m3: Motor<M3PwmA, M3PwmB, M3EncA, M3EncB>,
                m4: Motor<M4PwmA, M4PwmB, M4EncA, M4EncB>,
                imu: IcmImu<ImuI2C>,
                timer: MotionTimer,
    ) -> Self {
        Motion {
            // motors
            m1, m2, m3, m4,
            // imu instance
            imu,
            //timer
            timer,
            // get initial timer instant count
            last_timer_count: timer.get_counter(),
        }
    }

    // TODO: add config function for imu initialization
    pub fn configure(&mut self) {
        // put motors to default speed (0)
        self.stop_all_motors();

        // IMU stuff goes here :)
    }

    pub fn move_in_dir_duty(&mut self, duty: u16, direction: MoveDirection) {
        match direction {
            MoveDirection::Forward => {
                self.m1.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
                self.m2.set_duty_percent(duty, WheelRotDir::CW).unwrap();
                self.m3.set_duty_percent(duty, WheelRotDir::CW).unwrap();
                self.m4.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
            },
            MoveDirection::Right => {
                self.m1.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
                self.m2.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
                self.m3.set_duty_percent(duty, WheelRotDir::CW).unwrap();
                self.m4.set_duty_percent(duty, WheelRotDir::CW).unwrap();
            },
            MoveDirection::Backward => {
                self.m1.set_duty_percent(duty, WheelRotDir::CW).unwrap();
                self.m2.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
                self.m3.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
                self.m4.set_duty_percent(duty, WheelRotDir::CW).unwrap();
            },
            MoveDirection::Left => {
                self.m1.set_duty_percent(duty, WheelRotDir::CW).unwrap();
                self.m2.set_duty_percent(duty, WheelRotDir::CW).unwrap();
                self.m3.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
                self.m4.set_duty_percent(duty, WheelRotDir::CCW).unwrap();
            }
        };
    }

    pub fn move_in_dir_speed_diag(&mut self, speed: f32, direction: MoveDirection) {
        let speed = speed / (2.0 * SQRT_2);

        match direction {
            MoveDirection::Forward => {
                self.m1.set_speed_setpoint(-speed);
                self.m2.set_speed_setpoint(speed);
                self.m3.set_speed_setpoint(speed);
                self.m4.set_speed_setpoint(-speed);
            },
            MoveDirection::Right => {
                self.m1.set_speed_setpoint(-speed);
                self.m2.set_speed_setpoint(-speed);
                self.m3.set_speed_setpoint(speed);
                self.m4.set_speed_setpoint(speed);
            },
            MoveDirection::Backward => {
                self.m1.set_speed_setpoint(speed);
                self.m2.set_speed_setpoint(-speed);
                self.m3.set_speed_setpoint(-speed);
                self.m4.set_speed_setpoint(speed);
            },
            MoveDirection::Left => {
                self.m1.set_speed_setpoint(speed);
                self.m2.set_speed_setpoint(speed);
                self.m3.set_speed_setpoint(-speed);
                self.m4.set_speed_setpoint(-speed);
            }
        };
    }

    pub fn move_in_dir_speed(&mut self, speed: f32, direction: MoveDirection) {
        let speed = speed / 2.0;

        match direction {
            MoveDirection::Forward => {
                self.m1.set_speed_setpoint(0.0);
                self.m2.set_speed_setpoint(speed);
                self.m3.set_speed_setpoint(0.0);
                self.m4.set_speed_setpoint(-speed);
            },
            MoveDirection::Right => {
                self.m1.set_speed_setpoint(-speed);
                self.m2.set_speed_setpoint(0.0);
                self.m3.set_speed_setpoint(speed);
                self.m4.set_speed_setpoint(0.0);
            },
            MoveDirection::Backward => {
                self.m1.set_speed_setpoint(0.0);
                self.m2.set_speed_setpoint(-speed);
                self.m3.set_speed_setpoint(0.0);
                self.m4.set_speed_setpoint(speed);
            },
            MoveDirection::Left => {
                self.m1.set_speed_setpoint(speed);
                self.m2.set_speed_setpoint(0.0);
                self.m3.set_speed_setpoint(-speed);
                self.m4.set_speed_setpoint(0.0);
            }
        };
    }

    pub fn rotate_in_dir_speed(&mut self, speed: f32, direction: RotDirection) {
        let speed = speed / (2.0 * SQRT_2);

        match direction {
            RotDirection::CW => {
                self.m1.set_speed_setpoint(speed);
                self.m2.set_speed_setpoint(speed);
                self.m3.set_speed_setpoint(speed);
                self.m4.set_speed_setpoint(speed);
            },
            RotDirection::CCW => {
                self.m1.set_speed_setpoint(-speed);
                self.m2.set_speed_setpoint(-speed);
                self.m3.set_speed_setpoint(-speed);
                self.m4.set_speed_setpoint(-speed);
            },
        }
    }

    pub fn stop_all_motors(&mut self) {
        self.m1.stop();
        self.m2.stop();
        self.m3.stop();
        self.m4.stop();
    }

    pub(crate) fn adjust_motor_setpoint(&mut self, speed: f32, motor: MotorNum) {
        match motor {
            MotorNum::M1 => {
                self.m1.set_speed_setpoint(speed);
            },
            MotorNum::M2 => {
                self.m2.set_speed_setpoint(speed);
            },
            MotorNum::M3 => {
                self.m3.set_speed_setpoint(speed);
            },
            MotorNum::M4 => {
                self.m4.set_speed_setpoint(speed);
            }
        }
    }

    pub fn poll_all_setpoints(&mut self, backup_delta: f32) {
        let new_instant = self.timer.get_counter();
        let delta = match new_instant.checked_duration_since(self.last_timer_count) {
            Some(duration) => {(duration.to_micros() as f32) / 1_000_000.0},
            None => {
                info!("Invalid duration... Using backup delta");
                backup_delta
            },
        };
        self.last_timer_count = new_instant;

        // info!("Got delta: {} s", delta);
        self.m1.update_duty_to_setpoint(delta);
        self.m2.update_duty_to_setpoint(delta);
        self.m3.update_duty_to_setpoint(delta);
        self.m4.update_duty_to_setpoint(delta);
    }

    pub fn update_encoder_tick_all_motors(&mut self) {
        // check for interrupt flags on encoders and update if
        // tripped
        if self.m1.enc_a.interrupt_status(Interrupt::EdgeHigh) | self.m1.enc_a.interrupt_status(Interrupt::EdgeLow) {
            self.update_encoder_tick(MotorNum::M1);
        }
        if self.m2.enc_a.interrupt_status(Interrupt::EdgeHigh) | self.m2.enc_a.interrupt_status(Interrupt::EdgeLow) {
            self.update_encoder_tick(MotorNum::M2);
        }
        if self.m3.enc_a.interrupt_status(Interrupt::EdgeHigh) | self.m3.enc_a.interrupt_status(Interrupt::EdgeLow) {
            self.update_encoder_tick(MotorNum::M3);
        }
        if self.m4.enc_a.interrupt_status(Interrupt::EdgeHigh) | self.m4.enc_a.interrupt_status(Interrupt::EdgeLow) {
            self.update_encoder_tick(MotorNum::M4);
        }
    }

    pub fn update_encoder_tick(&mut self, m: MotorNum) {
        match m {
            MotorNum::M1 => {
                self.m1.update_encoder_tick();
            },
            MotorNum::M2 => {
                self.m2.update_encoder_tick();
            },
            MotorNum::M3 => {
                self.m3.update_encoder_tick();
            },
            MotorNum::M4 => {
                self.m4.update_encoder_tick();
            }
        }
    }

    pub fn update_encoder_tick_clear_irx_flag(&mut self, m: MotorNum) {
        match m {
            MotorNum::M1 => {
                self.m1.update_encoder_tick();
                self.m1.enc_a.clear_interrupt(Interrupt::EdgeHigh);
                self.m1.enc_a.clear_interrupt(Interrupt::EdgeLow);
                self.m1.enc_b.clear_interrupt(Interrupt::EdgeHigh);
                self.m1.enc_b.clear_interrupt(Interrupt::EdgeLow);
            },
            MotorNum::M2 => {
                self.m2.update_encoder_tick();
                self.m2.enc_a.clear_interrupt(Interrupt::EdgeHigh);
                self.m2.enc_a.clear_interrupt(Interrupt::EdgeLow);
                self.m2.enc_b.clear_interrupt(Interrupt::EdgeHigh);
                self.m2.enc_b.clear_interrupt(Interrupt::EdgeLow);
            },
            MotorNum::M3 => {
                self.m3.update_encoder_tick();
                self.m3.enc_a.clear_interrupt(Interrupt::EdgeHigh);
                self.m3.enc_a.clear_interrupt(Interrupt::EdgeLow);
                self.m3.enc_b.clear_interrupt(Interrupt::EdgeHigh);
                self.m3.enc_b.clear_interrupt(Interrupt::EdgeLow);
            },
            MotorNum::M4 => {
                self.m4.update_encoder_tick();
                self.m4.enc_a.clear_interrupt(Interrupt::EdgeHigh);
                self.m4.enc_a.clear_interrupt(Interrupt::EdgeLow);
                self.m4.enc_b.clear_interrupt(Interrupt::EdgeHigh);
                self.m4.enc_b.clear_interrupt(Interrupt::EdgeLow);
            }
        }
    }


    pub fn get_rotation_count(&mut self) -> (i32, i32, i32, i32) {
        let m1_count = (self.m1.get_encoder_ticks(true) / 12) as i32;
        let m2_count = (self.m2.get_encoder_ticks(true) / 12) as i32;
        let m3_count = (self.m3.get_encoder_ticks(true) / 12) as i32;
        let m4_count = (self.m4.get_encoder_ticks(true) / 12) as i32;

        (m1_count, m2_count, m3_count, m4_count)
    }

    pub fn get_encoder_ticks(&mut self) -> (i64, i64, i64, i64) {
        let m1_count = self.m1.get_encoder_ticks(false);
        let m2_count = self.m2.get_encoder_ticks(false);
        let m3_count = self.m3.get_encoder_ticks(false);
        let m4_count = self.m4.get_encoder_ticks(false);

        (m1_count, m2_count, m3_count, m4_count)
    }

    pub fn reset_encoder_ticks(&mut self) {
        self.m1.reset_encoder_ticks();
        self.m2.reset_encoder_ticks();
        self.m3.reset_encoder_ticks();
        self.m4.reset_encoder_ticks();
    }

    pub fn clear_all_encoder_interrupt(&mut self) {
        self.m1.enc_a.clear_interrupt(Interrupt::EdgeHigh);
        self.m1.enc_a.clear_interrupt(Interrupt::EdgeLow);
        self.m1.enc_b.clear_interrupt(Interrupt::EdgeHigh);
        self.m1.enc_b.clear_interrupt(Interrupt::EdgeLow);

        self.m2.enc_a.clear_interrupt(Interrupt::EdgeHigh);
        self.m2.enc_a.clear_interrupt(Interrupt::EdgeLow);
        self.m2.enc_b.clear_interrupt(Interrupt::EdgeHigh);
        self.m2.enc_b.clear_interrupt(Interrupt::EdgeLow);

        self.m3.enc_a.clear_interrupt(Interrupt::EdgeHigh);
        self.m3.enc_a.clear_interrupt(Interrupt::EdgeLow);
        self.m3.enc_b.clear_interrupt(Interrupt::EdgeHigh);
        self.m3.enc_b.clear_interrupt(Interrupt::EdgeLow);
 
        self.m4.enc_a.clear_interrupt(Interrupt::EdgeHigh);
        self.m4.enc_a.clear_interrupt(Interrupt::EdgeLow);
        self.m4.enc_b.clear_interrupt(Interrupt::EdgeHigh);
        self.m4.enc_b.clear_interrupt(Interrupt::EdgeLow);
    }

    pub fn test_gyro(&mut self) {
        // simply read the gyro and classify the rotational speed detected
        let enc_data = self.imu.read_gyro_z().unwrap();

        // for now just print the speed
        info!("Enc Z: {}", enc_data);
        // now print something based on the speed
    }
}
