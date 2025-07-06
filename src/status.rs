use embedded_hal::digital::OutputPin;
use motor_driver::rp235x_pkg::*;

pub struct Status {
    led_1: LED1,
    led_2: LED2,
    led_3: LED3,
    led_4: LED4,
}

impl Status {
    pub fn new(
        led_1: LED1,
        led_2: LED2,
        led_3: LED3,
        led_4: LED4
    ) -> Self {
        Status {
            led_1,
            led_2,
            led_3,
            led_4
        }
    }

    pub fn reset_all(&mut self) {
        self.led_1.set_low().unwrap();
        self.led_2.set_low().unwrap();
        self.led_3.set_low().unwrap();
        self.led_4.set_low().unwrap();
    }

    pub fn set_all(&mut self) {
        self.led_1.set_high().unwrap();
        self.led_2.set_high().unwrap();
        self.led_3.set_high().unwrap();
        self.led_4.set_high().unwrap();
    }
}
