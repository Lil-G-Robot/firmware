//! Support package for use with the RP235x series of microcontrollers

#![allow(unused)] // just to make this file look nicer

use rp235x_hal::{
    gpio::{bank0::*, *}, i2c::*, pac::I2C1, pwm::*, timer::{Timer, CopyableTimer1}
    };

// I could probably use macros maybe? Oh well here we go anyways
// This can definitely be more efficient... Especially if pins change

// Motor 1
pub type M1PwmA = Channel<Slice<Pwm4, FreeRunning>, A>;
pub type M1PwmB = Channel<Slice<Pwm4, FreeRunning>, B>;
pub type M1EncA = Pin<Gpio26, FunctionSio<SioInput>, PullDown>;
pub type M1EncB = Pin<Gpio27, FunctionSio<SioInput>, PullDown>;

// Motor 2
pub type M2PwmA = Channel<Slice<Pwm2, FreeRunning>, A>;
pub type M2PwmB = Channel<Slice<Pwm2, FreeRunning>, B>;
pub type M2EncA = Pin<Gpio22, FunctionSio<SioInput>, PullDown>;
pub type M2EncB = Pin<Gpio23, FunctionSio<SioInput>, PullDown>;

// Motor 3
pub type M3PwmA = Channel<Slice<Pwm0, FreeRunning>, A>;
pub type M3PwmB = Channel<Slice<Pwm0, FreeRunning>, B>;
pub type M3EncA = Pin<Gpio18, FunctionSio<SioInput>, PullDown>;
pub type M3EncB = Pin<Gpio19, FunctionSio<SioInput>, PullDown>;

// Motor 4
pub type M4PwmA = Channel<Slice<Pwm6, FreeRunning>, A>;
pub type M4PwmB = Channel<Slice<Pwm6, FreeRunning>, B>;
pub type M4EncA = Pin<Gpio14, FunctionSio<SioInput>, PullDown>;
pub type M4EncB = Pin<Gpio15, FunctionSio<SioInput>, PullDown>;

// I2C0
pub type ImuI2C = I2C<I2C1, (Pin<Gpio2, FunctionI2c, PullUp>, Pin<Gpio3, FunctionI2c, PullUp>)>;

pub type MotionTimer = Timer<CopyableTimer1>;

// LED
pub type LED1 = Pin<Gpio11, FunctionSio<SioOutput>, PullDown>;
pub type LED2 = Pin<Gpio10, FunctionSio<SioOutput>, PullDown>;
pub type LED3 = Pin<Gpio9, FunctionSio<SioOutput>, PullDown>;
pub type LED4 = Pin<Gpio8, FunctionSio<SioOutput>, PullDown>;
