//! Support package for use with the RP235x series of microcontrollers

#![allow(unused)] // just to make this file look nicer

use rp235x_hal::{
    gpio::{bank0::*, *}, i2c::*, pac::I2C0, pwm::*, timer::{Timer, CopyableTimer1}
    };

// I could probably use macros maybe? Oh well here we go anyways
// This can definitely be more efficient... Especially if pins change

// Motor 1
pub type M1PwmA = Channel<Slice<Pwm0, FreeRunning>, B>;
pub type M1PwmB = Channel<Slice<Pwm0, FreeRunning>, A>;
pub type M1EncA = Pin<Gpio22, FunctionSio<SioInput>, PullDown>;
pub type M1EncB = Pin<Gpio21, FunctionSio<SioInput>, PullDown>;

// Motor 2
pub type M2PwmA = Channel<Slice<Pwm2, FreeRunning>, A>;
pub type M2PwmB = Channel<Slice<Pwm5, FreeRunning>, A>;
pub type M2EncA = Pin<Gpio1, FunctionSio<SioInput>, PullDown>;
pub type M2EncB = Pin<Gpio0, FunctionSio<SioInput>, PullDown>;

// Motor 3
pub type M3PwmA = Channel<Slice<Pwm7, FreeRunning>, A>;
pub type M3PwmB = Channel<Slice<Pwm1, FreeRunning>, B>;
pub type M3EncA = Pin<Gpio2, FunctionSio<SioInput>, PullDown>;
pub type M3EncB = Pin<Gpio28, FunctionSio<SioInput>, PullDown>;

// Motor 4
pub type M4PwmA = Channel<Slice<Pwm1, FreeRunning>, A>;
pub type M4PwmB = Channel<Slice<Pwm7, FreeRunning>, B>;
pub type M4EncA = Pin<Gpio3, FunctionSio<SioInput>, PullDown>;
pub type M4EncB = Pin<Gpio27, FunctionSio<SioInput>, PullDown>;

// I2C0
pub type ImuI2C = I2C<I2C0, (Pin<Gpio12, FunctionI2c, PullUp>, Pin<Gpio13, FunctionI2c, PullUp>)>;

pub type MotionTimer = Timer<CopyableTimer1>;
