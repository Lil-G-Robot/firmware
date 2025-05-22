#![no_std]
#![crate_type = "lib"]

pub mod motor;

#[cfg(feature = "rp235x")]
pub mod rp235x_pkg;

