use arduino_hal::delay_ms;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use embedded_hal::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex};
use core::fmt::Debug;

use crate::imu::comm::Comm;

pub const I2C_DEFAULT_ADDR: u8 = 0x13;

pub mod reg {
    pub const CHIP_ID       : u8 = 0x00;
    pub const POWER_CONTROL : u8 = 0x1B;
    pub const ODR           : u8 = 0x1D;
}

pub mod val {
    pub const CHIP_ID : u8 = 0x50;
    pub const POWER_CONTROL_ON          : u8 = 0b00000011;
    pub const POWER_CONTROL_MODE_NORMAL : u8 = 0b00110000;
    pub const ODR_200HZ                 : u8 = 0x0;
}

pub fn init<C: Comm>(mut comm: C) -> Result<bool, <C as Comm>::Error> where <C as Comm>::Error: Debug {
    comm.reg_write_byte(reg::POWER_CONTROL, val::POWER_CONTROL_ON | val::POWER_CONTROL_MODE_NORMAL)?;
    if comm.reg_read_byte(reg::CHIP_ID).unwrap() != val::CHIP_ID {
        return Ok(false)
    }
    comm.reg_write_byte(reg::ODR, val::ODR_200HZ)?;
    Ok(true)
}
