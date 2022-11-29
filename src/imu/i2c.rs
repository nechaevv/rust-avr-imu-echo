//I2C convenience functions

use arduino_hal::I2c;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};

pub trait I2cRegExt {
    fn reg_read_byte(&mut self, dev_addr: u8, reg_addr: u8) -> u8;
    fn reg_read_array(&mut self, dev_addr: u8, reg_addr: u8, arr: &mut [u8]);
    fn reg_write_byte(&mut self, dev_addr: u8, reg_addr: u8, value: u8);
    fn reg_write_array(&mut self, dev_addr: u8, reg_addr: u8, arr: &[u8]);
}

impl I2cRegExt for I2c {

    #[inline]
    fn reg_read_byte(&mut self, dev_addr: u8, reg_addr: u8) -> u8 {//} Result<u8, Error> {
        let mut value: [u8;1] = [0];
        self.write_read(dev_addr, &[reg_addr], &mut value).unwrap();
        // i2c.write(dev_addr, &[reg_addr]);
        // i2c.read(dev_addr, &mut value);
        value[0]
    }

    #[inline]
    fn reg_read_array(&mut self, dev_addr: u8, reg_addr: u8, arr: &mut [u8]) {
        self.write_read(dev_addr, &[reg_addr], arr).unwrap();
    }

    #[inline]
    fn reg_write_byte(&mut self, dev_addr: u8, reg_addr: u8, value: u8) {
        self.write(dev_addr, &[reg_addr]).unwrap();
        self.write(dev_addr, &[value]).unwrap();
    }

    #[inline]
    fn reg_write_array(&mut self, dev_addr: u8, reg_addr: u8, arr: &[u8]) {
        self.write(dev_addr, &[reg_addr]).unwrap();
        self.write(dev_addr, arr).unwrap();
    }
}
