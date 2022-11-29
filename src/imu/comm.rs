use arduino_hal::I2c;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};

pub trait Comm {
    type Error;
    fn reg_read_byte(&mut self, reg_addr: u8) -> Result<u8, Self::Error>;
    fn reg_read_array<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error>;
    fn reg_read_to_buffer(&mut self, reg_addr: u8, buffer: &mut [u8], length: usize) -> Result<(), Self::Error>;
    fn reg_write_byte(&mut self, reg_addr: u8, value: u8) -> Result<(), Self::Error>;
    fn reg_write_array(&mut self, reg_addr: u8, arr: &[u8]) -> Result<(), Self::Error>;
}



impl Comm for (&mut I2c, u8) {
    type Error = arduino_hal::i2c::Error;

    #[inline]
    fn reg_read_byte(&mut self, reg_addr: u8) -> Result<u8, Self::Error> {
        let mut value: [u8;1] = [0];
        self.0.write_read(self.1, &[reg_addr], &mut value)?;
        Ok(value[0])
    }

    #[inline]
    fn reg_read_array<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error> {
        let mut buffer = [0u8; N];
        self.0.write_read(self.1, &[reg_addr], &mut buffer)?;
        Ok(buffer)
    }

    fn reg_read_to_buffer(&mut self, reg_addr: u8, buffer: &mut [u8], length: usize) -> Result<(), Self::Error> {
        self.0.write_read(self.1, &[reg_addr], &mut buffer[..length])
    }

    #[inline]
    fn reg_write_byte(&mut self, reg_addr: u8, value: u8) -> Result<(), Self::Error> {
        self.0.write(self.1, &[reg_addr])?;
        self.0.write(self.1, &[value])
    }

    #[inline]
    fn reg_write_array(&mut self, reg_addr: u8, arr: &[u8]) -> Result<(), Self::Error> {
        self.0.write(self.1, &[reg_addr])?;
        self.0.write(self.1, arr)
    }

}
