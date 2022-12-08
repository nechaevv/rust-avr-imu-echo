use arduino_hal::{I2c, Spi};
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use embedded_hal::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex};
use nb::block;

pub trait Comm {
    type Error;
    fn reg_read_byte(&mut self, reg_addr: u8) -> Result<u8, Self::Error>;
    fn reg_read_byte_bmp388(&mut self, reg_addr: u8) -> Result<u8, Self::Error>;
    fn reg_read_array<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error>;
    fn reg_read_array_bmp388<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error>;
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
    fn reg_read_byte_bmp388(&mut self, reg_addr: u8) -> Result<u8, Self::Error> {
        self.reg_read_byte(reg_addr)
    }

    #[inline]
    fn reg_read_array<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error> {
        let mut buffer = [0u8; N];
        self.0.write_read(self.1, &[reg_addr], &mut buffer)?;
        Ok(buffer)
    }

    #[inline]
    fn reg_read_array_bmp388<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error> {
        self.reg_read_array::<N>(reg_addr)
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

impl<CSPIN: OutputPin> Comm for (&mut Spi, &mut CSPIN) where <CSPIN as OutputPin>::Error: core::fmt::Debug {
    type Error = void::Void;

    fn reg_read_byte(&mut self, reg_addr: u8) -> Result<u8, Self::Error> {
        self.1.set_low().unwrap();
        let mut buffer = [reg_addr | 0x80, 0u8];
        self.0.transfer(&mut buffer)?;
        self.1.set_high().unwrap();
        Ok(buffer[1])
    }

    fn reg_read_byte_bmp388(&mut self, reg_addr: u8) -> Result<u8, Self::Error> {
        self.1.set_low().unwrap();
        let mut buffer = [reg_addr | 0x80, 0u8, 0u8];
        self.0.transfer(&mut buffer)?;
        self.1.set_high().unwrap();
        Ok(buffer[2])
    }

    fn reg_read_array<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error> {
        let mut buffer = [0u8; N];
        self.1.set_low().unwrap();
        block!(self.0.send(reg_addr | 0x80))?;
        block!(self.0.read())?;
        self.0.transfer(&mut buffer)?;
        self.1.set_high().unwrap();
        Ok(buffer)
    }

    fn reg_read_array_bmp388<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error> {
        let mut buffer = [0u8; N];
        self.1.set_low().unwrap();
        block!(self.0.send(reg_addr | 0x80))?;
        block!(self.0.read())?;
        block!(self.0.send(0))?;
        block!(self.0.read())?;
        self.0.transfer(&mut buffer)?;
        self.1.set_high().unwrap();
        Ok(buffer)
    }

    fn reg_read_to_buffer(&mut self, reg_addr: u8, buffer: &mut [u8], length: usize) -> Result<(), Self::Error> {
        self.1.set_low().unwrap();
        block!(self.0.send(reg_addr | 0x80))?;
        block!(self.0.read())?;
        self.0.transfer(&mut buffer[..length])?;
        self.1.set_high().unwrap();
        Ok(())
    }


    fn reg_write_byte(&mut self, reg_addr: u8, value: u8) -> Result<(), Self::Error> {
        self.1.set_low().unwrap();
        self.0.write(&[reg_addr, value])?;
        self.1.set_high().unwrap();
        Ok(())
    }

    fn reg_write_array(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> {
        todo!(); // not supported
    }
}
