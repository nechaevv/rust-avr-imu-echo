use arduino_hal::{delay_ms, I2c, Spi};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embedded_hal::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex};
use core::fmt::Debug;
use nb::block;

pub const I2C_DEFAULT_ADDR: u8 = 0x13;

pub mod reg {
    pub const CHIP_ID    : u8 = 0x00;
    pub const ERROR      : u8 = 0x02;
    pub const STATUS     : u8 = 0x03;
    pub const DATA       : u8 = 0x04;
    pub const PWR_CTRL   : u8 = 0x1B;
    pub const OSR        : u8 = 0x1C;
    pub const ODR        : u8 = 0x1D;
    pub const IIR_CONF   : u8 = 0x1F;
    pub const CALIB_DATA : u8 = 0x31;
    pub const COMMAND    : u8 = 0x7E;
}

pub mod val {
    pub const CHIP_ID : u8 = 0x50;

    pub const ERROR_CMD_FAILED_BIT : u8 = 0b00000010;
    pub const ERROR_CONFIG_ERR_BIT : u8 = 0b00000100;

    pub const STATUS_CMD_RDY_BIT   : u8 = 0b00010000;
    pub const STATUS_PRESS_RDY_BIT : u8 = 0b00100000;
    pub const STATUS_TEMP_RDY_BIT  : u8 = 0b01000000;

    pub const POWER_CONTROL_PRESS_ON    : u8 = 0b00000001;
    pub const POWER_CONTROL_TEMP_ON     : u8 = 0b00000010;
    pub const POWER_CONTROL_MODE_NORMAL : u8 = 0b00110000;
    pub const ODR_200HZ                 : u8 = 0x00;
    pub const ODR_100HZ                 : u8 = 0x01;
    pub const ODR_50HZ                  : u8 = 0x02;
    pub const ODR_25HZ                  : u8 = 0x03;
    pub const OSR_TEMP_X2               : u8 = 0b00001000;
    pub const OSR_PRESS_X16             : u8 = 0b00000100;
    pub const IIR_COEF_3                : u8 = 0b010;

    pub const CMD_SOFT_RESET : u8 = 0xB6;
}

pub struct CalibrationData {
    par_t1  : u16,
    par_t2  : u16,
    par_t3  : i8,
    par_p1  : i16,
    par_p2  : i16,
    par_p3  : i8,
    par_p4  : i8,
    par_p5  : u16,
    par_p6  : u16,
    par_p7  : i8,
    par_p8  : i8,
    par_p9  : i16,
    par_p10 : i8,
    par_p11 : i8
}

pub struct Bmp388<BE: BusEndpoint> {
    bus_endpoint: BE,
    calibration_data: CalibrationData
}

#[derive(Debug)]
pub enum Error {
    CommError,
    SensorError
}

impl<BE: BusEndpoint> Bmp388<BE> {
    pub fn init(mut bus_endpoint: BE, bus: &mut BE::Bus) -> Result<Bmp388<BE>, Error> {
        //chip id
        if bus_endpoint.read_byte(bus, reg::CHIP_ID)? != val::CHIP_ID {
            return Err(Error::SensorError)
        }
        //reset
        let status = bus_endpoint.read_byte(bus, reg::STATUS)?;
        if status & val::STATUS_CMD_RDY_BIT == 0 {
            return Err(Error::SensorError);
        }
        bus_endpoint.write_byte(bus, reg::COMMAND, val::CMD_SOFT_RESET)?;
        delay_ms(2);
        let err = bus_endpoint.read_byte(bus, reg::ERROR)?;
        if err & val::ERROR_CMD_FAILED_BIT > 0 {
            return Err(Error::SensorError);
        }
        // calibration data
        let buffer = bus_endpoint.read_array::<21>(bus, reg::CALIB_DATA)?;
        let calibration_data = CalibrationData {
            par_t1: u16::from_le_bytes([buffer[0], buffer[1]]), // as f64) * 256.0f64;
            par_t2: u16::from_le_bytes([buffer[2], buffer[3]]), // as f64) / 1073741824.0f64;
            par_t3: i8 ::from_le_bytes([buffer[4]]), // as f64) / 281474976710656.0f64;
            par_p1: i16::from_le_bytes([buffer[5], buffer[6]]), // - 16384) as f64 / 1048576.0;
            par_p2: i16::from_le_bytes([buffer[7], buffer[8]]), // - 16384) as f64 / 536870912.0;
            par_p3: i8 ::from_le_bytes([buffer[9]]), // as f64 / 4294967296.0;
            par_p4: i8 ::from_le_bytes([buffer[10]]), // as f64 / 137438953472.0;
            par_p5: u16::from_le_bytes([buffer[11], buffer[12]]), // as f64 * 8.0;
            par_p6: u16::from_le_bytes([buffer[13], buffer[14]]), // as f64 / 64.0;
            par_p7: i8 ::from_le_bytes([buffer[15]]), // as f64 / 256.0;
            par_p8: i8 ::from_le_bytes([buffer[16]]), // as f64 / 32768.0;
            par_p9: i16::from_le_bytes([buffer[17], buffer[18]]), // as f64 / 281474976710656.0;
            par_p10: i8::from_le_bytes([buffer[19]]), // as f64 / 281474976710656.0;
            par_p11: i8::from_le_bytes([buffer[20]]), // as f64 / 36893488147419103232.0;
        };

        //set_config
        let pc = bus_endpoint.read_byte(bus, reg::PWR_CTRL)?;
        bus_endpoint.write_byte(bus, reg::PWR_CTRL, pc | val::POWER_CONTROL_PRESS_ON | val::POWER_CONTROL_TEMP_ON | val::POWER_CONTROL_MODE_NORMAL)?;
        bus_endpoint.write_byte(bus, reg::IIR_CONF, val::IIR_COEF_3)?;
        bus_endpoint.write_byte(bus, reg::OSR, val::OSR_TEMP_X2 | val::OSR_PRESS_X16)?;
        bus_endpoint.write_byte(bus, reg::ODR, val::ODR_25HZ)?;

        Ok(Bmp388 {
            bus_endpoint,
            calibration_data
        })
    }

    pub fn get_data(&mut self, bus: &mut BE::Bus, pressure: &mut u32, temperature: &mut i32, timestamp: &mut u32) -> Result<bool, Error> {
        let status = self.bus_endpoint.read_byte(bus, reg::STATUS)?;
        if status & val::STATUS_PRESS_RDY_BIT == 0 {
            return Ok(false);
        }
        let buffer = self.bus_endpoint.read_array::<9>(bus, reg::DATA)?;
        let raw_press = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], 0]);
        let raw_temp = u32::from_le_bytes([buffer[3], buffer[4], buffer[5], 0]);
        *timestamp = u32::from_le_bytes([buffer[6], buffer[7], buffer[8], 0]);
        let t_lin = self.compensate_temp(raw_temp);
        *temperature = t_lin as i32; // 1/256 DegC
        let p_lin = self.compensate_press(raw_press, t_lin);
        *pressure = p_lin as u32; // 1/256 Pa
        Ok(true)
    }

    fn compensate_temp(&self, raw_temp: u32) -> i64 {
        let partial_data1 = ((raw_temp as i64) - ((self.calibration_data.par_t1 as i64) << 8)) << 8;
        let partial_data2 = (partial_data1 * (self.calibration_data.par_t2 as i64) ) >> 30;
        let partial_data3 = (partial_data1 * partial_data1) >> 8;
        partial_data2 + (partial_data3 * (self.calibration_data.par_t3 as i64) >> 48)
    }

    fn compensate_press(&self, raw_press: u32, t_lin: i64) -> i64 {
        let uncomp_press = raw_press as i64;
        let t_lin2 = (t_lin * t_lin) >> 8;
        let t_lin3 = (t_lin2 * t_lin) >> 8;
        let partial_out1 = {
            let partial_data1 = ((self.calibration_data.par_p6 as i64) * t_lin) >> 6;
            let partial_data2 = ((self.calibration_data.par_p7 as i64) * t_lin2) >> 8;
            let partial_data3 = ((self.calibration_data.par_p8 as i64) * t_lin3) >> 15;
            ((self.calibration_data.par_p5 as i64) << 11) + partial_data1 + partial_data2 + partial_data3
        };
        let partial_out2 = {
            let partial_data1 = ((self.calibration_data.par_p2 as i64 - 16384) * t_lin) >> 9;
            let partial_data2 = ((self.calibration_data.par_p3 as i64) * t_lin2) >> 12;
            let partial_data3 = ((self.calibration_data.par_p4 as i64) * t_lin3) >> 17;
            uncomp_press * (((self.calibration_data.par_p1 as i64) - 16384) << 8) + partial_data1 + partial_data2 + partial_data3
        };
        let partial_data4 = {
            let up2 = (uncomp_press * uncomp_press) >> 8;
            let partial_data2 = self.calibration_data.par_p9 as i64 + ((self.calibration_data.par_p10 as i64 * t_lin) >> 8);
            let partial_data3 = up2 * partial_data2;
            let up3 = (up2 * uncomp_press) >> 8;
            partial_data3 + ((up3 * (self.calibration_data.par_p11 as i64)) >> 17)
        };
        partial_out1 + (partial_out2 >> 20) + (partial_data4 >> 32)
    }

}

pub trait BusEndpoint {
    type Bus;
    fn read_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<u8, Error>;
    fn read_array<const N: usize>(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<[u8; N], Error>;
    fn write_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8, value: u8) -> Result<(), Error>;
}

pub struct I2cEndpoint {
    i2c_address: u8
}

pub struct SpiEndpoint<CSPIN: OutputPin> {
    cs_pin: CSPIN
}

impl From<arduino_hal::i2c::Error> for Error {
    fn from(_: arduino_hal::i2c::Error) -> Error {
        Error::CommError
    }
}

impl BusEndpoint for I2cEndpoint {
    type Bus = I2c;
    fn read_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<u8, Error> {
        let mut value: [u8;1] = [0];
        bus.write_read(self.i2c_address, &[reg_addr], &mut value)?;
        Ok(value[0])
    }
    fn read_array<const N: usize>(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<[u8; N], Error> {
        let mut buffer = [0u8; N];
        bus.write_read(self.i2c_address, &[reg_addr], &mut buffer)?;
        Ok(buffer)
    }
    fn write_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8, value: u8) -> Result<(), Error> {
        bus.write(self.i2c_address, &[reg_addr])?;
        bus.write(self.i2c_address, &[value])?;
        Ok(())
    }
}

fn comm_err<T>(_:T) -> Error {
    Error::CommError
}

impl<CSPIN: OutputPin> BusEndpoint for SpiEndpoint<CSPIN> where <CSPIN as OutputPin>::Error: Debug {
    type Bus = Spi;

    fn read_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<u8, Error> {
        self.cs_pin.set_low().map_err(comm_err)?;
        let mut buffer = [reg_addr | 0x80, 0u8, 0u8];
        bus.transfer(&mut buffer).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(buffer[2])
    }
    fn read_array<const N: usize>(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<[u8; N], Error> {
        let mut buffer = [0u8; N];
        self.cs_pin.set_low().map_err(comm_err)?;
        block!(bus.send(reg_addr | 0x80)).map_err(comm_err)?;
        block!(bus.read()).map_err(comm_err)?;
        block!(bus.send(0)).map_err(comm_err)?;
        block!(bus.read()).map_err(comm_err)?;
        bus.transfer(&mut buffer).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(buffer)
    }
    fn write_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8, value: u8) -> Result<(), Error> {
        self.cs_pin.set_low().map_err(comm_err)?;
        bus.write(&[reg_addr, value]).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(())
    }

}

pub fn new_i2c(i2c: &mut I2c, i2c_address: u8) -> Result<Bmp388<I2cEndpoint>, Error> {
    Bmp388::init(I2cEndpoint { i2c_address }, i2c)
}

pub fn new_spi<CSPIN: OutputPin>(spi: &mut Spi, cs_pin: CSPIN) -> Result<Bmp388<SpiEndpoint<CSPIN>>, Error> where <CSPIN as OutputPin>::Error: Debug {
    Bmp388::init(SpiEndpoint { cs_pin }, spi)
}
