use arduino_hal::{delay_ms, I2c, Spi};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embedded_hal::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex};
use core::fmt::Debug;
use nb::block;

pub const I2C_DEFAULT_ADDR: u8 = 0x68;

pub mod reg {
    pub const REG_BANK_SEL      : u8 = 0x76;
    pub mod bank0 {
        pub const DEVICE_CONFIG : u8 = 0x11;
        pub const DRIVE_CONFIG  : u8 = 0x13;
        pub const FIFO_CONFIG   : u8 = 0x16;

        pub const TEMP_DATA1    : u8 = 0x1D;
        pub const TEMP_DATA0    : u8 = 0x1E;
        pub const ACCEL_DATA_X1 : u8 = 0x1F;
        pub const ACCEL_DATA_X0 : u8 = 0x20;
        pub const ACCEL_DATA_Y1 : u8 = 0x21;
        pub const ACCEL_DATA_Y0 : u8 = 0x22;
        pub const ACCEL_DATA_Z1 : u8 = 0x23;
        pub const ACCEL_DATA_Z0 : u8 = 0x24;
        pub const GYRO_DATA_X1  : u8 = 0x25;
        pub const GYRO_DATA_X0  : u8 = 0x26;
        pub const GYRO_DATA_Y1  : u8 = 0x27;
        pub const GYRO_DATA_Y0  : u8 = 0x28;
        pub const GYRO_DATA_Z1  : u8 = 0x29;
        pub const GYRO_DATA_Z0  : u8 = 0x2A;

        pub const INT_STATUS    : u8 = 0x2D;
        pub const FIFO_COUNTH   : u8 = 0x2E;
        pub const FIFO_COUNTL   : u8 = 0x2F;
        pub const FIFO_DATA     : u8 = 0x30;

        pub const SIGNAL_PATH_RESET : u8 = 0x4B;
        pub const PWR_MGMT_0        : u8 = 0x4E;
        pub const GYRO_CONFIG_0     : u8 = 0x4F;
        pub const ACCEL_CONFIG_0    : u8 = 0x50;
        pub const SMD_CONFIG        : u8 = 0x57;
        pub const FIFO_CONFIG_1     : u8 = 0x5F;
        pub const FIFO_CONFIG_2     : u8 = 0x60;
        pub const FIFO_CONFIG_3     : u8 = 0x61;

        pub const WHO_AM_I: u8 = 0x75;
    }

}

pub mod val {
    pub const WHO_AM_I: u8 = 0x47;

    pub const DEVCONFIG_SOFT_RESET_ENABLE: u8 = 0x01;

    pub const DRIVECONFIG_MIN_SLEW_RATE: u8 = 0x00;

    pub const PWR_TEMP_DISABLE        : u8 = 0b00100000;
    pub const PWR_IDLE_0              : u8 = 0b00010000;
    pub const PWR_GYRO_MODE_LOW_NOISE : u8 = 0b00001100;
    pub const PWR_ACCEL_MODE_LOW_NOISE: u8 = 0b00000011;

    pub const ODR_32KHZ   : u8 = 0x1;
    pub const ODR_16KHZ   : u8 = 0x2;
    pub const ODR_8KHZ    : u8 = 0x3;
    pub const ODR_4KHZ    : u8 = 0x4;
    pub const ODR_2KHZ    : u8 = 0x5;
    pub const ODR_1KHZ    : u8 = 0x6;
    pub const ODR_200HZ   : u8 = 0x7;
    pub const ODR_100HZ   : u8 = 0x8;
    pub const ODR_50HZ    : u8 = 0x9;
    pub const ODR_25KHZ   : u8 = 0xA;
    pub const ODR_12_5KHZ : u8 = 0xB;
    pub const ODR_6_25KHZ : u8 = 0xC;
    pub const ODR_3_125HZ : u8 = 0xD;
    pub const ODR_1_5625HZ: u8 = 0xE;
    pub const ODR_500HZ   : u8 = 0xF;

    pub const ACFG0_FS_SEL_16G: u8 = 0x00;
    pub const GCFG0_FS_SEL_2000DPS: u8 = 0x00;

    pub const FIFOCONFIG_STREAM_TO_FIFO_MODE: u8 = 0b01000000;
    pub const FIFOCONFIG_STOP_ON_FULL_MODE  : u8 = 0b11000000;

    pub const FIFOCONFIG1_HIRES_EN      : u8 = 0b10000;
    pub const FIFOCONFIG1_TMST_FSYNC_EN : u8 = 0b01000;
    pub const FIFOCONFIG1_GYRO_EN       : u8 = 0b00010;
    pub const FIFOCONFIG1_ACCEL_EN      : u8 = 0b00001;

    pub const RESET_DONE_INT: u8 = 0b10000;
    pub const DATA_RDY_INT  : u8 = 0b01000;
    pub const FIFO_THS_INT  : u8 = 0b00100;
    pub const FIFO_FULL_INT : u8 = 0b00010;

    pub const FIFOHEADER_TIMESTAMP_ODR: u8 = 0b1000;
    pub const FIFOHEADER_ACCEL        : u8 = 0b0100;
    pub const FIFOHEADER_GYRO         : u8 = 0b0010;

    pub const FIFO_FLUSH: u8 = 0b10;

}

pub struct Icm42688<BE: BusEndpoint> {
    bus_endpoint: BE
}

#[derive(Debug)]
pub enum Error {
    CommError,
    SensorError
}

impl<BE: BusEndpoint> Icm42688<BE> {
    pub fn init(mut bus_endpoint: BE, bus: &mut BE::Bus) -> Result<Icm42688<BE>, Error> {
        // Software reset
        bus_endpoint.write_byte(bus, reg::bank0::DEVICE_CONFIG, val::DEVCONFIG_SOFT_RESET_ENABLE)?;
        delay_ms(1);

        loop {
            if bus_endpoint.read_byte(bus, reg::bank0::WHO_AM_I)? == val::WHO_AM_I
                && bus_endpoint.read_byte(bus, reg::bank0::DEVICE_CONFIG)? == 0x00
                && bus_endpoint.read_byte(bus, reg::bank0::INT_STATUS)? & val::RESET_DONE_INT != 0
            {
                break;
            }
            delay_ms(10);
        }

        // sensors config
        bus_endpoint.write_byte(bus, reg::bank0::ACCEL_CONFIG_0, val::ACFG0_FS_SEL_16G | val::ODR_4KHZ)?;
        bus_endpoint.write_byte(bus, reg::bank0::GYRO_CONFIG_0, val::GCFG0_FS_SEL_2000DPS | val::ODR_4KHZ)?;

        //comm.reg_write_byte(reg::bank0::DRIVE_CONFIG, val::DRIVECONFIG_MIN_SLEW_RATE)?;
        let pwr_mgmt = val::PWR_GYRO_MODE_LOW_NOISE | val::PWR_ACCEL_MODE_LOW_NOISE;
        bus_endpoint.write_byte(bus, reg::bank0::PWR_MGMT_0, pwr_mgmt)?;
        delay_ms(45);
        if bus_endpoint.read_byte(bus, reg::bank0::PWR_MGMT_0)? != pwr_mgmt {
            return Err(Error::SensorError);
        }
        // FIFO config
        let fifo_config_1 = val::FIFOCONFIG1_HIRES_EN; // | val::FIFOCONFIG1_TEMP_EN | val::FIFOCONFIG1_GYRO_EN | val::FIFOCONFIG1_ACCEL_EN;
        bus_endpoint.write_byte(bus, reg::bank0::FIFO_CONFIG_1, fifo_config_1)?;
        if bus_endpoint.read_byte(bus, reg::bank0::FIFO_CONFIG_1)? != fifo_config_1 {
            return Err(Error::SensorError);
        }
        bus_endpoint.write_byte(bus, reg::bank0::FIFO_CONFIG, val::FIFOCONFIG_STOP_ON_FULL_MODE/* val::FIFOCONFIG_STREAM_TO_FIFO_MODE*/)?;
        delay_ms(100);
        let mut sensor = Icm42688 { bus_endpoint };
        sensor.fifo_reset(bus)?;
        Ok(sensor)
    }

    pub fn get_data(&mut self, bus: &mut BE::Bus, accel: &mut [i32; 3], gyro: &mut [i32; 3], temp: &mut i16, timestamp: &mut i16) -> Result<u16, Error> {
        let mut buffer = [0u8; BUFFER_SIZE];
        let mut records_read = 0u16;


        let fifo_count = self.bus_endpoint.read_array::<2>(bus, reg::bank0::FIFO_COUNTH)?;
        let bytes_available = u16::from_be_bytes(fifo_count);
        if bytes_available < PACKET_SIZE_BYTES as u16 {
            return Ok(0u16);
        }
        let mut records_to_read: usize = bytes_available as usize / PACKET_SIZE_BYTES;
        if records_to_read > READ_BUFFER_RECORDS {
            records_to_read = READ_BUFFER_RECORDS;
        }
        self.bus_endpoint.read_to_buffer(bus, reg::bank0::FIFO_DATA, &mut buffer, records_to_read * PACKET_SIZE_BYTES)?;
        //let buffer = comm.reg_read_array::<BUFFER_SIZE>(reg::bank0::FIFO_DATA)?;
        let mut offset: usize = 0;
        for _ in 0..records_to_read {
            if buffer[offset] & 0b11110000 != 0b01110000 {
                continue;
            }
            let mut i2: usize = 0;
            for i in 0..3 {
                avg_with_gain_i32(&mut accel[i], assemble_20bit_accel(buffer[offset + 0x1 + i2], buffer[offset + 0x2 + i2], buffer[offset + 0x11 + i] & 0xF0));
                avg_with_gain_i32(&mut gyro[i], assemble_20bit_gyro(buffer[offset + 0x7 + i2], buffer[offset + 0x8 + i2], (buffer[offset + 0x11 + i] & 0x0F) << 4));
                i2 += 2;
            }
            //let t = ((buffer[offset + 0x0D] as u16) << 8) | (buffer[offset + 0x0E] as u16);
            let temp_data = i16::from_be_bytes([buffer[offset + 0x0D], buffer[offset + 0x0E]]);
            avg_with_gain_i16(temp, temp_data);
            *timestamp = i16::from_be_bytes([buffer[offset + 0x0F], buffer[offset + 0x10]]);
            offset += PACKET_SIZE_BYTES;
            records_read += 1;
        }

        //fifo_reset(&mut comm)?;
        Ok(records_read)
    }

    pub fn temp_to_mdeg(temp: i16) -> i16 {
        (temp * 10000 / 13248 /*207 (8-bit)*/) + 2500
    }

    pub fn gyro_to_dps(value: i32) -> i32 {
        value / DPS_COEFF
    }

    pub fn accel_to_mg(value: i32) -> i32 {
        value / MG_COEFF
    }

    fn fifo_reset(&mut self, bus: &mut BE::Bus) -> Result<(), Error> {
        self.bus_endpoint.write_byte(bus, reg::bank0::SIGNAL_PATH_RESET, val::FIFO_FLUSH)
    }
}

const READ_BUFFER_RECORDS: usize = 32;
const PACKET_SIZE_BYTES: usize = 20;
const BUFFER_SIZE: usize = READ_BUFFER_RECORDS * PACKET_SIZE_BYTES;
const AVG_FACTOR_POW2: usize = 2;
const GYRO_BITS: usize = 19;
const ACCEL_BITS: usize = 18;

const DPS_COEFF: i32 = 131 * (1 << AVG_FACTOR_POW2);
const MG_COEFF: i32 = 8192 * (1 << AVG_FACTOR_POW2) / 1000;

fn assemble_20bit_gyro(msb: u8, lsb: u8, llsb: u8) -> i32 {
    if msb & 0b10000000 == 0 {
        i32::from_be_bytes([msb, lsb, llsb & 0b11100000, 0]) >> (32 - GYRO_BITS + AVG_FACTOR_POW2)
    } else {
        i32::from_be_bytes([msb, lsb, llsb & 0b11100000 | 0b00011111, 0xFF]) >> (32 - GYRO_BITS + AVG_FACTOR_POW2)
    }
}
fn assemble_20bit_accel(msb: u8, lsb: u8, llsb: u8) -> i32 {
    if msb & 0b10000000 == 0 {
        i32::from_be_bytes([msb, lsb, llsb & 0b11000000, 0]) >> (32 - ACCEL_BITS + AVG_FACTOR_POW2)
    } else {
        i32::from_be_bytes([msb, lsb, llsb & 0b11000000 | 0b00111111, 0xFF]) >> (32 - ACCEL_BITS + AVG_FACTOR_POW2)
    }
}

fn avg_with_gain_i32(state: &mut i32, update: i32) {
    if *state == 0i32 {
        *state = update
    } else {
        *state -= *state >> AVG_FACTOR_POW2; // gain 1/4
        *state += update >> AVG_FACTOR_POW2;
    }
}
fn avg_with_gain_i16(state: &mut i16, update: i16) {
    if *state == 0i16 {
        *state = update
    } else {
        *state -= *state >> AVG_FACTOR_POW2; // gain 1/4
        *state += update >> AVG_FACTOR_POW2;
    }
}

pub trait BusEndpoint {
    type Bus;
    fn read_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<u8, Error>;
    fn read_array<const N: usize>(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<[u8; N], Error>;
    fn read_to_buffer(&mut self, bus: &mut Self::Bus, reg_addr: u8, buffer: &mut [u8], length: usize) -> Result<(), Error>;
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
    fn read_to_buffer(&mut self, bus: &mut Self::Bus, reg_addr: u8, buffer: &mut [u8], length: usize) -> Result<(), Error> {
        bus.write_read(self.i2c_address, &[reg_addr], &mut buffer[..length])?;
        Ok(())
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

impl<CSPIN: OutputPin> BusEndpoint for SpiEndpoint<CSPIN> {
    type Bus = Spi;

    fn read_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<u8, Error> {
        self.cs_pin.set_low().map_err(comm_err)?;
        let mut buffer = [reg_addr | 0x80, 0u8];
        bus.transfer(&mut buffer).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(buffer[1])
    }
    fn read_array<const N: usize>(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<[u8; N], Error> {
        let mut buffer = [0u8; N];
        self.cs_pin.set_low().map_err(comm_err)?;
        block!(bus.send(reg_addr | 0x80)).map_err(comm_err)?;
        block!(bus.read()).map_err(comm_err)?;
        bus.transfer(&mut buffer).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(buffer)
    }
    fn read_to_buffer(&mut self, bus: &mut Self::Bus, reg_addr: u8, buffer: &mut [u8], length: usize) -> Result<(), Error> {
        self.cs_pin.set_low().map_err(comm_err)?;
        block!(bus.send(reg_addr | 0x80)).map_err(comm_err)?;
        block!(bus.read()).map_err(comm_err)?;
        bus.transfer(&mut buffer[..length]).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(())
    }
    fn write_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8, value: u8) -> Result<(), Error> {
        self.cs_pin.set_low().map_err(comm_err)?;
        bus.write(&[reg_addr, value]).map_err(comm_err)?;
        self.cs_pin.set_high().map_err(comm_err)?;
        Ok(())
    }

}

pub fn new_i2c(i2c: &mut I2c, i2c_address: u8) -> Result<Icm42688<I2cEndpoint>, Error> {
    Icm42688::init(I2cEndpoint { i2c_address }, i2c)
}

pub fn new_spi<CSPIN: OutputPin>(spi: &mut Spi, cs_pin: CSPIN) -> Result<Icm42688<SpiEndpoint<CSPIN>>, Error> {
    Icm42688::init(SpiEndpoint { cs_pin }, spi)
}
