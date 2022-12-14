use arduino_hal::{delay_ms, I2c, Spi};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use embedded_hal::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex};
use core::fmt::Debug;
use nb::block;


pub const I2C_DEFAULT_ADDR: u8 = 0x13;

pub mod reg {
    pub const CHIP_ID       : u8 = 0x40;
    pub const DATA_X_LSB    : u8 = 0x42;
    pub const DATA_X_MSB    : u8 = 0x43;
    pub const DATA_Y_LSB    : u8 = 0x44;
    pub const DATA_Y_MSB    : u8 = 0x45;
    pub const DATA_Z_LSB    : u8 = 0x46;
    pub const DATA_Z_MSB    : u8 = 0x47;
    pub const RHALL_LSB_DRDY: u8 = 0x48;
    pub const RHALL_MSB     : u8 = 0x49;
    pub const INT_STATUS    : u8 = 0x4A;
    pub const POWER_CONTROL : u8 = 0x4B;
    pub const OP_MODE       : u8 = 0x4C;
    pub const INT_AXES_SET  : u8 = 0x4E;
    pub const REP_XY        : u8 = 0x51;
    pub const REP_Z         : u8 = 0x52;
    
    pub const DIG_X1       : u8 = 0x5D;
    pub const DIG_Y1       : u8 = 0x5E;
    pub const DIG_Z4_LSB   : u8 = 0x62;
    pub const DIG_Z4_MSB   : u8 = 0x63;
    pub const DIG_X2       : u8 = 0x64;
    pub const DIG_Y2       : u8 = 0x65;
    pub const DIG_Z2_LSB   : u8 = 0x68;
    pub const DIG_Z2_MSB   : u8 = 0x69;
    pub const DIG_Z1_LSB   : u8 = 0x6A;
    pub const DIG_Z1_MSB   : u8 = 0x6B;
    pub const DIG_XYZ1_LSB : u8 = 0x6C;
    pub const DIG_XYZ1_MSB : u8 = 0x6D;
    pub const DIG_Z3_LSB   : u8 = 0x6E;
    pub const DIG_Z3_MSB   : u8 = 0x6F;
    pub const DIG_XY2      : u8 = 0x70;
    pub const DIG_XY1      : u8 = 0x71;

}

pub mod val {
    pub const CHIP_ID           : u8 = 0x32;
    pub const POWER_CONTROL_ON  : u8 = 0b00000001;
    pub const OP_MODE_MSK       : u8 = 0b11111001;
    pub const OP_MODE_NORMAL    : u8 = 0b00000000;
    pub const OP_MODE_ODR_MSK   : u8 = 0b11000111;
    pub const OP_MODE_ODR_10HZ  : u8 = 0b00000000;
    pub const OP_MODE_ODR_20HZ  : u8 = 0b00101000; // max for high accuracy
    pub const OP_MODE_ODR_30HZ  : u8 = 0b00111000;
    pub const DISABLE_X_AXIS    : u8 = 0b00001000;
    pub const DISABLE_Y_AXIS    : u8 = 0b00010000;
    pub const DISABLE_Z_AXIS    : u8 = 0b00100000;
    pub const DRDY_BIT_MASK     : u8 = 0b00000001;

    pub const REPXY_HIGH_ACC    : u8 = 0x17; // 47 reps
    pub const REPZ_HIGH_ACC     : u8 = 0x29; // 83 reps

}

const OVERFLOW_ADCVAL_XYAXES_FLIP: i16 = -4096;
const OVERFLOW_ADCVAL_ZAXIS_HALL : i16 = -16384;
const NEGATIVE_SATURATION_Z      : i32 = -32767;
const POSITIVE_SATURATION_Z      : i32 =  32767;

pub struct TrimData {
    dig_x1  : i8,  // trim x1 data
    dig_y1  : i8,  // trim y1 data
    dig_x2  : i8,  // trim x2 data
    dig_y2  : i8,  // trim y2 data
    dig_z1  : u16, // trim z1 data
    dig_z2  : i16, // trim z2 data
    dig_z3  : i16, // trim z3 data
    dig_z4  : i16, // trim z4 data
    dig_xy1 : u8,  // trim xy1 data
    dig_xy2 : i8,  // trim xy2 data
    dig_xyz1: u16  // trim xyz1 data
}

pub struct Bmm150<BE: BusEndpoint> {
    bus_endpoint: BE,
    trim_data: TrimData
}

#[derive(Debug)]
pub enum Error {
    CommError,
    SensorError
}

impl<BE: BusEndpoint> Bmm150<BE> {

    pub fn init(mut bus_endpoint: BE, bus: &mut BE::Bus) -> Result<Bmm150<BE>, Error> {
        bus_endpoint.write_byte(bus, reg::POWER_CONTROL, 0)?;
        bus_endpoint.write_byte(bus, reg::POWER_CONTROL, val::POWER_CONTROL_ON)?;
        delay_ms(100);
        if bus_endpoint.read_byte(bus, reg::CHIP_ID).unwrap() != val::CHIP_ID {
            return Err(Error::SensorError)
        }
        let o1 = bus_endpoint.read_byte(bus, reg::OP_MODE)?;
        bus_endpoint.write_byte(bus, reg::OP_MODE, (o1 & val::OP_MODE_MSK & val::OP_MODE_ODR_MSK) | val::OP_MODE_NORMAL | val::OP_MODE_ODR_20HZ)?;

        bus_endpoint.write_byte(bus, reg::REP_XY, val::REPXY_HIGH_ACC)?;
        bus_endpoint.write_byte(bus, reg::REP_Z, val::REPZ_HIGH_ACC)?;


        let int_axes_set = bus_endpoint.read_byte(bus, reg::INT_AXES_SET)?;
        let enable_all_axes_msk = !(val::DISABLE_X_AXIS | val::DISABLE_Y_AXIS | val::DISABLE_Z_AXIS);
        bus_endpoint.write_byte(bus, reg::INT_AXES_SET, int_axes_set & enable_all_axes_msk)?;

        if bus_endpoint.read_byte(bus, reg::OP_MODE)? & 0b00111110 != val::OP_MODE_NORMAL | val::OP_MODE_ODR_20HZ {
            return Err(Error::SensorError);
        }

        // fetch trim data
        let trim_x1y1 = bus_endpoint.read_array::<2>(bus, reg::DIG_X1)?;
        let trim_xyxdata = bus_endpoint.read_array::<4>(bus, reg::DIG_Z4_LSB)?;
        let trim_xy1xy2 = bus_endpoint.read_array::<10>(bus, reg::DIG_Z2_LSB)?;
        let trim_data = TrimData {
            dig_x1  : i8::from_le_bytes([trim_x1y1[0]]),
            dig_y1  : i8::from_le_bytes([trim_x1y1[1]]),
            dig_x2  : i8::from_le_bytes([trim_xyxdata[2]]),
            dig_y2  : i8::from_le_bytes([trim_xyxdata[3]]),
            dig_z1  : u16::from_le_bytes([trim_xy1xy2[2], trim_xy1xy2[3]]),
            dig_z2  : i16::from_le_bytes([trim_xy1xy2[0], trim_xy1xy2[1]]),
            dig_z3  : i16::from_le_bytes([trim_xy1xy2[6], trim_xy1xy2[7]]),
            dig_z4  : i16::from_le_bytes([trim_xyxdata[0], trim_xyxdata[1]]),
            dig_xy1 : trim_xy1xy2[9],
            dig_xy2 : i8::from_le_bytes([trim_xy1xy2[8]]),
            dig_xyz1: u16::from_le_bytes([trim_xy1xy2[4], trim_xy1xy2[5] & 0x7F])
        };

        Ok(Bmm150 {
            bus_endpoint,
            trim_data
        })
    }



    pub fn get_data(&mut self, bus: &mut BE::Bus, mag_data: &mut [i16;3]) -> Result<bool, Error> {
        let buffer = self.bus_endpoint.read_array::<8>(bus, reg::DATA_X_LSB)?;
        if buffer[6] & val::DRDY_BIT_MASK > 0 {
            let raw_x = i16::from_le_bytes([buffer[0] & 0b11111000, buffer[1]]) >> 3;
            let raw_y = i16::from_le_bytes([buffer[2] & 0b11111000, buffer[3]]) >> 3;
            let raw_z = i16::from_le_bytes([buffer[4] & 0b11111000, buffer[5]]) >> 1;
            let rhall = u16::from_le_bytes([buffer[6], buffer[7]]) >> 2;
            Ok(self.compensate_x(raw_x, rhall, &mut mag_data[0])
                 && self.compensate_y(raw_y, rhall, &mut mag_data[1])
                 && self.compensate_z(raw_z, rhall, &mut mag_data[2]))
        } else {
            Ok(false)
        }
    }

    fn compensate_x(&self, raw_x: i16, rhall: u16, x_val: &mut i16) -> bool {
        /* Overflow condition check */
        if raw_x != OVERFLOW_ADCVAL_XYAXES_FLIP {
            let process_comp_x0 = if rhall != 0 {
                /* Availability of valid data */
                rhall as i32
            } else if self.trim_data.dig_xyz1 != 0 {
                self.trim_data.dig_xyz1 as i32
            } else {
                0i32
            };
            if process_comp_x0 != 0 {
                /* Processing compensation equations */
                let process_comp_x1 = (self.trim_data.dig_xyz1 as i32) << 14;
                let process_comp_x2 = (process_comp_x1 / process_comp_x0) - 0x4000;
                let process_comp_x3 = process_comp_x2 * process_comp_x2;
                let process_comp_x4 = (self.trim_data.dig_xy2 as i32) * (process_comp_x3 >> 7);
                let process_comp_x5 = (self.trim_data.dig_xy1 as i32) << 7;
                let process_comp_x6 = process_comp_x2 * process_comp_x5;
                let process_comp_x7 = ((process_comp_x4 + process_comp_x6) >> 9) + 0x100000;
                let process_comp_x8 = (self.trim_data.dig_x2 as i32) + 0xA0;
                let process_comp_x9 = (process_comp_x7 * process_comp_x8) >> 12;
                let process_comp_x10 = (raw_x as i32) * process_comp_x9;
                *x_val = (((process_comp_x10 >> 13) + ((self.trim_data.dig_x1 as i32) << 3)) >> 4) as i16;
                return true;
            } else {
                return false;
            }
        }else{
            /* Overflow condition */
            return false;
        }
    }

    fn compensate_y(&self, raw_y: i16, rhall: u16, y_val: &mut i16) -> bool {
        /* Overflow condition check */
        if raw_y != OVERFLOW_ADCVAL_XYAXES_FLIP {
            let process_comp_y0 = if rhall != 0 {
                /* Availability of valid data */
                rhall
            } else if self.trim_data.dig_xyz1 != 0 {
                self.trim_data.dig_xyz1
            } else {
                0
            };
            if process_comp_y0 != 0 {
                /* Processing compensation equations */
                let process_comp_y1 = ((self.trim_data.dig_xyz1 as i32) << 14) / (process_comp_y0 as i32);
                let process_comp_y2 = process_comp_y1 - 0x4000;
                let process_comp_y3 = process_comp_y2 * process_comp_y2;
                let process_comp_y4 = (self.trim_data.dig_xy2 as i32) * (process_comp_y3 >> 7);
                let process_comp_y5 = ((self.trim_data.dig_xy1 as i16) * 128) as i32;
                let process_comp_y6 = (process_comp_y4 + (process_comp_y2 * process_comp_y5)) >> 9;
                let process_comp_y7 = ((self.trim_data.dig_y2 as i16) + 0xA0) as i32;
                let process_comp_y8 = ((process_comp_y6 + 0x100000) * process_comp_y7) >> 12;
                let process_comp_y9 = (raw_y as i32) * process_comp_y8;
                *y_val = ((process_comp_y9 >> 13) as i16 + ((self.trim_data.dig_y1 as i16) << 3)) >> 4;
                return true;
            } else {
                return false;
            }
        } else {
            /* Overflow condition */
            return false;
        }

    }

    fn compensate_z(&self, raw_z: i16, rhall: u16, z_val: &mut i16) -> bool {
        if raw_z != OVERFLOW_ADCVAL_ZAXIS_HALL {
            if (self.trim_data.dig_z2 != 0) && (self.trim_data.dig_z1 != 0) && (rhall != 0) &&(self.trim_data.dig_xyz1 != 0) {
                /*Processing compensation equations */
                let process_comp_z0 = (rhall as i16) - ( self.trim_data.dig_xyz1 as i16);
                let process_comp_z1 = ((self.trim_data.dig_z3 as i32) * (process_comp_z0 as i32)) >> 2;
                let process_comp_z2 = ((raw_z - self.trim_data.dig_z4) as i32) << 15;
                let process_comp_z3 = (self.trim_data.dig_z1 as i32) * ((rhall as i32) << 1);
                let process_comp_z4 = ((process_comp_z3 + (32768)) >> 16) as i32;
                let mut retval = (process_comp_z2 - process_comp_z1) / (self.trim_data.dig_z2 as i32 + process_comp_z4);

                /* Saturate result to +/- 2 micro-tesla */
                if retval > POSITIVE_SATURATION_Z {
                    retval = POSITIVE_SATURATION_Z;
                } else if retval < NEGATIVE_SATURATION_Z {
                    retval = NEGATIVE_SATURATION_Z;
                }
                /* Conversion of LSB to micro-tesla */
                *z_val = (retval >> 4) as i16;
                return true;
            }else{
                return false;
            }
        }else{
            /* Overflow condition */
            return false;
        }

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

impl From<void::Void> for Error {
    fn from(_: void::Void) -> Error {
        Error::CommError
    }
}

impl<CSPIN: OutputPin> BusEndpoint for SpiEndpoint<CSPIN> where <CSPIN as OutputPin>::Error: Debug {
    type Bus = Spi;

    fn read_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<u8, Error> {
        self.cs_pin.set_low().unwrap();
        let mut buffer = [reg_addr | 0x80, 0u8];
        bus.transfer(&mut buffer)?;
        self.cs_pin.set_high().unwrap();
        Ok(buffer[1])
    }
    fn read_array<const N: usize>(&mut self, bus: &mut Self::Bus, reg_addr: u8) -> Result<[u8; N], Error> {
        let mut buffer = [0u8; N];
        self.cs_pin.set_low().unwrap();
        block!(bus.send(reg_addr | 0x80))?;
        block!(bus.read())?;
        bus.transfer(&mut buffer)?;
        self.cs_pin.set_high().unwrap();
        Ok(buffer)
    }
    fn write_byte(&mut self, bus: &mut Self::Bus, reg_addr: u8, value: u8) -> Result<(), Error> {
        self.cs_pin.set_low().unwrap();
        bus.write(&[reg_addr, value])?;
        self.cs_pin.set_high().unwrap();
        Ok(())
    }

}

pub fn new_i2c(i2c: &mut I2c, i2c_address: u8) -> Result<Bmm150<I2cEndpoint>, Error> {
    Bmm150::init(I2cEndpoint { i2c_address }, i2c)
}

pub fn new_spi<CSPIN: OutputPin>(spi: &mut Spi, cs_pin: CSPIN) -> Result<Bmm150<SpiEndpoint<CSPIN>>, Error> where <CSPIN as OutputPin>::Error: Debug {
    Bmm150::init(SpiEndpoint { cs_pin }, spi)
}
