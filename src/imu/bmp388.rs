use arduino_hal::delay_ms;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use embedded_hal::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex};
use core::fmt::Debug;

use crate::imu::comm::Comm;

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

/*
pub struct CalibrationData {
    par_t1  : f64,
    par_t2  : f64,
    par_t3  : f64,
    par_p1  : f64,
    par_p2  : f64,
    par_p3  : f64,
    par_p4  : f64,
    par_p5  : f64,
    par_p6  : f64,
    par_p7  : f64,
    par_p8  : f64,
    par_p9  : f64,
    par_p10 : f64,
    par_p11 : f64,
    t_lin   : f64
}
pub fn empty_calibration_data() -> CalibrationData {
    CalibrationData {
        par_t1  : 0f64,
        par_t2  : 0f64,
        par_t3  : 0f64,
        par_p1  : 0f64,
        par_p2  : 0f64,
        par_p3  : 0f64,
        par_p4  : 0f64,
        par_p5  : 0f64,
        par_p6  : 0f64,
        par_p7  : 0f64,
        par_p8  : 0f64,
        par_p9  : 0f64,
        par_p10 : 0f64,
        par_p11 : 0f64
    }
}
*/
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
pub fn empty_calibration_data() -> CalibrationData {
    CalibrationData {
        par_t1  : 0,
        par_t2  : 0,
        par_t3  : 0,
        par_p1  : 0,
        par_p2  : 0,
        par_p3  : 0,
        par_p4  : 0,
        par_p5  : 0,
        par_p6  : 0,
        par_p7  : 0,
        par_p8  : 0,
        par_p9  : 0,
        par_p10 : 0,
        par_p11 : 0
    }
}
pub fn init<C: Comm>(mut comm: C, calibration_data: &mut CalibrationData) -> Result<bool, <C as Comm>::Error> where <C as Comm>::Error: Debug {
    //chip id
    if comm.reg_read_byte_bmp388(reg::CHIP_ID)? != val::CHIP_ID {
        return Ok(false)
    }
    //reset
    let status = comm.reg_read_byte_bmp388(reg::STATUS)?;
    if status & val::STATUS_CMD_RDY_BIT == 0 {
        return Ok(false);
    }
    comm.reg_write_byte(reg::COMMAND, val::CMD_SOFT_RESET)?;
    delay_ms(2);
    let err = comm.reg_read_byte_bmp388(reg::ERROR)?;
    if err & val::ERROR_CMD_FAILED_BIT > 0 {
        return Ok(false);
    }
    // calibration data
    let buffer = comm.reg_read_array_bmp388::<21>(reg::CALIB_DATA)?;
    calibration_data.par_t1 = u16::from_le_bytes([buffer[0],buffer[1]]); // as f64) * 256.0f64;
    calibration_data.par_t2 = u16::from_le_bytes([buffer[2],buffer[3]]); // as f64) / 1073741824.0f64;
    calibration_data.par_t3 = i8::from_le_bytes([buffer[4]]); // as f64) / 281474976710656.0f64;
    calibration_data.par_p1 = i16::from_le_bytes([buffer[5],buffer[6]]); // - 16384) as f64 / 1048576.0;
    calibration_data.par_p2 = i16::from_le_bytes([buffer[7],buffer[8]]); // - 16384) as f64 / 536870912.0;
    calibration_data.par_p3 = i8::from_le_bytes([buffer[9]]); // as f64 / 4294967296.0;
    calibration_data.par_p4 = i8::from_le_bytes([buffer[10]]); // as f64 / 137438953472.0;
    calibration_data.par_p5 = u16::from_le_bytes([buffer[11],buffer[12]]); // as f64 * 8.0;
    calibration_data.par_p6 = u16::from_le_bytes([buffer[13],buffer[14]]); // as f64 / 64.0;
    calibration_data.par_p7 = i8::from_le_bytes([buffer[15]]); // as f64 / 256.0;
    calibration_data.par_p8 = i8::from_le_bytes([buffer[16]]); // as f64 / 32768.0;
    calibration_data.par_p9 = i16::from_le_bytes([buffer[17],buffer[18]]); // as f64 / 281474976710656.0;
    calibration_data.par_p10 = i8::from_le_bytes([buffer[19]]); // as f64 / 281474976710656.0;
    calibration_data.par_p11 = i8::from_le_bytes([buffer[20]]); // as f64 / 36893488147419103232.0;

    //set_config
    let pc = comm.reg_read_byte_bmp388(reg::PWR_CTRL)?;
    comm.reg_write_byte(reg::PWR_CTRL, pc | val::POWER_CONTROL_PRESS_ON | val::POWER_CONTROL_TEMP_ON | val::POWER_CONTROL_MODE_NORMAL)?;
    comm.reg_write_byte(reg::IIR_CONF, val::IIR_COEF_3)?;
    comm.reg_write_byte(reg::OSR, val::OSR_TEMP_X2 | val::OSR_PRESS_X16)?;
    comm.reg_write_byte(reg::ODR, val::ODR_25HZ)?;
    Ok(true)
}

pub fn get_data<C: Comm>(mut comm: C, pressure: &mut u32, temperature: &mut i32, timestamp: &mut u32, calibration_data: &CalibrationData) -> Result<bool, <C as Comm>::Error> where <C as Comm>::Error: Debug {
    let status = comm.reg_read_byte_bmp388(reg::STATUS)?;
    if status & val::STATUS_PRESS_RDY_BIT == 0 {
        return Ok(false);
    }
    let buffer = comm.reg_read_array_bmp388::<9>(reg::DATA)?;
    let raw_press = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], 0]);
    let raw_temp = u32::from_le_bytes([buffer[3], buffer[4], buffer[5], 0]);
    //let raw_temp = (buffer[3] as u32) | ((buffer[4] as u32) << 8) | ((buffer[5] as u32) << 16)
    *timestamp = u32::from_le_bytes([buffer[6], buffer[7], buffer[8], 0]);
    let t_lin = compensate_temp(raw_temp, calibration_data);
    *temperature = ((t_lin * 1000) >> 8) as i32; //millidegrees
    let p_lin = compensate_press(raw_press, t_lin, calibration_data);
    *pressure = (p_lin >> 8) as u32;
    //*temperature = raw_temp as i32;
    //*pressure = raw_press;
    Ok(true)
}

fn compensate_temp(raw_temp: u32, calibration_data: &CalibrationData) -> i64 {
    let partial_data1 = ((raw_temp as i64) - ((calibration_data.par_t1 as i64) << 8)) << 8;
    let partial_data2 = (partial_data1 * (calibration_data.par_t2 as i64) ) >> 30;
    let partial_data3 = (partial_data1 * partial_data1) >> 8;
    partial_data2 + (partial_data3 * (calibration_data.par_t3 as i64) >> 48)
}

fn compensate_press(raw_press: u32, t_lin: i64, calibration_data: &CalibrationData) -> i64 {
    let uncomp_press = raw_press as i64;
    let t_lin2 = (t_lin * t_lin) >> 8;
    let t_lin3 = (t_lin2 * t_lin) >> 8;
    let partial_out1 = {
        let partial_data1 = ((calibration_data.par_p6 as i64) * t_lin) >> 6;
        let partial_data2 = ((calibration_data.par_p7 as i64) * t_lin2) >> 8;
        let partial_data3 = ((calibration_data.par_p8 as i64) * t_lin3) >> 15;
        ((calibration_data.par_p5 as i64) << 11) + partial_data1 + partial_data2 + partial_data3
    };
    let partial_out2 = {
        let partial_data1 = ((calibration_data.par_p2 as i64 - 16384) * t_lin) >> 9;
        let partial_data2 = ((calibration_data.par_p3 as i64) * t_lin2) >> 12;
        let partial_data3 = ((calibration_data.par_p4 as i64) * t_lin3) >> 17;
        uncomp_press * (((calibration_data.par_p1 as i64) - 16384) << 8) + partial_data1 + partial_data2 + partial_data3
    };
    let partial_data4 = {
        let up2 = (uncomp_press * uncomp_press) >> 8;
        let partial_data2 = calibration_data.par_p9 as i64 + ((calibration_data.par_p10 as i64 * t_lin) >> 8);
        let partial_data3 = up2 * partial_data2;
        let up3 = (up2 * uncomp_press) >> 8;
        partial_data3 + ((up3 * (calibration_data.par_p11 as i64)) >> 17)
    };
    partial_out1 + (partial_out2 >> 20) + (partial_data4 >> 32)
}
/*
trait Interface {
    type Comm;
    fn reg_read_byte(&mut self, reg_addr: u8) -> Result<u8, Self::Error>;
    fn reg_read_array<const N: usize>(&mut self, reg_addr: u8) -> Result<[u8; N], Self::Error>;
    fn reg_write_byte(&mut self, reg_addr: u8, value: u8) -> Result<(), Self::Error>;
}

struct I2c {
    address: u8
}

impl Interface for I2c {
    type Comm = I2c;
}
*/
