use arduino_hal::delay_ms;
use crate::imu::comm::Comm;
use core::fmt::Debug;

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
    digX1  : i8,  // trim x1 data
    digY1  : i8,  // trim y1 data
    digX2  : i8,  // trim x2 data
    digY2  : i8,  // trim y2 data
    digZ1  : u16, // trim z1 data
    digZ2  : i16, // trim z2 data
    digZ3  : i16, // trim z3 data
    digZ4  : i16, // trim z4 data
    digXY1 : u8,  // trim xy1 data
    digXY2 : i8,  // trim xy2 data
    digXYZ1: u16  // trim xyz1 data
}
pub fn empty_trim_data() -> TrimData {
    TrimData {
        digX1  : 0,
        digY1  : 0,
        digX2  : 0,
        digY2  : 0,
        digZ1  : 0,
        digZ2  : 0,
        digZ3  : 0,
        digZ4  : 0,
        digXY1 : 0,
        digXY2 : 0,
        digXYZ1: 0
    }
}

pub fn init<C: Comm>(mut comm: C, trim_data: &mut TrimData) -> Result<bool, <C as Comm>::Error> where <C as Comm>::Error: Debug {
    comm.reg_write_byte(reg::POWER_CONTROL, 0)?;
    comm.reg_write_byte(reg::POWER_CONTROL, val::POWER_CONTROL_ON)?;
    delay_ms(100);
    if comm.reg_read_byte(reg::CHIP_ID).unwrap() != val::CHIP_ID {
        return Ok(false)
    }
    let o1 = comm.reg_read_byte(reg::OP_MODE)?;
    comm.reg_write_byte(reg::OP_MODE, (o1 & val::OP_MODE_MSK & val::OP_MODE_ODR_MSK) | val::OP_MODE_NORMAL | val::OP_MODE_ODR_20HZ)?;

    comm.reg_write_byte(reg::REP_XY, val::REPXY_HIGH_ACC)?;
    comm.reg_write_byte(reg::REP_Z, val::REPZ_HIGH_ACC)?;


    let int_axes_set = comm.reg_read_byte(reg::INT_AXES_SET)?;
    let enable_all_axes_msk = !(val::DISABLE_X_AXIS | val::DISABLE_Y_AXIS | val::DISABLE_Z_AXIS);
    comm.reg_write_byte(reg::INT_AXES_SET, int_axes_set & enable_all_axes_msk)?;

    if comm.reg_read_byte(reg::OP_MODE)? & 0b00111110 != val::OP_MODE_NORMAL | val::OP_MODE_ODR_20HZ {
        return Ok(false);
    }

    // fetch trim data
    let trim_x1y1 = comm.reg_read_array::<2>(reg::DIG_X1)?;
    let trim_xyxdata = comm.reg_read_array::<4>(reg::DIG_Z4_LSB)?;
    let trim_xy1xy2 = comm.reg_read_array::<10>(reg::DIG_Z2_LSB)?;
    trim_data.digX1 = i8::from_le_bytes([trim_x1y1[0]]);
    trim_data.digY1 = i8::from_le_bytes([trim_x1y1[1]]);
    trim_data.digX2 = i8::from_le_bytes([trim_xyxdata[2]]);
    trim_data.digY2 = i8::from_le_bytes([trim_xyxdata[3]]);
    trim_data.digZ1 = u16::from_le_bytes([trim_xy1xy2[2], trim_xy1xy2[3]]);
    trim_data.digZ2 = i16::from_le_bytes([trim_xy1xy2[0], trim_xy1xy2[1]]);
    trim_data.digZ3 = i16::from_le_bytes([trim_xy1xy2[6], trim_xy1xy2[7]]);
    trim_data.digZ4 = i16::from_le_bytes([trim_xyxdata[0], trim_xyxdata[1]]);
    trim_data.digXY1 = trim_xy1xy2[9];
    trim_data.digXY2 = i8::from_le_bytes([trim_xy1xy2[8]]);
    trim_data.digXYZ1 = u16::from_le_bytes([trim_xy1xy2[4], trim_xy1xy2[5] & 0x7F]);

    Ok(true)
}



pub fn get_data<C: Comm>(mut comm: C, mag_data: &mut [i16;3], trim_data: &TrimData) -> Result<bool, <C as Comm>::Error> {
    let buffer = comm.reg_read_array::<8>(reg::DATA_X_LSB)?;
    if buffer[6] & val::DRDY_BIT_MASK > 0 {
        let raw_x = i16::from_le_bytes([buffer[0] & 0b11111000, buffer[1]]) >> 3;
        let raw_y = i16::from_le_bytes([buffer[2] & 0b11111000, buffer[3]]) >> 3;
        let raw_z = i16::from_le_bytes([buffer[4] & 0b11111000, buffer[5]]) >> 1;
        let rhall = u16::from_le_bytes([buffer[6], buffer[7]]) >> 2;
        Ok(compensate_x(raw_x, rhall, trim_data, &mut mag_data[0])
             && compensate_y(raw_y, rhall, trim_data, &mut mag_data[1])
             && compensate_z(raw_z, rhall, trim_data, &mut mag_data[2]))
    } else {
        Ok(false)
    }
}

fn compensate_x(raw_x: i16, rhall: u16, trim_data: &TrimData, x_val: &mut i16) -> bool {
    /* Overflow condition check */
    if raw_x != OVERFLOW_ADCVAL_XYAXES_FLIP {
        let process_comp_x0 = if rhall != 0 {
            /* Availability of valid data */
            rhall as i32
        } else if trim_data.digXYZ1 != 0 {
            trim_data.digXYZ1 as i32
        } else {
            0
        };
        if process_comp_x0 != 0 {
            /* Processing compensation equations */
            let process_comp_x1 = (trim_data.digXYZ1 as i32) * 16384;
            let process_comp_x2 = ((process_comp_x1 / process_comp_x0) as u16) - 0x4000;
            let retval32 = process_comp_x2 as i32;
            let process_comp_x3 = retval32 * retval32;
            let process_comp_x4 = ((trim_data.digXY2 as i32) * (process_comp_x3 / 128));
            let process_comp_x5 = ((trim_data.digXY1 as i16) * 128) as i32;
            let process_comp_x6 = retval32 * process_comp_x5;
            let process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + 0x100000);
            let process_comp_x8 = ((trim_data.digX2 as i16) + 0xA0) as i32;
            let process_comp_x9 = (process_comp_x7 * process_comp_x8) / 4096;
            let process_comp_x10 = (raw_x as i32) * process_comp_x9;
            *x_val = ((process_comp_x10 / 8192) as i16 + ((trim_data.digX1 as i16) * 8) / 16);
            return true;
        } else {
            return false;
        }
    }else{
        /* Overflow condition */
        return false;
    }
}

fn compensate_y(raw_y: i16, rhall: u16, trim_data: &TrimData, y_val: &mut i16) -> bool {
    /* Overflow condition check */
    if raw_y != OVERFLOW_ADCVAL_XYAXES_FLIP {
        let process_comp_y0 = if rhall != 0 {
            /* Availability of valid data */
            rhall
        } else if trim_data.digXYZ1 != 0 {
            trim_data.digXYZ1
        } else {
            0
        };
        if process_comp_y0 != 0 {
            /* Processing compensation equations */
            let process_comp_y1 = ((trim_data.digXYZ1 as i32) * 16384) / (process_comp_y0 as i32);
            let process_comp_y2 = (process_comp_y1 as u16) - 0x4000;
            let retval32 = process_comp_y2 as i32;
            let process_comp_y3 = retval32 * retval32;
            let process_comp_y4 = (trim_data.digXY2 as i32) * (process_comp_y3 / 128);
            let process_comp_y5 = ((trim_data.digXY1 as i16) * 128) as i32;
            let process_comp_y6 = (process_comp_y4 + (retval32 * process_comp_y5)) / 512;
            let process_comp_y7 = ((trim_data.digY2 as i16) + 0xA0) as i32;
            let process_comp_y8 = ((process_comp_y6 + 0x100000) * process_comp_y7) / 4096;
            let process_comp_y9 = (raw_y as i32) * process_comp_y8;
            *y_val = ((process_comp_y9 / 8192) as i16 + ((trim_data.digY1 as i16) * 8)) / 16;
            return true;
        } else {
            return false;
        }
    } else {
        /* Overflow condition */
        return false;
    }

}

fn compensate_z(raw_z: i16, rhall: u16, trim_data: &TrimData, z_val: &mut i16) -> bool {
    if raw_z != OVERFLOW_ADCVAL_ZAXIS_HALL {
        if (trim_data.digZ2 != 0) && (trim_data.digZ1 != 0) && (rhall != 0) &&(trim_data.digXYZ1 != 0) {
            /*Processing compensation equations */
            let process_comp_z0 = (rhall as i16) - ( trim_data.digXYZ1 as i16);
            let process_comp_z1 = ((trim_data.digZ3 as i32) * (process_comp_z0 as i32)) / 4;
            let process_comp_z2 = (((raw_z - trim_data.digZ4) as i32) * 32768);
            let process_comp_z3 = (trim_data.digZ1 as i32) * ((rhall as i32) * 2);
            let process_comp_z4 = ((process_comp_z3 + (32768)) / 65536) as i32;
            let mut retval = ((process_comp_z2 - process_comp_z1) / (trim_data.digZ2 as i32 + process_comp_z4));

            /* Saturate result to +/- 2 micro-tesla */
            if retval > POSITIVE_SATURATION_Z {
                retval = POSITIVE_SATURATION_Z;
            } else if retval < NEGATIVE_SATURATION_Z {
                retval = NEGATIVE_SATURATION_Z;
            }
            /* Conversion of LSB to micro-tesla */
            *z_val = (retval / 16) as i16;
            return true;
        }else{
            return false;
        }
    }else{
        /* Overflow condition */
        return false;
    }

}
