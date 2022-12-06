use arduino_hal::delay_ms;
use crate::imu::comm::Comm;

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

pub fn check<C: Comm>(mut comm: C, whoami: &mut u8) -> Result<bool, <C as Comm>::Error> {
    //comm.reg_write_byte(reg::REG_BANK_SEL, 0);
    *whoami = comm.reg_read_byte(reg::bank0::WHO_AM_I)?;
    Ok(*whoami == val::WHO_AM_I)
}

pub fn init<C: Comm>(mut comm: C) -> Result<bool, <C as Comm>::Error> {
    // Software reset
    comm.reg_write_byte(reg::bank0::DEVICE_CONFIG, val::DEVCONFIG_SOFT_RESET_ENABLE)?;
    delay_ms(1);

    loop {
        if comm.reg_read_byte(reg::bank0::WHO_AM_I)? == val::WHO_AM_I
            && comm.reg_read_byte(reg::bank0::DEVICE_CONFIG)? == 0x00
            && comm.reg_read_byte(reg::bank0::INT_STATUS)? & val::RESET_DONE_INT != 0
        {
            break;
        }
        delay_ms(10);
    }

    // sensors config
    comm.reg_write_byte(reg::bank0::ACCEL_CONFIG_0, val::ACFG0_FS_SEL_16G     | val::ODR_4KHZ)?;
    comm.reg_write_byte(reg::bank0::GYRO_CONFIG_0,  val::GCFG0_FS_SEL_2000DPS | val::ODR_4KHZ)?;

    //comm.reg_write_byte(reg::bank0::DRIVE_CONFIG, val::DRIVECONFIG_MIN_SLEW_RATE)?;
    let pwr_mgmt = val::PWR_GYRO_MODE_LOW_NOISE | val::PWR_ACCEL_MODE_LOW_NOISE;
    comm.reg_write_byte(reg::bank0::PWR_MGMT_0, pwr_mgmt)?;
    delay_ms(45);
    if comm.reg_read_byte(reg::bank0::PWR_MGMT_0)? != pwr_mgmt {
        return Ok(false);
    }
    // FIFO config
    let fifo_config_1 = val::FIFOCONFIG1_HIRES_EN; // | val::FIFOCONFIG1_TEMP_EN | val::FIFOCONFIG1_GYRO_EN | val::FIFOCONFIG1_ACCEL_EN;
    comm.reg_write_byte(reg::bank0::FIFO_CONFIG_1, fifo_config_1)?;
    if comm.reg_read_byte(reg::bank0::FIFO_CONFIG_1)? != fifo_config_1 {
        return Ok(false);
    }
    comm.reg_write_byte(reg::bank0::FIFO_CONFIG, val::FIFOCONFIG_STOP_ON_FULL_MODE/* val::FIFOCONFIG_STREAM_TO_FIFO_MODE*/)?;
    delay_ms(100);
    fifo_reset(&mut comm)?;
    Ok(true)
}

const READ_BUFFER_RECORDS: usize = 20;
const PACKET_SIZE_BYTES: usize = 20;
const BUFFER_SIZE: usize = READ_BUFFER_RECORDS * PACKET_SIZE_BYTES;
const AVG_FACTOR_POW2: usize = 2;
const GYRO_BITS: usize = 19;
const ACCEL_BITS: usize = 18;

fn assemble_20bit_gyro(msb: u8, lsb: u8, llsb: u8) -> i32 {
    if msb & 0b10000000 == 0 {
        i32::from_be_bytes([msb,lsb,llsb & 0b11100000, 0]) >> (32-GYRO_BITS+AVG_FACTOR_POW2)
    } else {
        i32::from_be_bytes([msb,lsb,llsb & 0b11100000 | 0b00011111, 0xFF]) >> (32-GYRO_BITS+AVG_FACTOR_POW2)
    }
}
fn assemble_20bit_accel(msb: u8, lsb: u8, llsb: u8) -> i32 {
    if msb & 0b10000000 == 0 {
        i32::from_be_bytes([msb,lsb,llsb & 0b11000000, 0]) >> (32-ACCEL_BITS+AVG_FACTOR_POW2)
    } else {
        i32::from_be_bytes([msb,lsb,llsb & 0b11000000 | 0b00111111, 0xFF]) >> (32-ACCEL_BITS+AVG_FACTOR_POW2)
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

pub fn get_data<C: Comm>(mut comm: C, accel: &mut [i32;3], gyro: &mut [i32;3], temp: &mut i16, timestamp: &mut i16) -> Result<u16, <C as Comm>::Error> {

    let mut buffer = [0u8; BUFFER_SIZE];
    let mut records_read = 0u16;


//    loop {
        let fifo_count = comm.reg_read_array::<2>(reg::bank0::FIFO_COUNTH)?;
        let bytes_available = u16::from_be_bytes(fifo_count);
        if bytes_available < PACKET_SIZE_BYTES as u16 {
            return Ok(0u16);
        }
        let mut records_to_read: usize = bytes_available as usize / PACKET_SIZE_BYTES;
        if records_to_read > READ_BUFFER_RECORDS {
            records_to_read = READ_BUFFER_RECORDS;
        }
        comm.reg_read_to_buffer(reg::bank0::FIFO_DATA, &mut buffer, records_to_read * PACKET_SIZE_BYTES)?;
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
            avg_with_gain_i16( temp, temp_data);
            *timestamp = i16::from_be_bytes([buffer[offset + 0x0F], buffer[offset + 0x10]]);
            offset += PACKET_SIZE_BYTES;
            records_read += 1;
        }
//    }
    //fifo_reset(&mut comm)?;
    Ok(records_read)

    /*
    if bytes_available == 0 || bytes_available > 2080 {
         return Ok(false);
    }
    //comm.reg_read_to_buffer(reg::bank0::FIFO_DATA, &mut buffer, bytes_available as usize)?;
    //if wait_for_reg(&mut comm, reg::bank0::INT_STATUS, val::DATA_RDY_INT, val::DATA_RDY_INT, 10000000)? {
        let buffer = comm.reg_read_array::<20>(reg::bank0::FIFO_DATA)?;
        if buffer[0] & 0b10000000 == 0 { //buffer has data
            comm.reg_write_byte(reg::bank0::SIGNAL_PATH_RESET, val::FIFO_FLUSH)?;

        //if buffer[0] & 0xFC == val::FIFOHEADER_ACCEL | val::FIFOHEADER_GYRO | val::FIFOHEADER_TIMESTAMP_ODR {
            for i in 0..3 {
                let i2 = i << 1;
                accel[i] = assemble_20bit(buffer[0x1 + i2], buffer[0x2 + i2], buffer[0x11 + i]); //hi-res
                gyro[i] = assemble_20bit(buffer[0x7 + i2], buffer[0x8 + i2], buffer[0x11 + i] << 4); //hi-res
            }
            *temp = u16::from_be_bytes([buffer[0x0D], buffer[0x0E]]); //hi-res
            // *temp = buffer[0x0D] as u16;
            // *temp = ((buffer[0x0D] as u16) << 8) | (buffer[0x0E] as u16);
            // *temp = (buffer[buffer_idx] as u16) << 8;
            // buffer_idx += 1;
            // *temp |= buffer[16] as u16;
            // buffer_idx += 1;
            // *timestamp = u16::from_be_bytes([buffer[0x0F], buffer[0x10]]); //hi-res
            // *timestamp = u16::from_be_bytes([buffer[0x0E], buffer[0x0F]]);
            // *timestamp = (buffer[buffer_idx] as u16) << 8;
            // buffer_idx += 1;
            // *timestamp |= buffer[buffer_idx] as u16;
            // buffer_idx += 1;
            // for i in 0..3 {
            //     accel[i] <<= 4;
            //     accel[i] |= (buffer[buffer_idx] as i32) & 0xF;
            //     gyro[i] <<= 4;
            //     gyro[i] |= ((buffer[buffer_idx] as i32) & 0xF0 >> 4);
            //     buffer_idx += 1;
            // }
            return Ok(true);
        }
        return Ok(false);
    //}
    return Ok(false);

     */
}

pub fn temp_to_mdeg(temp: i16) -> i16 {
    (temp * 10000 / 13248 /*207 (8-bit)*/) + 2500
}

const DPS_COEFF: i32 = 131 * (1 << AVG_FACTOR_POW2);
const MG_COEFF: i32 = 8192 * (1 << AVG_FACTOR_POW2) / 1000;

pub fn gyro_to_dps(value: i32) -> i32 {
    value / DPS_COEFF
}

pub fn accel_to_mg(value: i32) -> i32 {
    value / MG_COEFF
}

fn fifo_reset<C: Comm>(comm: &mut C) -> Result<(), <C as Comm>::Error> {
    comm.reg_write_byte(reg::bank0::SIGNAL_PATH_RESET, val::FIFO_FLUSH)
}

/*
fn wait_for_reg<C: Comm>(comm: &mut C, reg: u8, mask: u8, expect: u8, max_timeout_us: u32) -> Result<bool, <C as Comm>::Error> {
    let mut cnt = 0u32;
    loop {
        // let status = comm.reg_read_byte(reg::bank0::INT_STATUS)?;
        // if status & val::INTSTATUS_FIFO_THS_INT != 0 {
        let status = comm.reg_read_byte(reg)?;
        if status & mask == expect {
            return Ok(true);
        } else {
            delay_us(1);
            cnt += 1;
            if cnt > max_timeout_us {
                return Ok(false);
            }
        }
    }
}
 */

