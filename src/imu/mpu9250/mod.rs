pub mod gyro;
pub mod mag;

// MPU9250/9255 MPU sensor
use arduino_hal::I2c;
use crate::imu::i2c::I2cRegExt;

/**
 * @brief  Initializes MPU9250
 * @param  None
 * @retval None
 */
pub fn init(i2c: &mut I2c)
{
    /* Reset device. */
    // i2c.reg_write_byte(gyro::I2C_ADDRESS, gyro::REG_PWR_MGMT_1, gyro::BIT_RESET);
    // delay_ms(100);
    /* Wake up chip. */
    //i2c.reg_write_byte(gyro::I2C_ADDRESS, gyro::REG_PWR_MGMT_1, 0x00);
    i2c.reg_write_byte(gyro::I2C_ADDRESS, gyro::REG_USER_CTRL, 0x00);

    enable_i2c_bypass(i2c);

    //i2c.reg_write_byte(mag::I2C_ADDRESS, mag::REG_CNTL, mag::AKM_POWER_DOWN);

}

pub fn enable_i2c_bypass(i2c: &mut I2c) {
    // let tmp = i2c.reg_read_byte(gyro::I2C_ADDRESS, gyro::REG_USER_CTRL);
    // i2c.reg_write_byte(gyro::I2C_ADDRESS, gyro::REG_USER_CTRL, tmp & (!gyro::BIT_AUX_IF_EN));
    // delay_ms(3);
    let tmp = gyro::BIT_BYPASS_EN | gyro::BIT_ACTL;
    i2c.reg_write_byte(gyro::I2C_ADDRESS, gyro::REG_INT_PIN_CFG, tmp);
}


pub fn check_mpu(i2c: &mut I2c, whoami: &mut u8) -> bool
{
    *whoami = i2c.reg_read_byte(gyro::I2C_ADDRESS, gyro::REG_WHO_AM_I);
    return gyro::VAL_WHO_AM_I == *whoami;
}

pub fn check_mag(i2c: &mut I2c, whoami: &mut u8) -> bool
{
    *whoami = i2c.reg_read_byte(mag::I2C_ADDRESS, mag::REG_WHO_AM_I);
    return mag::VAL_WHO_AM_I == *whoami;
}

#[inline]
fn i16_vec_from_bytes(bytes: &[u8;6], vec: &mut [i16;3]) {
    for i in 0..3 {
        let bi = i << 1;
        vec[i] = ((bytes[bi+1] as i16) << 8) | (bytes[bi] as i16);
    }
}

/**
 * @brief Get accelerometer datas
 * @param  None
 * @retval None
 */
pub fn accel_read(i2c: &mut I2c, vec: &mut [i16;3])
{
    let mut buf = [0u8;6];
    i2c.reg_read_array(gyro::I2C_ADDRESS, gyro::REG_RAW_ACCEL, &mut buf);
    i16_vec_from_bytes(&buf, vec);
}
/**
 * @brief Get gyroscopes datas
 * @param  None
 * @retval None
 */
pub fn gyro_read(i2c: &mut I2c, vec: &mut [i16;3])
{
    let mut buf = [0u8;6];
    i2c.reg_read_array(gyro::I2C_ADDRESS, gyro::REG_RAW_GYRO, &mut buf);
    i16_vec_from_bytes(&buf, vec);
}
/**
 * @brief Get compass datas
 * @param  None
 * @retval None
 */
pub fn mag_read(i2c: &mut I2c, vec: &mut [i16;3])
{
    i2c.reg_write_byte(mag::I2C_ADDRESS, mag::REG_CNTL, mag::AKM_SINGLE_MEASUREMENT);
    let mut buf = [0u8;8];

    i2c.reg_read_array(mag::I2C_ADDRESS, mag::REG_AKM_ST1, &mut buf);
    // TODO: check readiness
    let mut data = [0u8;6];
    data.copy_from_slice(&buf[1..7]);
    i16_vec_from_bytes(&data, vec);
}

