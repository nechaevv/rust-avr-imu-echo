// BMP280 Barometric sensor

/*
 *  BMP280 I2c address
 */
use arduino_hal::I2c;
use libm;
use crate::imu::i2c::I2cRegExt;

pub const BMP280_AD0_LOW: u8 = 0x76;
//address pin low (GND)
pub const BMP280_AD0_HIGH: u8 = 0x77;
//address pin high (VCC)
pub const BMP280_ADDR: u8 = BMP280_AD0_HIGH;
// default I2C address

pub const BMP280_CHIPID_VAL: u8 = 0x58;
/*
 *  BMP280 register address
 */
pub const BMP280_REGISTER_DIG_T1: u8 = 0x88;
pub const BMP280_REGISTER_DIG_T2: u8 = 0x8A;
pub const BMP280_REGISTER_DIG_T3: u8 = 0x8C;

pub const BMP280_REGISTER_DIG_P1: u8 = 0x8E;
pub const BMP280_REGISTER_DIG_P2: u8 = 0x90;
pub const BMP280_REGISTER_DIG_P3: u8 = 0x92;
pub const BMP280_REGISTER_DIG_P4: u8 = 0x94;
pub const BMP280_REGISTER_DIG_P5: u8 = 0x96;
pub const BMP280_REGISTER_DIG_P6: u8 = 0x98;
pub const BMP280_REGISTER_DIG_P7: u8 = 0x9A;
pub const BMP280_REGISTER_DIG_P8: u8 = 0x9C;
pub const BMP280_REGISTER_DIG_P9: u8 = 0x9E;

pub const BMP280_REGISTER_CHIPID: u8 = 0xD0;
pub const BMP280_REGISTER_VERSION: u8 = 0xD1;
pub const BMP280_REGISTER_SOFTRESET: u8 = 0xE0;
pub const BMP280_REGISTER_STATUS: u8 = 0xF3;
pub const BMP280_REGISTER_CONTROL: u8 = 0xF4;
pub const BMP280_REGISTER_CONFIG: u8 = 0xF5;

pub const BMP280_TEMP_XLSB_REG: u8 = 0xFC;    /*Temperature XLSB Register */
pub const BMP280_TEMP_LSB_REG: u8 = 0xFB;    /*Temperature LSB Register  */
pub const BMP280_TEMP_MSB_REG: u8 = 0xFA;    /*Temperature MSB Register  */
pub const BMP280_PRESS_XLSB_REG: u8 = 0xF9;    /*Pressure XLSB  Register   */
pub const BMP280_PRESS_LSB_REG: u8 = 0xF8;    /*Pressure LSB Register     */
pub const BMP280_PRESS_MSB_REG: u8 = 0xF7;    /*Pressure MSB Register     */

/*calibration parameters */
pub const BMP280_DIG_T1_LSB_REG: u8 = 0x88;
pub const BMP280_DIG_T1_MSB_REG: u8 = 0x89;
pub const BMP280_DIG_T2_LSB_REG: u8 = 0x8A;
pub const BMP280_DIG_T2_MSB_REG: u8 = 0x8B;
pub const BMP280_DIG_T3_LSB_REG: u8 = 0x8C;
pub const BMP280_DIG_T3_MSB_REG: u8 = 0x8D;
pub const BMP280_DIG_P1_LSB_REG: u8 = 0x8E;
pub const BMP280_DIG_P1_MSB_REG: u8 = 0x8F;
pub const BMP280_DIG_P2_LSB_REG: u8 = 0x90;
pub const BMP280_DIG_P2_MSB_REG: u8 = 0x91;
pub const BMP280_DIG_P3_LSB_REG: u8 = 0x92;
pub const BMP280_DIG_P3_MSB_REG: u8 = 0x93;
pub const BMP280_DIG_P4_LSB_REG: u8 = 0x94;
pub const BMP280_DIG_P4_MSB_REG: u8 = 0x95;
pub const BMP280_DIG_P5_LSB_REG: u8 = 0x96;
pub const BMP280_DIG_P5_MSB_REG: u8 = 0x97;
pub const BMP280_DIG_P6_LSB_REG: u8 = 0x98;
pub const BMP280_DIG_P6_MSB_REG: u8 = 0x99;
pub const BMP280_DIG_P7_LSB_REG: u8 = 0x9A;
pub const BMP280_DIG_P7_MSB_REG: u8 = 0x9B;
pub const BMP280_DIG_P8_LSB_REG: u8 = 0x9C;
pub const BMP280_DIG_P8_MSB_REG: u8 = 0x9D;
pub const BMP280_DIG_P9_LSB_REG: u8 = 0x9E;
pub const BMP280_DIG_P9_MSB_REG: u8 = 0x9F;

pub struct CalibrationData {
    t1: u16,    /*<calibration t1 data*/
    t2: i16,    /*<calibration T2 data*/
    t3: i16,    /*<calibration T3 data*/
    p1: u16,    /*<calibration P1 data*/
    p2: i16,    /*<calibration P2 data*/
    p3: i16,    /*<calibration P3 data*/
    p4: i16,    /*<calibration P4 data*/
    p5: i16,    /*<calibration P5 data*/
    p6: i16,    /*<calibration P6 data*/
    p7: i16,    /*<calibration P7 data*/
    p8: i16,    /*<calibration P8 data*/
    p9: i16,    /*<calibration P9 data*/
}

pub fn new_calibration_data() -> CalibrationData {
    CalibrationData {
        t1: 0, t2: 0, t3: 0, p1:0, p2: 0, p3: 0, p4: 0, p5:0, p6:0, p7:0, p8:0, p9:0 
    }
}

pub const MSLP: i32 = 101325;

fn read_calibration(i2c: &mut I2c, dig: &mut CalibrationData)
{
    let mut lsb: u8;
    let mut msb: u8;

    /* read the temperature calibration parameters */
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_T1_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_T1_MSB_REG);
    dig.t1 = (msb as u16) << 8 | (lsb as u16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_T2_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_T2_MSB_REG);
    dig.t2 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_T3_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_T3_MSB_REG);
    dig.t3 = (msb as i16) << 8 | (lsb as i16);

    /* read the pressure calibration parameters */
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P1_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P1_MSB_REG);
    dig.p1 = (msb as u16) << 8 | (lsb as u16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P2_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P2_MSB_REG);
    dig.p2 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P3_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P3_MSB_REG);
    dig.p3 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P4_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P4_MSB_REG);
    dig.p4 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P5_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P5_MSB_REG);
    dig.p5 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P6_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P6_MSB_REG);
    dig.p6 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P7_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P7_MSB_REG);
    dig.p7 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P8_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P8_MSB_REG);
    dig.p8 = (msb as i16) << 8 | (lsb as i16);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P9_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_DIG_P9_MSB_REG);
    dig.p9 = (msb as i16) << 8 | (lsb as i16);
}

pub fn init(i2c: &mut I2c, calibration_data: &mut CalibrationData)
{
    i2c.reg_write_byte(BMP280_ADDR, BMP280_REGISTER_CONTROL, 0xFF);
    i2c.reg_write_byte(BMP280_ADDR, BMP280_REGISTER_CONFIG, 0x14);
    read_calibration(i2c, calibration_data);
}

pub fn check(i2c: &mut I2c, whoami: &mut u8) -> bool
{
    *whoami = i2c.reg_read_byte(BMP280_ADDR, BMP280_REGISTER_CHIPID);
    return *whoami == BMP280_CHIPID_VAL;
}

fn compensate_temperature(dig: &CalibrationData, adc_t: i32, t_fine: &mut i32) -> i32
{
    let var1  = (((adc_t>>3) - (dig.t1 as i32) <<1) *(dig.t2 as i32)) >> 11;
    let var2  = (((((adc_t>>4) - (dig.t1 as i32)) *(((adc_t as i32)>>4) -
        (dig.t1 as i32))) >> 12) * (dig.t3 as i32)) >> 14;

    *t_fine = var1 + var2;

    return (*t_fine * 5 + 128) >> 8;
}

fn compensate_pressure(dig: &CalibrationData, adc_p: i32, t_fine: i32) -> u32
{
    let (mut var1, mut var2, mut pressure): (i64,i64,i64);

    var1 = (t_fine as i64) - 128000;
    var2 = var1 * var1 * (dig.p6 as i64);
    var2 = var2 + ((var1*(dig.p5 as i64))<<17);
    var2 = var2 + ((dig.p4 as i64)<<35);
    var1 = ((var1 * var1 * (dig.p3 as i64))>>8) + ((var1 * (dig.p2 as i64))<<12);
    var1 = (((1 as i64)<<47)+var1)*(dig.p1 as i64)>>33;

    if var1 == 0 {
        return 0; // avoid exception caused by division by zero
    }

    pressure = 1048576 - (adc_p as i64);
    pressure = (((pressure<<31) - var2)*3125) / var1;
    var1 = ((dig.p9 as i64) * (pressure>>13) * (pressure>>13)) >> 25;
    var2 = ((dig.p8 as i64) * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + ((dig.p7 as i64)<<4);
    return pressure as u32;

// var1 = (((int64_t)t_fine)>>1) - (int64_t)64000;
// var2 = (((var1>>2) * (var1>>2)) >> 11 ) *((int64_t)dig_P6);
// var2 = var2 + ((var1 *((int64_t)dig_P5))<<1);
// var2 = (var2>>2) + (((int64_t)dig_P4)<<16);
// var1 = (((dig_P3 * (((var1>>2) * (var1>>2))>>13))>>3) + ((((int64_t)dig_P2) * var1)>>1))>>18;
// var1 = ((((32768+var1))*((int64_t)dig_P1))>>15);
// if(var1 ==0)
// {
// return 0;
// }
// pressure = (1048576.0 - adc_P) - (var2>>12)*3125;
// if(pressure < 0x80000000)
// {
// pressure = (pressure<<1)/((uint64_t)var1);
// }
// else
// {
// pressure = (pressure/(uint64_t)var1)*2;
// }
// var1 = (((int64_t)dig_P9) *((int64_t)(((pressure>>3)*(pressure>>3))>>13)))>>12;
// var2 = (((int64_t)(pressure>>2))*((int64_t)dig_P8))>>13;
// pressure = (uint64_t)((int64_t)pressure) +((var1 + var2 + dig_P7)>>4);
// return (float)pressure;

}

fn t_and_p_get(i2c: &mut I2c, calibration_data: &CalibrationData, temperature: &mut i32, pressure: &mut u32)
{
    let (mut lsb, mut msb, mut xlsb): (u8,u8,u8);
    let (mut adc_p, mut adc_t): (i32, i32);
    let mut t_fine: i32 = 0;

    xlsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_TEMP_XLSB_REG);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_TEMP_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_TEMP_MSB_REG);
    //adc_t = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    adc_t = msb as i32;
    adc_t <<= 8;
    adc_t |= lsb as i32;
    adc_t <<= 8;
    adc_t |= xlsb as i32;
    adc_t >>= 4;
    //adc_t = 415148;
    *temperature = compensate_temperature(calibration_data, adc_t, &mut t_fine);

    xlsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_PRESS_XLSB_REG);
    lsb = i2c.reg_read_byte(BMP280_ADDR, BMP280_PRESS_LSB_REG);
    msb = i2c.reg_read_byte(BMP280_ADDR, BMP280_PRESS_MSB_REG);
    //adc_p = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    adc_p = msb as i32;
    adc_p <<= 8;
    adc_p |= lsb as i32;
    adc_p <<= 8;
    adc_p |= xlsb as i32;
    adc_p >>= 4;
    //adc_p = 51988;
    *pressure = compensate_pressure(calibration_data, adc_p, t_fine);
}
#[inline]
fn calculate_absolute_altitude(pressure: u32) -> i32
{
    return 4433000 * (1.0 - libm::powf((pressure as f32) / (MSLP as f32), 0.1903)) as i32;
}

pub fn press_sensor_data_get(i2c: &mut I2c, calibration_data: &CalibrationData, temperature: &mut i32, pressure: &mut u32, altitude: &mut i32)
{
    t_and_p_get(i2c, calibration_data, temperature, pressure);
    *altitude = calculate_absolute_altitude(*pressure);
}
