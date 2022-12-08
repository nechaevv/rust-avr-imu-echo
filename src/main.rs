#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![feature(abi_avr_interrupt)]

pub mod imu;
pub mod ring_buffer;

use arduino_hal::delay_ms;
use arduino_hal::spi::{DataOrder, SerialClockRate};
use embedded_hal::spi;
use panic_halt as _;
use crate::imu::comm::Comm;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let spi_settings = arduino_hal::spi::Settings {
        data_order: DataOrder::MostSignificantFirst,
        clock: SerialClockRate::OscfOver4,
        mode: spi::MODE_0,
    };
    let mut cs_imu_pin = pins.d10.into_output();
    let mut cs_mag = pins.d8.into_output();
    let mut cs_alt = pins.d9.into_output();
    cs_imu_pin.set_high();
    cs_mag.set_high();
    cs_alt.set_high();
    let (mut spi, mut cs_imu) = arduino_hal::Spi::with_external_pullup(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12, //.into_pull_up_input(),
        cs_imu_pin,
        spi_settings
    );

    /*
    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100000,
    );
*/

    let mut mag_trim_data = imu::bmm150::empty_trim_data();
    let mut alt_cal_data = imu::bmp388::empty_calibration_data();

    if imu::icm42688::init((&mut spi, &mut cs_imu)).unwrap() {
        ufmt::uwriteln!(&mut serial, "IMU Init completed").unwrap();
    } else {
        ufmt::uwriteln!(&mut serial, "IMU Init failed").unwrap();
    }

    if imu::bmm150::init((&mut spi, &mut cs_mag), &mut mag_trim_data).unwrap() {
        ufmt::uwriteln!(&mut serial, "MAG Init completed").unwrap();
    } else {
        ufmt::uwriteln!(&mut serial, "MAG Init failed").unwrap();
    }

    if imu::bmp388::init((&mut spi, &mut cs_alt), &mut alt_cal_data).unwrap() {
        ufmt::uwriteln!(&mut serial, "ALT Init completed").unwrap();
        //ufmt::uwriteln!(&mut serial, "T1:{} T2:{} T3:{}", alt_cal_data.par_t1, alt_cal_data.par_t2, alt_cal_data.par_t3).unwrap();
    } else {
        ufmt::uwriteln!(&mut serial, "ALT Init failed").unwrap();
    }


    let (mut gyro, mut accel, mut temp, mut timestamp) = ([0i32;3],[0i32;3], 0i16, 0i16);
    let (mut gyro_offset, mut accel_offset) = ([0i32;3],[0i32;3]);
    let mut rotation = [0i32;3];
    //calibrate offsets
    let mut offset_samples = 0u16;
    loop {
        let (mut gyro_sample, mut accel_sample) = ([0i32;3],[0i32;3]);
        if imu::icm42688::get_data((&mut spi, &mut cs_imu), &mut accel_sample, &mut gyro_sample, &mut temp, &mut timestamp).unwrap() > 0 {
            for i in 0..3 {
                accel_offset[i] += accel_sample[i];
                gyro_offset[i] += gyro_sample[i];
            }
            offset_samples += 1;
        }
        if offset_samples > 0xFF { //8 bit
            break;
        }
    }
    for i in 0..3 {
        accel_offset[i] >>= 8;
        gyro_offset[i] >>= 8;
    }
    ufmt::uwriteln!(&mut serial, "Offset calibration done").unwrap();

    let mut counter = 0u8;
    loop {
        let samples = imu::icm42688::get_data((&mut spi, &mut cs_imu), &mut accel, &mut gyro, &mut temp, &mut timestamp).unwrap();
        for i in 0..3 {
            rotation[i] += gyro[i] - gyro_offset[i];
        }
        counter += 1;
        if counter > 200 {
            counter = 0;
            ufmt::uwriteln!(&mut serial, "Accel: {} {} {}", accel[0] - accel_offset[0], accel[1] - accel_offset[1], accel[2] - accel_offset[2]).unwrap();
            ufmt::uwriteln!(&mut serial, "Gyro: {} {} {}", gyro[0]- gyro_offset[0], gyro[1] - gyro_offset[1], gyro[2] - gyro_offset[2]).unwrap();
            ufmt::uwriteln!(&mut serial, "Rotation: {} {} {}",rotation[0] >> 16, rotation[1] >> 16, rotation[2] >> 16).unwrap();
            ufmt::uwriteln!(&mut serial, "Temp: {}", temp).unwrap();
            //ufmt::uwriteln!(&mut serial, "Timestamp: {}", timestamp).unwrap();
            ufmt::uwriteln!(&mut serial, "Samples: {}", samples).unwrap();
            let (mut mag, mut press, mut temp, mut ts) = ([0i16;3],0u32,0i32,0u32);
            if imu::bmm150::get_data((&mut spi, &mut cs_mag), &mut mag, &mag_trim_data).unwrap() {
                ufmt::uwriteln!(&mut serial, "Mag: {} {} {}", mag[0], mag[1], mag[2]).unwrap();
            }
            if imu::bmp388::get_data((&mut spi, &mut cs_alt), &mut press, &mut temp, &mut ts, &alt_cal_data).unwrap() {
                ufmt::uwriteln!(&mut serial, "Baro: {} {} {}", press, temp, ts).unwrap();
            }
        }
    }
}
