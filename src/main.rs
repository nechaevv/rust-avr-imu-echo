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
    let mut cspin = pins.d10.into_output();
    cspin.set_high();
    let (mut spi, mut cs) = arduino_hal::Spi::with_external_pullup(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12, //.into_pull_up_input(),
        cspin,
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

    let init_result = imu::icm42688::init((&mut spi, &mut cs)).unwrap();

    if init_result {
        ufmt::uwriteln!(&mut serial, "Init completed").unwrap();
    } else {
        ufmt::uwriteln!(&mut serial, "Init failed").unwrap();
    }

    let (mut gyro, mut accel, mut temp, mut timestamp) = ([0i32;3],[0i32;3], 0i16, 0i16);
    let (mut gyro_offset, mut accel_offset) = ([0i32;3],[0i32;3]);
    let mut rotation = [0i32;3];
    //calibrate offsets
    let mut offset_samples = 0u16;
    loop {
        let (mut gyro_sample, mut accel_sample) = ([0i32;3],[0i32;3]);
        if imu::icm42688::get_data((&mut spi, &mut cs), &mut accel_sample, &mut gyro_sample, &mut temp, &mut timestamp).unwrap() > 0 {
            for i in 0..3 {
                accel_offset[i] += accel_sample[i];
                gyro_offset[i] += gyro_sample[i];
            }
            offset_samples += 1;
        }
        if offset_samples > 0x7FF {
            break;
        }
    }
    for i in 0..3 {
        accel_offset[i] >>= 11;
        gyro_offset[i] >>= 11;
    }
    ufmt::uwriteln!(&mut serial, "Offset calibration done").unwrap();

    let mut counter = 0u8;
    loop {
        let samples = imu::icm42688::get_data((&mut spi, &mut cs), &mut accel, &mut gyro, &mut temp, &mut timestamp).unwrap();
        for i in 0..3 {
            rotation[i] += gyro[i] - gyro_offset[i];
        }
        // mag_read(&mut i2c, &mut mag);
        // press_sensor_data_get(&mut i2c, &cd, &mut t, &mut p, &mut a);
        //delay_ms(1);
        counter += 1;
        if counter > 100 {
            counter = 0;
            ufmt::uwriteln!(&mut serial, "Accel: {} {} {}", accel[0] - accel_offset[0], accel[1] - accel_offset[1], accel[2] - accel_offset[2]).unwrap();
            ufmt::uwriteln!(&mut serial, "Gyro: {} {} {}", gyro[0]- gyro_offset[0], gyro[1] - gyro_offset[1], gyro[2] - gyro_offset[2]).unwrap();
            ufmt::uwriteln!(&mut serial, "Rotation: {} {} {}",rotation[0] >> 16, rotation[1] >> 16, rotation[2] >> 16).unwrap();
            ufmt::uwriteln!(&mut serial, "Temp: {}", temp).unwrap();
            //ufmt::uwriteln!(&mut serial, "Timestamp: {}", timestamp).unwrap();
            ufmt::uwriteln!(&mut serial, "Samples: {}", samples).unwrap();
//          ufmt::uwriteln!(&mut serial, "Mag: {} {} {}", mag[0], mag[1], mag[2]).unwrap();
//          ufmt::uwriteln!(&mut serial, "Baro: {} {} {}", p, t, a).unwrap();
        }
    }
}
