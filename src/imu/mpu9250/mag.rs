pub const I2C_ADDRESS   :u8 = 0x0C;

pub const REG_WHO_AM_I      :u8 = 0x00;
//pub const REG_RAW_COMPASS   :u8 = 0x49;
pub const REG_CNTL          :u8 = 0x0A;
pub const REG_AKM_ST1       :u8 = 0x02;
// pub const s0_addr        = 0x25,
// pub const s0_reg         = 0x26,
// pub const s0_ctrl        = 0x27,
// pub const s1_addr        = 0x28,
// pub const s1_reg         = 0x29,
// pub const s1_ctrl        = 0x2A,
// pub const s4_ctrl        = 0x34,
// pub const s0_do          = 0x63,
// pub const s1_do          = 0x64,
// pub const i2c_delay_ctrl = 0x67

pub const VAL_WHO_AM_I  :u8 = 0x48; // AK8963 - 0x48

const SUPPORTS_AK89XX_HIGH_SENS: u8 = 0x10;
//#define AK89xx_FSR                  (4915)

//pub const AKM_POWER_DOWN        : u8 = 0x00 | SUPPORTS_AK89XX_HIGH_SENS;
pub const AKM_SINGLE_MEASUREMENT: u8 = 0x01 | SUPPORTS_AK89XX_HIGH_SENS;
