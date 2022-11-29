pub const I2C_ADDRESS   : u8 = 0x68;

pub const REG_WHO_AM_I      :u8 = 0x75;
pub const REG_USER_CTRL     :u8 = 0x6A;
pub const REG_RAW_GYRO      :u8 = 0x43;
pub const REG_RAW_ACCEL     :u8 = 0x3B;
pub const REG_PWR_MGMT_1    :u8 = 0x6B;
pub const REG_INT_PIN_CFG   :u8 = 0x37;

pub const VAL_WHO_AM_I  :u8 = 0x71; // MPU9250 - 0x71 (MPU9255 - 0x73)

pub const BIT_RESET     :u8 = 0x80;
pub const BIT_AUX_IF_EN :u8 = 0x20;
pub const BIT_BYPASS_EN :u8 = 0x02;
pub const BIT_ACTL      :u8 = 0x80;
