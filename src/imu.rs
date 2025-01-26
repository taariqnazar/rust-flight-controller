use stm32g4xx_hal::hal::blocking::i2c::{Write, WriteRead};

pub const DEVICE_ADDR: u8 = 0x6B;
pub const CTRL1_XL: u8 = 0x10;
pub const CTRL2_G: u8 = 0x11;
pub const CTRL3_C: u8 = 0x12;
pub const CTRL4_C: u8 = 0x13;
pub const CTRL5_C: u8 = 0x14;
pub const CTRL6_C: u8 = 0x15;
pub const CTRL7_C: u8 = 0x16;
pub const CTRL8_XL: u8 = 0x17;
pub const CTRL9_XL: u8 = 0x18;
pub const CTRL10_C: u8 = 0x19;
pub const MASTER_CONFIG: u8 = 0x1A;

pub const OUT_TEMP_L: u8 = 0x20;
pub const OUT_TEMP_H: u8 = 0x21;
pub const OUTX_L_G: u8 = 0x22;
pub const OUTX_H_G: u8 = 0x23;
pub const OUTY_L_G: u8 = 0x24;
pub const OUTY_H_G: u8 = 0x25;
pub const OUTZ_L_G: u8 = 0x26;
pub const OUTZ_H_G: u8 = 0x27;
pub const OUTX_L_XL: u8 = 0x28;
pub const OUTX_H_XL: u8 = 0x29;
pub const OUTY_L_XL: u8 = 0x2A;
pub const OUTY_H_XL: u8 = 0x2B;
pub const OUTZ_L_XL: u8 = 0x2C;
pub const OUTZ_H_XL: u8 = 0x2D;

pub enum AccelODR {
    PowerDown = 0b0000,
    Hz_1_6 = 0b1011,
    Hz_12_5 = 0b0001,
    Hz_26 = 0b0010,
    Hz_52 = 0b0011,
    Hz_104 = 0b0100,
    Hz_208 = 0b0101,
    Hz_416 = 0b0110,
    Hz_833 = 0b0111,
    Hz_1660 = 0b1000,
    Hz_3330 = 0b1001,
    Hz_6660 = 0b1010,
}

pub enum AccelRange {
    Range2G = 0b00,
    Range4G = 0b10,
    Range8G = 0b11,
    Range16G = 0b01,
}

impl AccelRange {
    pub fn scale(&self) -> f32 {
        match self {
            AccelRange::Range2G => 0.061 / 1000.0,
            AccelRange::Range4G => 0.122 / 1000.0,
            AccelRange::Range8G => 0.244 / 1000.0,
            AccelRange::Range16G => 0.488 / 1000.0,
        }
    }
}

pub enum GyroODR {
    PowerDown = 0b0000,
    Hz_12_5 = 0b0001,
    Hz_26 = 0b0010,
    Hz_52 = 0b0011,
    Hz_104 = 0b0100,
    Hz_208 = 0b0101,
    Hz_416 = 0b0110,
    Hz_833 = 0b0111,
    Hz_1660 = 0b1000,
    Hz_3330 = 0b1001,
    Hz_6660 = 0b1010,
}

// TODO: Add FS_125
pub enum GyroRange {
    Range245DPS = 0b00,
    Range500DPS = 0b01,
    Range1000DPS = 0b10,
    Range2000DPS = 0b11,
}

impl GyroRange {
    pub fn scale(&self) -> f32 {
        match self {
            GyroRange::Range245DPS => 8.75 / 1000.0,
            GyroRange::Range500DPS => 17.50 / 1000.0,
            GyroRange::Range1000DPS => 3.50 / 1000.0,
            GyroRange::Range2000DPS => 70.0 / 1000.0,
        }
    }
}

#[derive(Debug)]
pub enum IMUError<E> {
    I2CError(E),
    InvalidConfig(E),
}

pub struct IMUConfig {
    pub accel_odr: AccelODR,
    pub accel_range: AccelRange,
    pub gyro_odr: GyroODR,
    pub gyro_range: GyroRange,
}

impl Default for IMUConfig {
    fn default() -> Self {
        Self {
            accel_odr: AccelODR::Hz_52,
            accel_range: AccelRange::Range2G,
            gyro_odr: GyroODR::Hz_52,
            gyro_range: GyroRange::Range245DPS,
        }
    }
}

impl IMUConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_accel_odr(mut self, accel_odr: AccelODR) -> Self {
        self.accel_odr = accel_odr;
        self
    }
    pub fn with_accel_range(mut self, accel_range: AccelRange) -> Self {
        self.accel_range = accel_range;
        self
    }
    pub fn with_gyro_odr(mut self, gyro_odr: GyroODR) -> Self {
        self.gyro_odr = gyro_odr;
        self
    }
    pub fn with_gyro_range(mut self, gyro_range: GyroRange) -> Self {
        self.gyro_range = gyro_range;
        self
    }
}

pub struct IMU<I2C> {
    i2c: I2C,
    accel_scale: f32,
    gyro_scale: f32,
}

impl<I2C, E> IMU<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        IMU {
            i2c,
            accel_scale: 1.0,
            gyro_scale: 1.0,
        }
    }

    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), IMUError<E>> {
        let mut data = [0u8; 6];
        self.i2c
            .write_read(DEVICE_ADDR, &[OUTX_L_XL], &mut data)
            .map_err(IMUError::I2CError);

        let accel_x = (data[0] as i16 | (data[1] as i16) << 8) as f32 * self.accel_scale;
        let accel_y = (data[2] as i16 | (data[3] as i16) << 8) as f32 * self.accel_scale;
        let accel_z = (data[4] as i16 | (data[5] as i16) << 8) as f32 * self.accel_scale;

        Ok((accel_x, accel_y, accel_z))
    }

    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), IMUError<E>> {
        let mut data = [0u8; 6];
        self.i2c
            .write_read(DEVICE_ADDR, &[OUTX_L_G], &mut data)
            .map_err(IMUError::I2CError);

        let gyro_x = (data[0] as i16 | (data[1] as i16) << 8) as f32 * self.gyro_scale;
        let gyro_y = (data[2] as i16 | (data[3] as i16) << 8) as f32 * self.gyro_scale;
        let gyro_z = (data[4] as i16 | (data[5] as i16) << 8) as f32 * self.gyro_scale;

        Ok((gyro_x, gyro_y, gyro_z))
    }

    pub fn read_temp(&mut self) -> Result<f32, IMUError<E>> {
        let mut data = [0u8; 2];
        self.i2c
            .write_read(DEVICE_ADDR, &[OUT_TEMP_L], &mut data)
            .map_err(IMUError::I2CError);

        let temp = (data[0] as i16 | (data[1] as i16) << 8) as f32 / 256.0 + 25.0;

        Ok(temp)
    }
    pub fn calibrate() {}
}

impl<I2C, E> IMU<I2C>
where
    I2C: Write<Error = E>,
{
    pub fn configure(&mut self, config: IMUConfig) -> Result<(), IMUError<E>> {
        self.accel_scale = config.accel_range.scale();
        self.gyro_scale = config.gyro_range.scale();

        // Configure accelerometer
        self.i2c
            .write(
                DEVICE_ADDR,
                &[
                    CTRL1_XL,
                    (config.accel_odr as u8) << 4 | (config.accel_range as u8) << 2,
                ],
            )
            .map_err(IMUError::InvalidConfig);

        // Configure gyroscope
        self.i2c
            .write(
                DEVICE_ADDR,
                &[
                    CTRL2_G,
                    (config.gyro_odr as u8) << 4 | (config.gyro_range as u8) << 2,
                ],
            )
            .map_err(IMUError::InvalidConfig);

        Ok(())
    }

    pub fn reset(&mut self) {
        // Reset device
        match self.i2c.write(DEVICE_ADDR, &[CTRL3_C, 0b00000101])
            {
            Ok(_) => {}
            Err(_) => {}
        };
    }
}
