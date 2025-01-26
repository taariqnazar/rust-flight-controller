#![no_main]
#![no_std]

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use imu::IMUConfig;
use panic_halt as _;

use stm32g4xx_hal::{i2c::Config, prelude::*, pwr::PwrExt, rcc, stm32, time::RateExtU32};

use rtt_target::{rprint, rprintln, rtt_init_print};

mod imu;

const OUT_TEMP_L: u8 = 0x20; // Lower byte of temperature data
const OUT_TEMP_H: u8 = 0x21; // Higher byte of temperature data

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();
    let rcc_cfg = rcc::Config::pll().pll_cfg(rcc::PllConfig {
        mux: rcc::PllSrc::HSE(16u32.MHz()), // 16 MHz
        m: rcc::PllMDiv::DIV_2,             // 16 / 2 = 8 MHz
        n: rcc::PllNMul::MUL_24,            // 8 * 24 = 192 MHz
        r: Some(rcc::PllRDiv::DIV_2),       // 192 / 2 = 96 MHz -> System clock
        q: Some(rcc::PllQDiv::DIV_4),       // 192 / 4 = 48 MHz -> USB clock
        p: None,
    });
    //let mut rcc: rcc::Rcc = dp.RCC.freeze(rcc_cfg, pwr);
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Setup board LED
    let mut led1 = gpiob.pb0.into_push_pull_output();
    let mut led2 = gpioa.pa7.into_push_pull_output();
    let mut led3 = gpioa.pa2.into_push_pull_output();

    let sda = gpioa.pa8.into_alternate_open_drain();
    let scl = gpioa.pa9.into_alternate_open_drain();
    let mut i2c = dp.I2C2.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

    rtt_init_print!();

    let mut imu_ = imu::IMU::new(i2c);
    imu_.reset();

    let config: IMUConfig = IMUConfig::new()
        .with_accel_odr(imu::AccelODR::Hz_416)
        .with_accel_range(imu::AccelRange::Range4G)
        .with_gyro_odr(imu::GyroODR::Hz_416)
        .with_gyro_range(imu::GyroRange::Range1000DPS);

    match imu_.configure(config) {
        Ok(_) => rprintln!("IMU configured"),
        Err(e) => rprintln!("Error configuring IMU: {:?}", e),
    }

    imu_.calibrate(); 

    loop {
        match imu_.read_accel() {
            Ok((accel_x, accel_y, accel_z)) => {
                rprintln!("Accel: X: {:.2} Y: {:.2} Z: {:.2}", accel_x, accel_y, accel_z);
            }
            Err(e) => {
                rprintln!("Error reading accel: {:?}", e);
            }
        }
 
        delay(10_000_000);
    }
}
