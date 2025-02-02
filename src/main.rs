#![no_std]
#![no_main]

use cortex_m_rt::entry;
use filter::Filter;
use imu::IMUConfig;
use panic_halt as _;

use stm32g4xx_hal::{
    i2c::Config, pac::TIM2, prelude::*, pwr::PwrExt, rcc, stm32, time::ExtU32, time::RateExtU32,
    timer::CountDownTimer, timer::Timer,
};

use cortex_m as _;
use defmt::info;
use defmt_rtt as _;



mod imu;
mod filter;


struct State {
    pitch: f32,
    roll: f32,
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    //let rcc_cfg = rcc::Config::pll().pll_cfg(rcc::PllConfig {
    //    mux: rcc::PllSrc::HSE(16u32.MHz()), // 16 MHz
    //    m: rcc::PllMDiv::DIV_2,             // 16 / 2 = 8 MHz
    //    n: rcc::PllNMul::MUL_24,            // 8 * 24 = 192 MHz
    //    r: Some(rcc::PllRDiv::DIV_2),       // 192 / 2 = 96 MHz -> System clock
    //    q: Some(rcc::PllQDiv::DIV_4),       // 192 / 4 = 48 MHz -> USB clock
    //    p: None,
    //});
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let sda = gpioa.pa8.into_alternate_open_drain();
    let scl = gpioa.pa9.into_alternate_open_drain();
    let mut i2c = dp.I2C2.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

    let mut imu_ = imu::IMU::new(i2c);
    imu_.reset();

    let config: IMUConfig = IMUConfig::new()
        .with_accel_odr(imu::AccelODR::Hz_416)
        .with_accel_range(imu::AccelRange::Range4G)
        .with_gyro_odr(imu::GyroODR::Hz_416)
        .with_gyro_range(imu::GyroRange::Range1000DPS);

    imu_.configure(config);
    imu_.calibrate();

    let mut timer: CountDownTimer<TIM2> =
        Timer::new(dp.TIM2, &mut rcc.clocks).start_count_down(2.millis());

    let mut compfilter = filter::ComplementaryFilter::new(0.2);

    loop {
        if let (Ok((ax, ay, az)), Ok((gx, gy, gz))) = (imu_.read_accel(), imu_.read_gyro()) {
            let (roll, pitch) = compfilter.update(ax, ay, az, gx, gy, gz, 0.02);

            info!(
                "Pitch: {}, Roll: {}",
                pitch, roll
            );
        }
        while timer.wait().is_err() {}
    }
}
