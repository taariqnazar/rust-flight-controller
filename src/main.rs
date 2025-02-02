#![no_std]
#![no_main]

use cortex_m_rt::entry;
use imu::IMUConfig;
use panic_halt as _;

use stm32g4xx_hal::{
    delay::DelayFromCountDownTimer, i2c::Config, interrupt, prelude::*, pwr::PwrExt, rcc, stm32,
    time::ExtU32, time::RateExtU32, timer::Event::TimeOut, timer::Timer,
};

use cortex_m as _;
use defmt::info;
use defmt_rtt as _;

use libm::{atan2f, sqrtf};

mod imu;

const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

struct State {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

static mut STATE: State = State {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
};

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

    //let gpioa = dp.GPIOA.split(&mut rcc);

    //let sda = gpioa.pa8.into_alternate_open_drain();
    //let scl = gpioa.pa9.into_alternate_open_drain();
    //let mut i2c = dp.I2C2.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

    //let mut imu_ = imu::IMU::new(i2c);
    //imu_.reset();

    //let config: IMUConfig = IMUConfig::new()
    //    .with_accel_odr(imu::AccelODR::Hz_416)
    //    .with_accel_range(imu::AccelRange::Range4G)
    //    .with_gyro_odr(imu::GyroODR::Hz_416)
    //    .with_gyro_range(imu::GyroRange::Range1000DPS);

    //imu_.configure(config);
    //imu_.calibrate();

    info!("This is pre interrupt");

    let mut timer = Timer::new(dp.TIM2, &rcc.clocks).start_count_down(2.millis());
    timer.listen(TimeOut);


    loop {
        cortex_m::asm::wfi();
    }
}
#[interrupt]
fn TIM2() {
    // if let (Ok((ax, ay, az)), Ok((gx, gy, gz))) = (imu_.read_accel(), imu_.read_gyro()) {
    //     let a_pitch = atan2f(ay, az) * RAD_TO_DEG;
    //     let a_roll = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    //     let g_pitch = STATE.pitch + 0.02 * gx;
    //     let g_roll = STATE.roll + 0.02 * gy;

    //     STATE.pitch = 0.98 * g_pitch + 0.02 * a_pitch;
    //     STATE.roll = 0.98 * g_roll + 0.02 * a_roll;
    //     STATE.yaw = STATE.yaw + 0.02 * gz;

    //     info!("Pitch: {}, Roll: {}, Yaw: {}", pitch, roll, yaw);
    // }
    info!("This is the interrupt fn");
}
