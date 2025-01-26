#![no_mainj
#![no_std]

use cortex_m_rt::entry;
use cortex_m::asm::delay;
use panic_halt as _;

use stm32g4xx_hal::{
    gpio::{gpiob::{PB3, PB4, PB5}, Alternate, AF5}, pac::fdcan::test, prelude::*, pwr::PwrExt, rcc, spi, stm32, time::RateExtU32
};

use rtt_target::{rprintln, rtt_init_print};
// add dft
pub mod imu;


#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();
    let rcc_cfg = rcc::Config::pll().pll_cfg(
        rcc::PllConfig{
            mux: rcc::PllSrc::HSE(16u32.MHz()), // 16 MHz
            m: rcc::PllMDiv::DIV_2,  // 16 / 2 = 8 MHz
            n: rcc::PllNMul::MUL_24,  // 8 * 24 = 192 MHz
            r: Some(rcc::PllRDiv::DIV_2), // 192 / 2 = 96 MHz -> System clock
            q: Some(rcc::PllQDiv::DIV_4), // 192 / 4 = 48 MHz -> USB clock
            p: None,
        }
    );
    let mut rcc: rcc::Rcc = dp.RCC.freeze(rcc_cfg, pwr);
    imu::imu_test();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    //
    // Setup board LED
    let mut led = gpiob.pb0.into_push_pull_output();

    
    let sclk: PB3<Alternate<AF5>> = gpiob.pb3.into_alternate();
    let miso: PB4<Alternate<AF5>> = gpiob.pb4.into_alternate();
    let mosi: PB5<Alternate<AF5>> = gpiob.pb5.into_alternate();

    let mut spi = dp.SPI1.spi((sclk, miso, mosi), spi::MODE_0, 400.kHz(), &mut rcc);
    let mut cs = gpioa.pa15.into_push_pull_output();
    cs.set_high().unwrap(); // Deselect the device

    // Configure IMU
    // Reset IMU
    rtt_init_print!();

    let mut buffer = [0x75 | 0x80, 0x00];

    rprintln!("Works");
    cs.set_low().unwrap();
    match spi.transfer(&mut buffer) {
        Ok(received_data) => {
            // If successful, print the received data
            rprintln!("Received data: {:?}", received_data);
        }
        Err(e) => {
            // If an error occurred, print the error
            rprintln!("SPI transfer error: {:?}", e);
        }
    }
    cs.set_high().unwrap();
    //let who_am_i = read_register(WHO_AM_I);
    //rprintln!("WHO_AM_I: {:#X}", who_am_i);
    //if who_am_i != 0x70 {
    //rprintln!("MPU6500 not detected! Check connections.");
    //return;

    loop {
        led.set_high().unwrap();
        delay(30_000_000);
        led.set_low().unwrap();
        delay(10_000_000);
        rprintln!("HELLO!");
        // Read sensor data
       // let mut tx_buffer = [0u8; 15];
       // tx_buffer[0] = 0x3B | 0x80 ; // start register adress with MSB 1

       // cs.set_low().unwrap();
       // spi.transfer(&mut tx_buffer).unwrap();
       // cs.set_high().unwrap();

       // let accel_x = (((tx_buffer[1] as i16) << 8) | tx_buffer[2] as i16) as f32 / 16384.0;
       // let accel_y = (((tx_buffer[3] as i16) << 8) | tx_buffer[4] as i16) as f32 / 16384.0;
       // let accel_z = (((tx_buffer[5] as i16) << 8) | tx_buffer[6] as i16) as f32 / 16384.0;
        //let temp = (((tx_buffer[7] as i16) << 8) | tx_buffer[8] as i16 ) as f32 / 340.0 + 36.53;
        //let gyro_x = (((tx_buffer[9] as i16) << 8) | tx_buffer[10] as i16) as f32 / 131.0;
        //let gyro_y = (((tx_buffer[11] as i16) << 8) | tx_buffer[12] as i16) as f32 / 131.0;
        //let gyro_z = (((tx_buffer[13] as i16) << 8) | tx_buffer[14] as i16) as f32 / 131.0;

       // rprintln!("Accel X: {}", accel_x);
       // rprintln!("Accel Y: {}", accel_y);
       // rprintln!("Accel Z: {}", accel_z);

   }
}
