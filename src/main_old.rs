#![no_main]
#![no_std]

use cortex_m_rt::entry;
use cortex_m::asm::delay;
use panic_halt as _;

use hal::{
    prelude::*,
    stm32, 
    rcc,
//    spi,
    pwr::PwrExt,
    time::RateExtU32,
//   gpio::{Alternate, AF5},
//   gpio::gpiob::{PB3, PB4, PB5},
};
use stm32g4xx_hal as hal;

mod usb;
use usb::{Peripheral, UsbBus};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

//TODO: 
//[x]Print to computer using usb for debugging 
//[]Read data from mpu6500

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();
    // Configure the clock to run @96 MHz and USB to run at 48 MHz
    //let rcc_cfg = rcc::Config::pll().pll_cfg(
    //    rcc::PllConfig{
    //        mux: rcc::PllSrc::HSE(16u32.MHz()), // 16 MHz
    //        m: rcc::PllMDiv::DIV_2,  // 16 / 2 = 8 MHz
    //        n: rcc::PllNMul::MUL_24,  // 8 * 24 = 192 MHz
    //        r: Some(rcc::PllRDiv::DIV_2), // 192 / 2 = 96 MHz -> System clock
    //        q: Some(rcc::PllQDiv::DIV_4), // 192 / 4 = 48 MHz -> USB clock
    //        p: None,
    //    }
    //);
    let rcc_cfg = rcc::Config::hsi();
    let mut rcc = dp.RCC.freeze(rcc_cfg, pwr);
    rcc.enable_hsi48();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Setup board LED
    let mut led = gpiob.pb7.into_push_pull_output();

    // Setup USB
    let usb_dm = gpioa.pa11.into_alternate();
    let usb_dp = gpioa.pa12.into_alternate();

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    // Setup SPI 
    //let sclk: PB3<Alternate<AF5>> = gpiob.pb3.into_alternate();
    //let miso: PB4<Alternate<AF5>> = gpiob.pb4.into_alternate();
    //let mosi: PB5<Alternate<AF5>> = gpiob.pb5.into_alternate();

    //let mut spi = dp.SPI1.spi((sclk, miso, mosi), spi::MODE_0, 400.kHz(), &mut rcc);
    //let mut cs = gpioa.pa15.into_push_pull_output();
    //cs.set_high().unwrap(); // Deselect the device

    //// Configure IMU
    //// Reset IMU
    //let mut tx_buffer = [0x6B & 0x7F, 0x80];
    //cs.set_low().unwrap();
    //spi.transfer(&mut tx_buffer).unwrap();
    //cs.set_high().unwrap();
    // Inititialize settings 

    loop {
        led.set_high().unwrap();
        delay(10_000_000);
        led.set_low().unwrap();
        delay(10_000_000);
        // Read sensor data
        //let mut tx_buffer = [0u8; 15];
        //tx_buffer[0] = 0x3B | 0x80 ; // start register adress with MSB 1

        //cs.set_low().unwrap();
        //spi.transfer(&mut tx_buffer).unwrap();
        //cs.set_high().unwrap();

        //let accel_x = (((tx_buffer[1] as i16) << 8) | tx_buffer[2] as i16) as f32 / 16384.0;
        ////let accel_y = (((tx_buffer[3] as i16) << 8) | tx_buffer[4] as i16) as f32 / 16384.0;
        ////let accel_z = (((tx_buffer[5] as i16) << 8) | tx_buffer[6] as i16) as f32 / 16384.0;
        ////let temp = (((tx_buffer[7] as i16) << 8) | tx_buffer[8] as i16 ) as f32 / 340.0 + 36.53;
        ////let gyro_x = (((tx_buffer[9] as i16) << 8) | tx_buffer[10] as i16) as f32 / 131.0;
        ////let gyro_y = (((tx_buffer[11] as i16) << 8) | tx_buffer[12] as i16) as f32 / 131.0;
        ////let gyro_z = (((tx_buffer[13] as i16) << 8) | tx_buffer[14] as i16) as f32 / 131.0;

        //let mut buf = [0u8; 128]; // Create a buffer
        //let mut writer = BufferWriter::new(&mut buf); // Wrap it in the BufferWriter

        //let _len = core::fmt::write(
        //    &mut writer,
        //    format_args!(
        //       // "Accel: X={:.2} Y={:.2} Z={:.2}, Temp={:.2}°C, Gyro: X={:.2} Y={:.2} Z={:.2}\n",
        //       // accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z
        //        
        //        "Accel: X={:.2}",
        //        accel_x
        //    ),
        //).unwrap();
        
        // Send over USB
        if usb_dev.poll(&mut [&mut serial]) {
            serial.write(b"Hello\n").ok();
        }
    }
}

//struct BufferWriter<'a> {
//    buf: &'a mut [u8],
//    pos: usize,
//}
//
//impl<'a> BufferWriter<'a> {
//    fn new(buf: &'a mut [u8]) -> Self {
//        Self { buf, pos: 0 }
//    }
//}
//
//impl<'a> core::fmt::Write for BufferWriter<'a> {
//    fn write_str(&mut self, s: &str) -> core::fmt::Result {
//        // Check if there's enough space in the buffer
//        let bytes = s.as_bytes();
//        if self.pos + bytes.len() > self.buf.len() {
//            return Err(core::fmt::Error);
//        }
//
//        // Write the string into the buffer
//        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
//        self.pos += bytes.len();
//        Ok(())
//    }
//}

