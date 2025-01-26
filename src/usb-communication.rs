#![no_main]
#![no_std]

use cortex_m_rt::entry;
use cortex_m::asm::delay;
use panic_halt as _;

use stm32g4xx_hal as hal;
use hal::{
    prelude::*,
    stm32, 
    rcc,
    pwr::PwrExt,
    time::RateExtU32,
};

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
    let mut rcc = dp.RCC.freeze(rcc_cfg, pwr);

    let gpioa = dp.GPIOA.split(&mut rcc);
    
    // Setup USB debugging
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
        .manufacturer("STM32")
        .product("Drone-Slam ESC")
        .serial_number("TESTING123")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    //let gpiob = dp.GPIOB.split(&mut rcc);
    //let mut led = gpiob.pb7.into_push_pull_output();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }
                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
        //led.set_high().unwrap();
        //delay(10_000_000);

        //led.set_low().unwrap();
        //delay(10_000_000);
    }
}
