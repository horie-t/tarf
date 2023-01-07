#![no_std]
#![no_main]

use panic_halt as _;
use core::fmt::Write;
use embedded_hal::blocking::delay::DelayMs;

use bno055;

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::gpio::*;
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::{Pins, UART, entry};


#[entry]
fn main() -> ! {
    // 初期化処理
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);    
    let gclk0 = &clocks.gclk0();

    let mut pins = Pins::new(peripherals.PORT);

    let uart = UART {
        tx: pins.txd,
        rx: pins.rxd
    };
    let mut serial = uart.init(
        &mut clocks,
        115200.hz(),
        peripherals.SERCOM2,
        &mut peripherals.MCLK,
        &mut pins.port
    );

    delay.delay_ms(2000u32);

    let i2c: I2CMaster3<
        Sercom3Pad0<Pa17<PfD>>,
        Sercom3Pad1<Pa16<PfD>>
    > = I2CMaster3::new(
        &clocks.sercom3_core(&gclk0).unwrap(),
        400.khz(),
        peripherals.SERCOM3,
        &mut peripherals.MCLK,
        pins.i2c1_sda.into_pad(&mut pins.port),
        pins.i2c1_scl.into_pad(&mut pins.port)
    );

    delay.delay_ms(2000u32);
    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();
    let result = imu.init(&mut delay);
    if result.is_err() {
        writeln!(&mut serial, "error.\r").unwrap();
        result.unwrap();
    }
    imu.init(&mut delay).unwrap();
    writeln!(&mut serial, "BNO055 intialized.\r").unwrap();

    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
    writeln!(&mut serial, "BNO055 NDOF Mode.\r").unwrap();

    loop {
        if let Some(quat) = imu.quaternion().ok() {
            writeln!(&mut serial, "x: {}, y: {}, z: {}, s{}\r", quat.v.x, quat.v.y, quat.v.z, quat.s).unwrap();
        };
        delay.delay_ms(2000u32);
    }
}
