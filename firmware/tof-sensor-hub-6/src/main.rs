#![no_std]
#![no_main]

use panic_halt as _;
use core::fmt::Write;
use vl53l0x::VL53L0x;
use xca9548a::{SlaveAddr, Xca9548a};

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

    let i2c_switch = Xca9548a::new(i2c, SlaveAddr::default());
    let i2c_ports = i2c_switch.split();

    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

    delay.delay_ms(2000u32);
    let mut sensors = [
        VL53L0x::new(i2c_ports.i2c0).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c1).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c2).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c3).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c4).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c5).ok().unwrap()
    ];

    writeln!(&mut serial, "VL53L0x intialized.\r").unwrap();

    loop {
        for sensor in sensors.iter_mut() {
            if let Some(distance) = (*sensor).read_range_single_millimeters_blocking().ok() {
                write!(&mut serial, "0: {}, ", distance).unwrap();
            }
        }
        write!(&mut serial, "\r\n").unwrap();
        delay.delay_ms(500u32);
    }
}
