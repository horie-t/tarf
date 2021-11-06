#![no_std]
#![no_main]

use panic_halt as _;
use core::fmt::Write;

use wio_terminal as wio;
use wio::{Pins, UART, entry};
use wio::hal::clock::GenericClockController;
use wio::pac::Peripherals;
use wio::prelude::*;


#[entry]
fn main() -> ! {
    // 初期化処理
    let mut peripherals = Peripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

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

    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

    loop {
    }
}
