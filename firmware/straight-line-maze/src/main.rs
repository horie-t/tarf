#![no_std]
#![no_main]

use panic_halt as _;

use wio_terminal as wio;
use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::v2::{Output, Pin, PinId, Pins, PushPull};
use wio::pac::Peripherals;
use wio::prelude::*;


struct Wheel<S: PinId, D: PinId> {
    id: u32,
    step_pin: Pin<S, Output<PushPull>>,
    direction_pin: Pin<D, Output<PushPull>>,
}

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

    let gclk5 = clocks.get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5)
        .unwrap();
    let timer_clock = clocks.tc2_tc3(&gclk5).unwrap();
    let timer_clock1 = clocks.tc4_tc5(&gclk5).unwrap();

    let pins = Pins::new(peripherals.PORT);
        
    let wheel_0 = Wheel {
        id: 0,
        step_pin: pins.pb08.into_push_pull_output(),
        direction_pin: pins.pb09.into_push_pull_output(),
    };

    loop {
    }
}
