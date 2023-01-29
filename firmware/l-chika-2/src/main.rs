#![no_std]
#![no_main]

use panic_halt as _;

use embedded_hal::blocking::delay::DelayMs;

use wio_terminal::entry;
use wio_terminal::hal::clock::GenericClockController;
use wio_terminal::hal::delay::Delay;
use wio_terminal::pac::{CorePeripherals, Peripherals};
use wio_terminal::prelude::*;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = wio_terminal::Pins::new(peripherals.PORT).split();
    let mut led = pins.user_led.into_push_pull_output();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    loop {
        led.toggle().ok();
        delay.delay_ms(1_000u16);
    }
}
