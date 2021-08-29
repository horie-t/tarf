#![no_std]
#![no_main]

use embedded_hal::digital::v2::StatefulOutputPin;
use panic_halt as _;

use embedded_hal::blocking::delay::DelayMs;

use wio_terminal::entry;
use wio_terminal::hal::clock::GenericClockController;
use wio_terminal::hal::delay::Delay;
use wio_terminal::hal::gpio::*;
use wio_terminal::pac::{CorePeripherals, Peripherals};
use wio_terminal::prelude::*;

struct Led<I: v2::PinId> {
    pin: v1::Pin<I, Output<PushPull>>,
}

impl<I: v2::PinId> Led<I> {
    fn new(pin: v1::Pin<I, Input<Floating>>, port: &mut Port) -> Led<I> {
        Led {
            pin: pin.into_push_pull_output(port),
        }
    }
    
    pub fn set_high(&mut self) {
        self.pin.set_high().unwrap();
    }

    pub fn set_low(&mut self) {
        self.pin.set_low().unwrap();
    }

    pub fn is_set_high(&mut self) -> bool {
        self.pin.is_set_high().unwrap()
    }

    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

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

    let mut pins = wio_terminal::Pins::new(peripherals.PORT);
    let mut led_external = Led::new(pins.i2c1_sda, &mut pins.port);
    led_external.set_low();
    let mut led_internal = Led::new(pins.user_led, &mut pins.port);
    led_internal.set_low();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    loop {
        delay.delay_ms(3_000u16);
        if ! led_external.is_set_high() {
            led_external.set_high();
        }
    }
}
