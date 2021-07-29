#![no_std]
#![no_main]

use panic_halt as _;

use embedded_hal::blocking::delay::DelayMs;

use wio_terminal::entry;
use wio_terminal::hal::clock::GenericClockController;
use wio_terminal::hal::delay::Delay;
use wio_terminal::hal::gpio::{Port, Pa17, Input, Floating, Output, PushPull};
use wio_terminal::pac::{CorePeripherals, Peripherals};

struct Led {
    pin: Pa17<Output<PushPull>>,
}

impl Led {
    fn new(pin: Pa17<Input<Floating>>, port: &mut Port) -> Led {
        Led {
            pin: pin.into_push_pull_output(port),
        }
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
    let mut led = Led::new(pins.i2c1_sda, &mut pins.port);
    let mut delay = Delay::new(core.SYST, &mut clocks);

    loop {
        delay.delay_ms(1_000u16);
        led.toggle();
    }
}
