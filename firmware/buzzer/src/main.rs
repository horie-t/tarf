#![no_std]
#![no_main]

use embedded_hal::Pwm;
use panic_halt as _;
use wio_terminal as wio;

use embedded_hal::blocking::delay::DelayMs;

use wio::{entry, Pins};
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::pwm::Channel;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;

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
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut sets = Pins::new(peripherals.PORT).split();
    let mut buzzer = sets.buzzer.init(
        &mut clocks, peripherals.TCC0, &mut peripherals.MCLK, &mut sets.port);

    loop {
        buzzer.set_period(500.hz());
        let max_duty=  buzzer.get_max_duty();
        buzzer.set_duty(Channel::_4, max_duty / 2);

        buzzer.enable(Channel::_4);
        delay.delay_ms(2_000u16);
        buzzer.disable(Channel::_4);
        delay.delay_ms(2_000u16);
    }
}
