#![no_std]
#![no_main]

use panic_halt as _;
use wio::hal::pwm::{TCC0Pinout, Tcc0Pwm};

use core::fmt::Write;

use wio_terminal as wio;

use wio::{entry, Pins};
use wio::hal::clock::GenericClockController;
//use wio::hal::gpio::{self, v1::*};
use wio::hal::hal::Pwm;
use wio::hal::pwm::Channel;
use wio::pac::Peripherals;
use wio::prelude::*;

struct Wheel<P: Pwm> {
    pwm: P
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
    let gclk0 = clocks.gclk0();
    let mut sets = Pins::new(peripherals.PORT).split();

    // UART出力
    let mut serial = sets.uart.init(
        &mut clocks,
        115200.hz(),
        peripherals.SERCOM2,
        &mut peripherals.MCLK,
        &mut sets.port
    );
    writeln!(&mut serial, "{} start!\r", "tarf").unwrap();

    let pinout = TCC0Pinout::Pd11(sets.buzzer.ctr.into_function_f(&mut sets.port));
    let mut wheel = Wheel { 
        pwm: Tcc0Pwm::new(
            & clocks.tcc0_tcc1(&gclk0).unwrap(),
            1.khz(),
            peripherals.TCC0,
            pinout,
            &mut peripherals.MCLK
        )
    };

    wheel.pwm.set_period(261.hz());
    let max_duty= wheel.pwm.get_max_duty();
    wheel.pwm.set_duty(Channel::_4, max_duty / 2);

    loop {

    }
}
