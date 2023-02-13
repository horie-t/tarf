#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal as wio;

use cortex_m::peripheral::NVIC;
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::*;
use wio::hal::hal::serial::*;
use wio::hal::timer::TimerCounter;
use wio::pac::{interrupt, Peripherals, TC3, TC5};
use wio::prelude::*;
use wio::{entry, Pins, Sets};



pub struct Led {
    pin: Pa15<Output<PushPull>>,
}

impl Led {
    pub fn new(pin: Pa15<Input<Floating>>, port: &mut Port) -> Led {
        Led {
            pin: pin.into_push_pull_output(port),
        }
    }

    pub fn turn_on(&mut self) {
        self.pin.set_high().unwrap();
    }

    pub fn turn_off(&mut self) {
        self.pin.set_low().unwrap();
    }

    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

struct Ctx {
    led: Led,
    tc5: TimerCounter<TC5>,
}
static mut CTX: Option<Ctx> = None;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut sets: Sets = Pins::new(peripherals.PORT).split();
    let gclk5 = clocks
        .get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5)
        .unwrap();
    let timer_clock5 = clocks.tc4_tc5(&gclk5).unwrap();
    let mut tc5 = TimerCounter::tc5_(&timer_clock5, peripherals.TC5, &mut peripherals.MCLK);

    unsafe {
        NVIC::unmask(interrupt::TC5);
    }

    tc5.start(2.s());
    tc5.enable_interrupt();

    unsafe {
        CTX = Some(Ctx {
            led: Led::new(sets.user_led, &mut sets.port),
            tc5,
        });
    }

    loop {
    }
}


#[interrupt]
fn TC5() {
    unsafe {
        let ctx = CTX.as_mut().unwrap();
        ctx.tc5.wait().unwrap();
        ctx.led.toggle();
    }
}