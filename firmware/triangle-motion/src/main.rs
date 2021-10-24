#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::peripheral::NVIC;

use wio::hal::time::Milliseconds;
use wio_terminal as wio;

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::*;
use wio::hal::gpio::v2::{PA07, PB04, PB05, PB06, PB08, PB09, PinId};
use wio::hal::timer::{Count16, TimerCounter};
use wio::pac::{interrupt, Peripherals, TC3, TC2, TC4};
use wio::prelude::*;

macro_rules! wheel_interrupt {
    ($Handler:ident, $Wheel:ident) => {
        #[interrupt]
        fn $Handler() {
            unsafe {
                let wheel = $Wheel.as_mut().unwrap();
                wheel.tc.wait().unwrap();
                wheel.step.toggle();
            }
        }
    };
}

struct Wheel<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> {
    dir: Pin<DIRPIN, Output<PushPull>>,
    step: Pin<STEPPIN, Output<PushPull>>,
    tc: TimerCounter<TC>
}

impl<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> Wheel<DIRPIN, STEPPIN, TC> {
    pub fn new(mut port: &mut v1::Port, dir: Pin<DIRPIN, Input<Floating>>, step: Pin<STEPPIN, Input<Floating>>, 
        tc: TimerCounter<TC>, interrupt: interrupt) -> Wheel<DIRPIN, STEPPIN, TC> {
            let mut wheel = Wheel {
                dir: dir.into_push_pull_output(&mut port), 
                step: step.into_push_pull_output(&mut port), 
                tc
            };
            wheel.dir.set_high().unwrap();
            unsafe {
                NVIC::unmask(interrupt);
            }
            wheel
        }

    pub fn start(&mut self, timeout: Milliseconds) {
        self.tc.start(timeout);
        self.tc.enable_interrupt();
    }
}

static mut WHEEL_0: Option<Wheel<PB08, PB09, TC2>> = None;
static mut WHEEL_1: Option<Wheel<PA07, PB04, TC3>> = None;
static mut WHEEL_2: Option<Wheel<PB05, PB06, TC4>> = None;

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

    let mut pins = wio::Pins::new(peripherals.PORT);

    let mut wheel_0 = Wheel::new(&mut pins.port, pins.a0_d0, pins.a1_d1, 
        TimerCounter::tc2_(&timer_clock, peripherals.TC2, &mut peripherals.MCLK), interrupt::TC2);
    wheel_0.start(10.ms());

    let mut wheel_1 = Wheel::new(&mut pins.port, pins.a2_d2, pins.a3_d3,
        TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK), interrupt::TC3);
    wheel_1.start(10.ms());

    let mut wheel_2 = Wheel::new(&mut pins.port, pins.a4_d4, pins.a5_d5,
        TimerCounter::tc4_(&timer_clock1, peripherals.TC4, &mut peripherals.MCLK), interrupt::TC4);
    wheel_2.start(10.ms());

    unsafe {
        WHEEL_0 = Some(wheel_0);
        WHEEL_1 = Some(wheel_1);
        WHEEL_2 = Some(wheel_2);
        wheel_interrupt!(TC2, WHEEL_0);
        wheel_interrupt!(TC3, WHEEL_1);
        wheel_interrupt!(TC4, WHEEL_2);
    }

    loop {
    }
}
