#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::peripheral::NVIC;

use wio_terminal as wio;

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::*;
use wio::hal::gpio::v2::{PA07, PB04, PB05, PB06, PB08, PB09, PinId};
use wio::hal::timer::TimerCounter;
use wio::pac::{interrupt, Peripherals, TC3, TC2, TC4};
use wio::prelude::*;

struct Wheel<DIRPIN: PinId, STEPPIN: PinId, TC> {
    dir: Pin<DIRPIN, Output<PushPull>>,
    step: Pin<STEPPIN, Output<PushPull>>,
    tc: TimerCounter<TC>
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

    let mut wheel_0 = Wheel {
        dir: pins.a0_d0.into_push_pull_output(&mut pins.port),
        step: pins.a1_d1.into_push_pull_output(&mut pins.port),
        tc: TimerCounter::tc2_(&timer_clock, peripherals.TC2, &mut peripherals.MCLK)
    };
    wheel_0.tc.start(10.ms());
    wheel_0.tc.enable_interrupt();
    wheel_0.dir.set_high().unwrap();

    let mut wheel_1 = Wheel {
        dir: pins.a2_d2.into_push_pull_output(&mut pins.port),
        step: pins.a3_d3.into_push_pull_output(&mut pins.port),
        tc: TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK)
    };
    wheel_1.tc.start(10.ms());
    wheel_1.tc.enable_interrupt();
    wheel_1.dir.set_high().unwrap();

    let mut wheel_2 = Wheel {
        dir: pins.a4_d4.into_push_pull_output(&mut pins.port),
        step: pins.a5_d5.into_push_pull_output(&mut pins.port),
        tc: TimerCounter::tc4_(&timer_clock1, peripherals.TC4, &mut peripherals.MCLK)
    };
    wheel_2.tc.start(10.ms());
    wheel_2.tc.enable_interrupt();
    wheel_2.dir.set_high().unwrap();

    unsafe {
        NVIC::unmask(interrupt::TC2);
        NVIC::unmask(interrupt::TC3);
        NVIC::unmask(interrupt::TC4);
        WHEEL_0 = Some(wheel_0);
        WHEEL_1 = Some(wheel_1);
        WHEEL_2 = Some(wheel_2);
    }

    loop {
    }
}

#[interrupt]
fn TC2() {
    unsafe {
        let ctx = WHEEL_0.as_mut().unwrap();
        ctx.tc.wait().unwrap();
        ctx.step.toggle();
    }
}

#[interrupt]
fn TC3() {
    unsafe {
        let ctx = WHEEL_1.as_mut().unwrap();
        ctx.tc.wait().unwrap();
        ctx.step.toggle();
    }
}

#[interrupt]
fn TC4() {
    unsafe {
        let ctx = WHEEL_2.as_mut().unwrap();
        ctx.tc.wait().unwrap();
        ctx.step.toggle();
    }
}
