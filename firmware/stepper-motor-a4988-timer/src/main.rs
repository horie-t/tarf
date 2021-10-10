#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::peripheral::NVIC;

use wio_terminal as wio;

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::*;
use wio::hal::gpio::v2::{PB04, PB06, PB09};
use wio::hal::timer::TimerCounter;
use wio::pac::{interrupt, Peripherals, TC3, TC2, TC4};
use wio::prelude::*;

struct Ctx2 {
    stepper_motor: Pin<PB04, Output<PushPull>>,
    tc2: TimerCounter<TC2>
}

static mut CTX2: Option<Ctx2> = None;

struct Ctx3 {
    stepper_motor: Pin<PB09, Output<PushPull>>,
    tc3: TimerCounter<TC3>
}

static mut CTX3: Option<Ctx3> = None;

struct Ctx4 {
    stepper_motor: Pin<PB06, Output<PushPull>>,
    tc4: TimerCounter<TC4>
}

static mut CTX4: Option<Ctx4> = None;

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

    //let mut tc1 = TimerCounter::tc
    let mut tc3 = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK);
    let mut tc2 = TimerCounter::tc2_(&timer_clock, peripherals.TC2, &mut peripherals.MCLK);
    let mut tc4 = TimerCounter::tc4_(&timer_clock1, peripherals.TC4, &mut peripherals.MCLK);
    unsafe {
        NVIC::unmask(interrupt::TC3);
        NVIC::unmask(interrupt::TC2);
        NVIC::unmask(interrupt::TC4);
    }
    tc3.start(1.ms());
    tc3.enable_interrupt();
    tc2.start(3.ms());
    tc2.enable_interrupt();
    tc4.start(5.ms());
    tc4.enable_interrupt();

    let mut pins = wio::Pins::new(peripherals.PORT);

    let mut dir1 = pins.a0_d0.into_push_pull_output(&mut pins.port);
    dir1.set_high().unwrap();
    let stepper_motor1 = pins.a1_d1.into_push_pull_output(&mut pins.port);

    let mut dir2 = pins.a2_d2.into_push_pull_output(&mut pins.port);
    dir2.set_high().unwrap();
    let stepper_motor2 = pins.a3_d3.into_push_pull_output(&mut pins.port);

    let mut dir3 = pins.a4_d4.into_push_pull_output(&mut pins.port);
    dir3.set_high().unwrap();
    let stepper_motor3 = pins.a5_d5.into_push_pull_output(&mut pins.port);

    unsafe {
        CTX3 = Some(Ctx3 {
            stepper_motor: stepper_motor1,
            tc3
        });
        CTX2 = Some(Ctx2 {
            stepper_motor: stepper_motor2,
            tc2
        });
         CTX4 = Some(Ctx4 {
            stepper_motor: stepper_motor3,
            tc4
        })
    }

    loop {
    }
}

#[interrupt]
fn TC3() {
    unsafe {
        let ctx = CTX3.as_mut().unwrap();
        ctx.tc3.wait().unwrap();
        ctx.stepper_motor.toggle();
    }
}

#[interrupt]
fn TC2() {
    unsafe {
        let ctx = CTX2.as_mut().unwrap();
        ctx.tc2.wait().unwrap();
        ctx.stepper_motor.toggle();
    }
}

#[interrupt]
fn TC4() {
    unsafe {
        let ctx = CTX4.as_mut().unwrap();
        ctx.tc4.wait().unwrap();
        ctx.stepper_motor.toggle();
    }
}
