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

struct Ctx<DIRPIN: PinId, STEPPIN: PinId, TC> {
    dir: Pin<DIRPIN, Output<PushPull>>,
    step: Pin<STEPPIN, Output<PushPull>>,
    tc: TimerCounter<TC>
}

// static mut CTX0: Option<Ctx<PA07, PB04, TC2>> = None;
static mut CTX1: Option<Ctx<PB08, PB09, TC3>> = None;
static mut CTX2: Option<Ctx<PB05, PB06, TC4>> = None;

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

    // let mut ctx0 = Ctx {
    //     dir: pins.a2_d2.into_push_pull_output(&mut pins.port),
    //     step: pins.a3_d3.into_push_pull_output(&mut pins.port),
    //     tc: TimerCounter::tc2_(&timer_clock, peripherals.TC2, &mut peripherals.MCLK)
    // };
    // ctx0.tc.start(1.ms());
    // ctx0.tc.enable_interrupt();
    // ctx0.dir.set_high().unwrap();

    let mut ctx1 = Ctx {
        dir: pins.a0_d0.into_push_pull_output(&mut pins.port),
        step: pins.a1_d1.into_push_pull_output(&mut pins.port),
        tc: TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK)
    };
    ctx1.tc.start(10.ms());
    ctx1.tc.enable_interrupt();
    ctx1.dir.set_low().unwrap();

    let mut ctx2 = Ctx {
        dir: pins.a4_d4.into_push_pull_output(&mut pins.port),
        step: pins.a5_d5.into_push_pull_output(&mut pins.port),
        tc: TimerCounter::tc4_(&timer_clock1, peripherals.TC4, &mut peripherals.MCLK)
    };
    ctx2.tc.start(10.ms());
    ctx2.tc.enable_interrupt();
    ctx2.dir.set_high().unwrap();

    unsafe {
        NVIC::unmask(interrupt::TC3);
        // NVIC::unmask(interrupt::TC2);
        NVIC::unmask(interrupt::TC4);
        // CTX0 = Some(ctx0);
        CTX1 = Some(ctx1);
        CTX2 = Some(ctx2);
    }

    loop {
    }
}

// static mut CTX0_COUNT: i32  = 0;

// #[interrupt]
// fn TC2() {
//     unsafe {
//         let ctx = CTX0.as_mut().unwrap();
//         ctx.tc.wait().unwrap();
//         ctx.step.toggle();

//         CTX0_COUNT = CTX0_COUNT + 1;
//         if CTX0_COUNT == 400 {
//             ctx.dir.toggle();
//             CTX0_COUNT = 0;
//         }
//     }
// }

static mut CTX1_COUNT: i32  = 0;

#[interrupt]
fn TC3() {
    unsafe {
        let ctx = CTX1.as_mut().unwrap();
        ctx.tc.wait().unwrap();
        ctx.step.toggle();

        CTX1_COUNT = CTX1_COUNT + 1;
        if CTX1_COUNT == 400 {
            ctx.dir.toggle();
            CTX1_COUNT = 0;
        }
    }
}

static mut CTX2_COUNT: i32  = 0;

#[interrupt]
fn TC4() {
    unsafe {
        let ctx = CTX2.as_mut().unwrap();
        ctx.tc.wait().unwrap();
        ctx.step.toggle();

        CTX2_COUNT = CTX2_COUNT + 1;
        if CTX2_COUNT == 400 {
            ctx.dir.toggle();
            CTX2_COUNT = 0;
        }
    }
}
