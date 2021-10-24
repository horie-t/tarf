#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::peripheral::NVIC;

use wio::hal::time::Milliseconds;
use wio_terminal as wio;

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::v2::*;
use wio::hal::timer::{Count16, TimerCounter};
use wio::pac::{interrupt, Peripherals, TC3, TC2, TC4};
use wio::prelude::*;

enum Wheel {
    Front,
    Right,
    Left
}

macro_rules! wheel_interrupt {
    ($Handler:ident, $WheelController:ident) => {
        #[interrupt]
        fn $Handler() {
            unsafe {
                let wheel = $WheelController.as_mut().unwrap();
                wheel.tc.wait().unwrap();
                wheel.step.toggle().unwrap();
            }
        }
    };
}

struct WheelController<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> {
    id: Wheel,
    dir: Pin<DIRPIN, PushPullOutput>,
    step: Pin<STEPPIN, PushPullOutput>,
    tc: TimerCounter<TC>
}

impl<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> WheelController<DIRPIN, STEPPIN, TC> {
    pub fn new(id: Wheel, dir: Pin<DIRPIN, PushPullOutput>, step: Pin<STEPPIN, PushPullOutput>,
        tc: TimerCounter<TC>, interrupt: interrupt) -> WheelController<DIRPIN, STEPPIN, TC> {
            let mut wheel = WheelController { id, dir, step, tc };
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

static mut WHEEL_FRONT: Option<WheelController<PB08, PB09, TC2>> = None;
static mut WHEEL_RIGHT: Option<WheelController<PA07, PB04, TC3>> = None;
static mut WHEEL_LEFT: Option<WheelController<PB05, PB06, TC4>> = None;

// static mut WHEEL_EVENT_Q: Queue<WheelEvent, 8> = Queue::new();

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

    let pins = Pins::new(peripherals.PORT);

    let mut wheel_front = WheelController::new(Wheel::Front, pins.pb08.into_push_pull_output(), pins.pb09.into_push_pull_output(),
        TimerCounter::tc2_(&timer_clock, peripherals.TC2, &mut peripherals.MCLK), interrupt::TC2);
    wheel_front.start(10.ms());

    let mut wheel_right = WheelController::new(Wheel::Right, pins.pa07.into_push_pull_output(), pins.pb04.into_push_pull_output(),
        TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK), interrupt::TC3);
    wheel_right.start(10.ms());

    let mut wheel_left = WheelController::new(Wheel::Left, pins.pb05.into_push_pull_output(), pins.pb06.into_push_pull_output(),
        TimerCounter::tc4_(&timer_clock1, peripherals.TC4, &mut peripherals.MCLK), interrupt::TC4);
    wheel_left.start(10.ms());

    unsafe {
        WHEEL_FRONT = Some(wheel_front);
        WHEEL_RIGHT = Some(wheel_right);
        WHEEL_LEFT = Some(wheel_left);
        wheel_interrupt!(TC2, WHEEL_FRONT);
        wheel_interrupt!(TC3, WHEEL_RIGHT);
        wheel_interrupt!(TC4, WHEEL_LEFT);
    }

    loop {
    }
}
