#![no_std]
#![no_main]

use heapless::consts::U16;
use panic_halt as _;

use core::fmt::Write;

use cortex_m::interrupt::{free as disable_interrupts};
use cortex_m::peripheral::NVIC;

use wio::hal::time::{Microseconds, Milliseconds, Nanoseconds};
use wio_terminal as wio;

use wio::{Microphone, Pins, Sets, entry};
use wio::hal::clock::GenericClockController;
use wio::hal::gpio::v1::*;
use wio::hal::gpio::v2::{PA07, PB04, PB05, PB06, PB08, PB09};
use wio::hal::timer::{Count16, TimerCounter};
use wio::pac::{interrupt, Peripherals, TC3, TC2, TC4};
use wio::prelude::*;

use heapless::spsc::Queue;

#[derive(Clone, Copy)]
enum Wheel {
    Front,
    Right,
    Left
}

struct WheelEvent {
    id: Wheel,
    dir: bool
}

macro_rules! wheel_interrupt {
    ($Handler:ident, $WheelController:ident) => {
        #[interrupt]
        fn $Handler() {
            disable_interrupts(|_cs| unsafe {
                let mut wheel = $WheelController.as_mut().unwrap();
                // wheel.tc.wait().unwrap();
                if (wheel.is_step_pin_move) {
                    if wheel.is_step_pin_high {
                        wheel.is_step_pin_high = false;
                        wheel.step_pin.set_low().unwrap();
                    } else {
                        wheel.is_step_pin_high = true;
                        wheel.step_pin.set_high().unwrap();
                    }
                    let mut queue = WHEEL_EVENT_Q.split().0;
                    queue.enqueue(WheelEvent{ id: wheel.id, dir: wheel.is_dir_pin_high}).ok();
                }
                if wheel.timeout.0 > (500 * 1000).ns().0 {
                    wheel.timeout = Nanoseconds(wheel.timeout.0 - (10 * 1000).ns().0);
                    wheel.tc.start(wheel.timeout);
                    wheel.tc.enable_interrupt();
                } else {
                    wheel.tc.wait().unwrap();
                }
                // let mut command_queue = wheel.command_queue.split().1;
                // if let Some(_command) = command_queue.dequeue() {
                //     wheel.toggle();
                // }
            });
        }
    };
}

struct WheelAssembly<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> {
    id: Wheel,
    dir_pin: Pin<DIRPIN, Output<PushPull>>,
    is_dir_pin_high: bool,
    step_pin: Pin<STEPPIN, Output<PushPull>>,
    is_step_pin_high: bool,
    is_step_pin_move: bool,
    tc: TimerCounter<TC>,
    timeout: Nanoseconds,
    command_queue: Queue<WheelCommand, U16>
}

impl<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> WheelAssembly<DIRPIN, STEPPIN, TC> {
    pub fn new(id: Wheel, dir: Pin<DIRPIN, Output<PushPull>>, step: Pin<STEPPIN, Output<PushPull>>,
        tc: TimerCounter<TC>, interrupt: interrupt) -> WheelAssembly<DIRPIN, STEPPIN, TC> {
            let mut wheel = WheelAssembly { id,
                dir_pin: dir, is_dir_pin_high: true, 
                step_pin: step, is_step_pin_high: true, is_step_pin_move: false,
                tc, timeout: u32::MAX.ns(),
                command_queue: Queue(heapless::i::Queue::new())};
            wheel.dir_pin.set_high().unwrap();
            unsafe {
                NVIC::unmask(interrupt);
            }
            wheel
        }
}


trait WheelController {
    fn start<T>(&mut self, timeout: T) where T: Into<Nanoseconds>;
    fn stop(&mut self);
    fn restart(&mut self);
    fn toggle(&mut self);
}



impl<DIRPIN: PinId, STEPPIN: PinId, TC: Count16> WheelController for WheelAssembly<DIRPIN, STEPPIN, TC> {
    fn start<T>(&mut self, timeout: T) where T: Into<Nanoseconds> {
        self.timeout = timeout.into();
        self.tc.start(self.timeout);
        self.tc.enable_interrupt();
        self.is_step_pin_move = true;
    }

    fn stop(&mut self) {
        self.is_step_pin_move = false;
    }

    fn restart(&mut self) {
        self.is_step_pin_move = true;
    }

    fn toggle(&mut self) {
        if self.is_dir_pin_high {
            self.is_dir_pin_high = false;
            self.dir_pin.set_low().unwrap();
        } else {
            self.is_dir_pin_high = true;
            self.dir_pin.set_high().unwrap();
        }
    }
}

struct WheelCommand {
    
}

static mut WHEEL_FRONT: Option<WheelAssembly<PB08, PB09, TC2>> = None;
static mut WHEEL_RIGHT: Option<WheelAssembly<PA07, PB04, TC3>> = None;
static mut WHEEL_LEFT: Option<WheelAssembly<PB05, PB06, TC4>> = None;

static mut WHEEL_EVENT_Q: Queue<WheelEvent, U16> = Queue(heapless::i::Queue::new());

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

    let mut sets = Pins::new(peripherals.PORT).split();
    let mut serial = sets.uart.init(
        &mut clocks,
        115200.hz(),
        peripherals.SERCOM2,
        &mut peripherals.MCLK,
        &mut sets.port
    );

    let header_pins = sets.header_pins;

    let mut wheel_front = WheelAssembly::new(Wheel::Front,
        header_pins.a0_d0.into_push_pull_output(&mut sets.port),
        header_pins.a1_d1.into_push_pull_output(&mut sets.port),
        TimerCounter::tc2_(&timer_clock, peripherals.TC2, &mut peripherals.MCLK), interrupt::TC2);
    wheel_front.start(10.ms());
    wheel_front.stop();

    let mut wheel_right = WheelAssembly::new(Wheel::Right,
        header_pins.a2_d2.into_push_pull_output(&mut sets.port),
        header_pins.a3_d3.into_push_pull_output(&mut sets.port),
        TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK), interrupt::TC3);
    wheel_right.start(10.ms());

    let mut wheel_left = WheelAssembly::new(Wheel::Left,
        header_pins.a4_d4.into_push_pull_output(&mut sets.port),
        header_pins.a5_d5.into_push_pull_output(&mut sets.port),
        TimerCounter::tc4_(&timer_clock1, peripherals.TC4, &mut peripherals.MCLK), interrupt::TC4);
    wheel_left.start(10.ms());
    wheel_left.toggle();

    unsafe {
        WHEEL_FRONT = Some(wheel_front);
        WHEEL_RIGHT = Some(wheel_right);
        WHEEL_LEFT = Some(wheel_left);
        wheel_interrupt!(TC2, WHEEL_FRONT);
        wheel_interrupt!(TC3, WHEEL_RIGHT);
        wheel_interrupt!(TC4, WHEEL_LEFT);

        WHEEL_EVENT_Q.split();
    }

    let mut consumer = unsafe { WHEEL_EVENT_Q.split().1 };

    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

    loop {
        if let Some(event) = consumer.dequeue() {

        }
    }
}
