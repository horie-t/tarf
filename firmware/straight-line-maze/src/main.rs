#![no_std]
#![no_main]

use cortex_m::interrupt::{free as disable_interrupts};
use cortex_m::peripheral::NVIC;
use heapless::consts::U8;
use heapless::spsc::Queue;

use panic_halt as _;

use wio::hal::eic::ConfigurableEIC;
use wio_terminal as wio;
use wio::{entry, Pins};
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::{ExtInt12, ExternalInterrupt, Sense};
use wio::hal::gpio::v1::{Port, Pc28};
use wio::hal::gpio::v2::{Floating, Input, Interrupt, Output, PA07, PB04, PB05, PB06, PB08, PB09, Pin, PinId, PushPull};
use wio::hal::timer::{Count16, TimerCounter};
use wio::pac::{CorePeripherals, Peripherals, TC2, TC3, TC4, interrupt};
use wio::prelude::*;

#[derive(Clone, Copy)]
enum WheelRotateDirection {
    ClockWise,
    CounterClockWise,
}

struct Wheel<S: PinId, D: PinId, T: Count16> {
    id: u32,
    step_pin: Pin<S, Output<PushPull>>,
    direction_pin: Pin<D, Output<PushPull>>,
    timer_counter: TimerCounter<T>,
}

impl<S: PinId, D: PinId, T: Count16> Wheel<S, D, T> {
    fn new(id: u32, step_pin: Pin<S, Input<Floating>>, direction_pin: Pin<D, Input<Floating>>,
        timer_counter: TimerCounter<T>, interrupt: interrupt) -> Wheel<S, D, T>{
            let mut wheel = Wheel {
                id,
                step_pin: step_pin.into_push_pull_output(),
                direction_pin: direction_pin.into_push_pull_output(),
                timer_counter
            };
            wheel.set_rotate_direction(WheelRotateDirection::ClockWise);

            unsafe {
                NVIC::unmask(interrupt);
            }

            wheel
    }

    fn step(&mut self) {
        self.step_pin.toggle().unwrap();
    }

    fn set_rotate_direction(&mut self, dir: WheelRotateDirection) {
        match dir {
            WheelRotateDirection::ClockWise => {
                self.direction_pin.set_high().unwrap();
            },
            WheelRotateDirection::CounterClockWise => {
                self.direction_pin.set_low().unwrap();
            }
        }
    }

    fn start(&mut self, rpm: f32) {
        let pulse_hz = (((2_f32 * 200_f32) / 60_f32 * rpm) as u32).hz();
        self.timer_counter.start(pulse_hz);
        self.timer_counter.enable_interrupt();
    }
}

macro_rules! wheel_interrupt {
    ($RunningSystem:ident, $Handler:ident, $Wheel:ident) => {
        #[interrupt]
        fn $Handler() {
            disable_interrupts(|_cs| unsafe {
                let running_system = $RunningSystem.as_mut().unwrap();
                running_system.$Wheel.step();
                running_system.$Wheel.timer_counter.wait().unwrap();
            });
        }
    };
}

struct RunningSystem<S0: PinId, D0: PinId, T0: Count16, S1: PinId, D1: PinId, T1: Count16, S2: PinId, D2: PinId, T2: Count16> {
    wheel_0: Wheel<S0, D0, T0>,
    wheel_1: Wheel<S1, D1, T1>,
    wheel_2: Wheel<S2, D2, T2>,
}

static mut RUNNING_SYSTEM: Option<RunningSystem<PB08, PB09, TC2, PA07, PB04, TC3, PB05, PB06, TC4>> = None;

struct Button {
    queue: Queue<ButtonEvent, U8>,
    pin: ExtInt12<Pc28<Interrupt<Floating>>>,
}

impl Button {
    pub fn new(
        pin: Pc28<Input<Floating>>,
        queue: Queue<ButtonEvent, U8>,
        eic: &mut ConfigurableEIC,
        port: &mut Port,
    ) -> Button {
        eic.button_debounce_pins(&[pin.id()]);

        let mut pin = pin.into_floating_ei(port);
        pin.sense(eic, Sense::FALL);
        pin.enable_interrupt(eic);

        Button {
            pin, queue
        }
    }

    pub fn enable(&self, nvic: &mut NVIC, interrupt: interrupt) {
        unsafe {
            nvic.set_priority(interrupt, 1);
            NVIC::unmask(interrupt);
        }
    }
}

struct ButtonEvent {
    pressed: bool,
}

macro_rules! button_interrupt {
    ($Button:ident, $Interrupt:ident) => {
        #[interrupt]
        fn $Interrupt() {
            disable_interrupts(|_cs| unsafe {
                let button = $Button.as_mut().unwrap();
                if button.pin.is_interrupt() {
                    button.pin.clear_interrupt();
                    let mut q = button.queue.split().0;
                    q.enqueue(ButtonEvent { pressed: true }).ok();
                }
            });
        }
    };
}

static mut START_BUTTON: Option<Button> = None;

#[entry]
fn main() -> ! {
    // 初期化処理
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let clk = clocks.gclk1();
    let mut configurable_eic = eic::init_with_ulp32k(&mut peripherals.MCLK, clocks.eic(&clk).unwrap(), peripherals.EIC);

    let gclk5 = clocks.get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5)
        .unwrap();
    let tc2_tc3 = clocks.tc2_tc3(&gclk5).unwrap();
    let tc4_tc5 = clocks.tc4_tc5(&gclk5).unwrap();

    let mut pins = Pins::new(peripherals.PORT);
    
    // 走行装置の初期化
    let mut running_system = RunningSystem {
        wheel_0: Wheel::new(0, pins.a0_d0.into(), pins.a1_d1.into(),
            TimerCounter::tc2_(&tc2_tc3, peripherals.TC2, &mut peripherals.MCLK), interrupt::TC2),
        wheel_1: Wheel::new(1, pins.a2_d2.into(), pins.a3_d3.into(),
            TimerCounter::tc3_(&tc2_tc3, peripherals.TC3, &mut peripherals.MCLK), interrupt::TC3),
        wheel_2: Wheel::new(2, pins.a4_d4.into(), pins.a5_d5.into(),
            TimerCounter::tc4_(&tc4_tc5, peripherals.TC4, &mut peripherals.MCLK), interrupt::TC4),
    };
    running_system.wheel_1.set_rotate_direction(WheelRotateDirection::CounterClockWise);

    unsafe {
        RUNNING_SYSTEM = Some(running_system);
        wheel_interrupt!(RUNNING_SYSTEM, TC2, wheel_0);
        wheel_interrupt!(RUNNING_SYSTEM, TC3, wheel_1);
        wheel_interrupt!(RUNNING_SYSTEM, TC4, wheel_2);
    }

    // スタートボタンの初期化
    let start_button = Button::new(
        pins.button3,
        Queue::new(),
        &mut configurable_eic,
        &mut pins.port,
    );
    let nvic = &mut core.NVIC;
    disable_interrupts(|_| unsafe {
        start_button.enable(nvic, interrupt::EIC_EXTINT_12);
        START_BUTTON = Some(start_button);
    });
    button_interrupt!(START_BUTTON, EIC_EXTINT_12);

    configurable_eic.finalize();

    let mut consumer = unsafe { START_BUTTON.as_mut().unwrap().queue.split().1 };
    loop {
        if let Some(event) = consumer.dequeue() {
            if event.pressed {
                unsafe {
                    let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                    running_system.wheel_0.start(10_f32);
                    running_system.wheel_1.start(20_f32);
                    running_system.wheel_2.start(10_f32);
                }
            }
        }
    }
}
