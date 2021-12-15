#![no_std]
#![no_main]

use panic_halt as _;

use wio_terminal as wio;
use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::gpio::v2::{Output, Pin, PinId, Pins, PushPull};
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;

#[derive(Clone, Copy)]
enum WheelRotateDirection {
    ClockWise,
    CounterClockWise,
}

struct Wheel<S: PinId, D: PinId> {
    id: u32,
    step_pin: Pin<S, Output<PushPull>>,
    direction_pin: Pin<D, Output<PushPull>>,
}

impl<S: PinId, D: PinId> Wheel<S, D> {
    fn step(&mut self) {
        self.step_pin.toggle().unwrap();
    }

    fn set_rotate_direction(&mut self, dir: WheelRotateDirection) {
        match dir {
            WheelRotateDirection::ClockWise => {
                self.direction_pin.set_high();
            },
            WheelRotateDirection::CounterClockWise => {
                self.direction_pin.set_low();
            }
        }
    }
}

struct RunningSystem<S0: PinId, D0: PinId> {
    wheel_0: Wheel<S0, D0>,
}

#[entry]
fn main() -> ! {
    // 初期化処理
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

    let gclk5 = clocks.get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5)
        .unwrap();
    let timer_clock = clocks.tc2_tc3(&gclk5).unwrap();
    let timer_clock1 = clocks.tc4_tc5(&gclk5).unwrap();

    let pins = Pins::new(peripherals.PORT);
        
    let wheel_0 = Wheel {
        id: 0,
        step_pin: pins.pb08.into_push_pull_output(),
        direction_pin: pins.pb09.into_push_pull_output(),
    };

    let mut running_system = RunningSystem {
        wheel_0,
    };

    running_system.wheel_0.set_rotate_direction(WheelRotateDirection::ClockWise);

    loop {
        running_system.wheel_0.step();
        delay.delay_ms(10u32);
    }
}
