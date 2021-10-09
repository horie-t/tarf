#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal as wio;

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;


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

    let mut pins = wio::Pins::new(peripherals.PORT);

    let mut dir1 = pins.a0_d0.into_push_pull_output(&mut pins.port);
    dir1.set_high().unwrap();
    let mut stepper_motor1 = pins.a1_d1.into_push_pull_output(&mut pins.port);

    let mut dir2 = pins.a2_d2.into_push_pull_output(&mut pins.port);
    dir2.set_high().unwrap();
    let mut stepper_motor2 = pins.a3_d3.into_push_pull_output(&mut pins.port);

    let mut dir3 = pins.a4_d4.into_push_pull_output(&mut pins.port);
    dir3.set_high().unwrap();
    let mut stepper_motor3 = pins.a5_d5.into_push_pull_output(&mut pins.port);

    loop {
        delay.delay_ms(1u16);
        stepper_motor1.toggle();
        stepper_motor2.toggle();
        stepper_motor3.toggle();
    }
}
