#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;

use heapless::consts::*;
use heapless::spsc::Queue;

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::gpio::*;
use wio::hal::timer::TimerCounter;
use wio::pac::{CorePeripherals, interrupt, Peripherals, TC5};
use wio::prelude::*;
use wio::{entry, Pins, UART};



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

pub struct SensorEvent {
    pub x: f32
}
static mut EVENT_QUEUE: Queue<SensorEvent, U16> = Queue(heapless::i::Queue::new());

#[entry]
fn main() -> ! {
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

    let mut pins = Pins::new(peripherals.PORT);

    let uart = UART {
        tx: pins.txd,
        rx: pins.rxd
    };
    let mut serial = uart.init(
        &mut clocks,
        115200.hz(),
        peripherals.SERCOM2,
        &mut peripherals.MCLK,
        &mut pins.port
    );

    delay.delay_ms(1000u32);

    let gclk5 = clocks
        .get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5)
        .unwrap();
    let timer_clock5 = clocks.tc4_tc5(&gclk5).unwrap();
    let mut tc5 = TimerCounter::tc5_(&timer_clock5, peripherals.TC5, &mut peripherals.MCLK);

    unsafe {
        NVIC::unmask(interrupt::TC5);
    }

    tc5.start(3.s());
    tc5.enable_interrupt();

    writeln!(&mut serial, "Hello, Tarf!\r").unwrap();

    unsafe {
        CTX = Some(Ctx {
            led: Led::new(pins.user_led, &mut pins.port),
            tc5,
        });
    }

    loop {
        if let Some(val) = disable_interrupts(|cs| unsafe { EVENT_QUEUE.split().1.dequeue()}) {
            writeln!(&mut serial, "val: {}\r", val.x).unwrap();
        }
    }
}


#[interrupt]
fn TC5() {
    unsafe {
        disable_interrupts(|cs| unsafe {
            let ctx = CTX.as_mut().unwrap();
            ctx.tc5.wait().unwrap();
            ctx.led.toggle();
            let mut q = EVENT_QUEUE.split().0;
            q.enqueue(SensorEvent { x: 16.0_f32 }).ok();
        });
    }
}