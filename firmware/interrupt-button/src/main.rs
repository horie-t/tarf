#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal as wio;

use wio::hal::clock::GenericClockController;
use wio::hal::gpio::{Port, Pa15, Input, Floating, Output, PushPull};
use wio::pac::{interrupt, CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::{button_interrupt, entry, ButtonController, ButtonEvent, Pins, Sets};

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};

use heapless::consts::U8;
use heapless::spsc::Queue;

struct Led {
    pin: Pa15<Output<PushPull>>,
}

impl Led {
    fn new(pin: Pa15<Input<Floating>>, port: &mut Port) -> Led {
        Led {
            pin: pin.into_push_pull_output(port),
        }
    }

    pub fn set_high(&mut self) {
        self.pin.set_high().unwrap();
    }

    pub fn set_low(&mut self) {
        self.pin.set_low().unwrap();
    }

    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}


#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = Pins::new(peripherals.PORT);
    let mut sets: Sets = pins.split();

    let button_ctrlr = sets.buttons.init(
        peripherals.EIC,
        &mut clocks,
        &mut peripherals.MCLK,
        &mut sets.port,
    );
    let nvic = &mut core.NVIC;
    disable_interrupts(|_| unsafe {
        button_ctrlr.enable(nvic);
        BUTTON_CTRLR = Some(button_ctrlr);
    });

    let mut led = Led::new(sets.user_led, &mut sets.port);

    let mut consumer = unsafe { Q.split().1 };
    loop {
        if let Some(press) = consumer.dequeue() {
            if press.down {
                led.set_high();
            } else {
                led.set_low();
            }
        }
    }

}

static mut BUTTON_CTRLR: Option<ButtonController> = None;
static mut Q: Queue<ButtonEvent, U8> = Queue(heapless::i::Queue::new());

button_interrupt!(
    BUTTON_CTRLR,
    unsafe fn on_button_event(_cs: &CriticalSection, event: ButtonEvent) {
        let mut q = Q.split().0;
        q.enqueue(event).ok();
    }
);
