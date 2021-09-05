#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal as wio;

use wio::hal::clock::GenericClockController;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::*;
use wio::hal::common::gpio::*;
use wio::pac::{interrupt, EIC, MCLK, CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::{entry, Button, ButtonEvent, Pins, Sets};

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;

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

struct ButtonPins {
    /// button3 pin
    pub button3: Pc28<Input<Floating>>,
}

impl ButtonPins {
    pub fn init(
        self,
        eic: EIC,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> ButtonController {
        let clk = clocks.gclk1();
        let mut eic = eic::init_with_ulp32k(mclk, clocks.eic(&clk).unwrap(), eic);
        eic.button_debounce_pins(&[
            self.button3.id(),

        ]);

        let mut b3 = self.button3.into_floating_ei(port);
        b3.sense(&mut eic, Sense::BOTH);
        b3.enable_interrupt(&mut eic);
        ButtonController {
            _eic: eic.finalize(),
            b3,
        }
    }
}

pub struct ButtonController {
    _eic: eic::EIC,
    b3: ExtInt12<Pc28<Interrupt<Floating>>>,
}

macro_rules! isr {
    ($Handler:ident, $($Event:expr, $Button:ident),+) => {
        pub fn $Handler(&mut self) -> Option<ButtonEvent> {
            $(
                {
                    let b = &mut self.$Button;
                    if b.is_interrupt() {
                        b.clear_interrupt();
                        return Some(ButtonEvent {
                            button: $Event,
                            down: !b.state(),
                        })
                    }
                }
            )+

            None
        }
    };
}

impl ButtonController {
    pub fn enable(&self, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::EIC_EXTINT_12, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_12);
        }
    }

    isr!(interrupt_extint12, Button::TopLeft, b3);
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

    let buttons = ButtonPins {
        button3: sets.buttons.button3
    };
    let button_ctrlr = buttons.init(
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
static mut Q: Queue<ButtonEvent, 8> = Queue::new();

macro_rules! button_interrupt {
    ($controller:ident, unsafe fn $func_name:ident ($cs:ident: $cstype:ty, $event:ident: ButtonEvent ) $code:block) => {
        unsafe fn $func_name($cs: $cstype, $event: ButtonEvent) {
            $code
        }

        macro_rules! _button_interrupt_handler {
            ($Interrupt:ident, $Handler:ident) => {
                #[interrupt]
                fn $Interrupt() {
                    disable_interrupts(|cs| unsafe {
                        $controller.as_mut().map(|ctrlr| {
                            if let Some(event) = ctrlr.$Handler() {
                                $func_name(cs, event);
                            }
                        });
                    });
                }
            };
        }

        _button_interrupt_handler!(EIC_EXTINT_12, interrupt_extint12);
    };
}

button_interrupt!(
    BUTTON_CTRLR,
    unsafe fn on_button_event(_cs: &CriticalSection, event: ButtonEvent) {
        let mut q = Q.split().0;
        q.enqueue(event).ok();
    }
);
