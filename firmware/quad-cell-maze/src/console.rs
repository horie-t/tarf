use cortex_m::peripheral::NVIC;

use heapless::consts::{U8, U16, U40};
use heapless::spsc::Queue;

use wio_terminal as wio;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::{ExtInt4, ExtInt6, ExtInt7, ExtInt10, ExtInt12, ExtInt13, ExtInt14, ExternalInterrupt, Sense};
use wio::hal::eic::ConfigurableEIC;
use wio::hal::gpio::v1::{Port, Pa4, Pa16, Pa17, Pa6, Pb7, Pb12, Pb13, Pb14, Pc26, PfD};
use wio::hal::gpio::v2::{Alternate, D, Floating, Input, Interrupt, Output, PA07, PA16, PA17, PB04, PB05, PB06, PB08, PB09, Pin, PinId, PushPull};
use wio::pac::{CorePeripherals, Peripherals, SERCOM3, TC2, TC3, TC4, interrupt};
use wio::prelude::*;


/* 
 * スタート・ボタン系
 */
pub struct Button {
    pub queue: Queue<ButtonEvent, U8>,
    pub pin: ExtInt10<Pc26<Interrupt<Floating>>>,
}

impl Button {
    pub fn new(
        pin: Pc26<Input<Floating>>,
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

pub struct ButtonEvent {
    pub pressed: bool,
}

#[macro_export]
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

