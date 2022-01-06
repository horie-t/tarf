use cortex_m::peripheral::NVIC;

use heapless::consts::*;
use heapless::spsc::Queue;

use wio_terminal as wio;
use wio::hal::common::eic::pin::{ExtInt10, ExternalInterrupt, Sense};
use wio::hal::eic::ConfigurableEIC;
use wio::hal::gpio::v1::{Port, Pc26};
use wio::hal::gpio::v2::{Floating, Input, Interrupt};
use wio::pac::interrupt;
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

