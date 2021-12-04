#![no_std]
#![no_main]

use panic_halt as _;
use core::fmt::Write;
use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;
use heapless::consts::U16;
use heapless::spsc::Queue;
use vl53l0x::VL53L0x;

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::*;
use wio::hal::gpio::*;
use wio::hal::gpio::v1::Pin;
use wio::hal::gpio::v2::{Alternate, D, PA16, PA17};
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::hal::sercom::Pad;
use wio::hal::sercom::v2::{Pad0, Pad1};
use wio::pac::{CorePeripherals, EIC, MCLK, Peripherals, SERCOM3, interrupt};
use wio::prelude::*;
use wio::{Pins, UART, entry};


struct InterruptEvent {}

struct InterruptPins {
    /// interrupt pin
    pub pin: Pa4<Input<Floating>>,
}

impl InterruptPins {
    pub fn init(
        self,
        eic: EIC,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> InterruptController {
        let clk = clocks.gclk1();
        let mut eic = eic::init_with_ulp32k(mclk, clocks.eic(&clk).unwrap(), eic);
        eic.button_debounce_pins(&[
            self.pin.id(),
        ]);

        let mut p = self.pin.into_floating_ei(port);
        p.sense(&mut eic, Sense::FALL);
        p.enable_interrupt(&mut eic);
        InterruptController {
            _eic: eic.finalize(),
            p,
        }
    }
}

struct InterruptController {
    _eic: eic::EIC,
    p: ExtInt4<Pa4<Interrupt<Floating>>>,
}

macro_rules! isr {
    ($Handler:ident, $($Event:expr, $Pin:ident),+) => {
        pub fn $Handler(&mut self) -> Option<InterruptEvent> {
            $(
                {
                    let p = &mut self.$Pin;
                    if p.is_interrupt() {
                       p.clear_interrupt();
                        return Some(InterruptEvent {
                        })
                    }
                }
            )+

            None
        }
    };
}

impl InterruptController {
    pub fn enable(&self, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::EIC_EXTINT_4, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_4);
        }
    }

    isr!(interrupt_extint4, InterruptEvent, p);
}

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
    let gclk0 = &clocks.gclk0();

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

    let i2c: I2CMaster3<
        Sercom3Pad0<Pa17<PfD>>,
        Sercom3Pad1<Pa16<PfD>>
    > = I2CMaster3::new(
        &clocks.sercom3_core(&gclk0).unwrap(),
        400.khz(),
        peripherals.SERCOM3,
        &mut peripherals.MCLK,
        pins.i2c1_sda.into_pad(&mut pins.port),
        pins.i2c1_scl.into_pad(&mut pins.port)
    );

    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

    delay.delay_ms(2000u32);
    let mut tof_sensor: VL53L0x<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>> = VL53L0x::new(i2c).unwrap();
    tof_sensor.start_continuous(0).unwrap();
    writeln!(&mut serial, "VL53L0x intialized.\r").unwrap();

    let interrupt_pins = InterruptPins {
        pin: pins.a6_d6
    };
    let interrupt_controller = interrupt_pins.init(
        peripherals.EIC,
        &mut clocks,
        &mut peripherals.MCLK,
        &mut pins.port,
    );
    let nvic = &mut core.NVIC;
    disable_interrupts(|_| unsafe {
        interrupt_controller.enable(nvic);
        INTERRUPT_CTRLR = Some(interrupt_controller);
    });
    writeln!(&mut serial, "set up interrupt.\r").unwrap();

    let mut consumer = unsafe { EVENT_QUEUE.split().1 };

    loop {
        if let Some(_interrupt_event) = consumer.dequeue() {
            match tof_sensor.read_range_mm() {
                Ok(meas) => {
                    write!(&mut serial, "vl: millis {}\r\n", meas).unwrap();
                }
                Err(e) => {
                    write!(&mut serial, "Err meas: {:?}\r\n", e).unwrap();
                }
            }
        }
    }
}

static mut INTERRUPT_CTRLR: Option<InterruptController> = None;
static mut EVENT_QUEUE: Queue<InterruptEvent, U16> = Queue(heapless::i::Queue::new());

macro_rules! ext_interrupt {
    ($controller:ident, unsafe fn $func_name:ident ($cs:ident: $cstype:ty, $event:ident: InterruptEvent) $code:block) => {
        unsafe fn $func_name($cs: $cstype, $event: InterruptEvent) {
            $code
        }

        macro_rules! _ext_interrupt_handler {
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

        _ext_interrupt_handler!(EIC_EXTINT_4, interrupt_extint4);
    };
}

ext_interrupt!(
    INTERRUPT_CTRLR,
    unsafe fn on_interrupt_event(_cs: &CriticalSection, event: InterruptEvent) {
        let mut q = EVENT_QUEUE.split().0;
        q.enqueue(event).ok();
    }
);
