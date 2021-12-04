#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;
use heapless::consts::U16;
use heapless::spsc::Queue;
use panic_halt as _;
use vl53l0x::VL53L0x;
use xca9548a::{I2cSlave, SlaveAddr, Xca9548a};

use wio::hal::clock::GenericClockController;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::*;
use wio::hal::delay::Delay;
use wio::hal::gpio::v1::Pin;
use wio::hal::gpio::v2::{Alternate, D, PA16, PA17};
use wio::hal::gpio::*;
use wio::hal::sercom::v2::{Pad0, Pad1};
use wio::hal::sercom::Pad;
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::pac::{interrupt, CorePeripherals, Peripherals, EIC, MCLK, SERCOM3};
use wio::prelude::*;
use wio::{entry, Pins, UART};
use wio_terminal as wio;

struct InterruptEvent {
    distance: u16,
}

struct InterruptPins {
    /// interrupt pin
    pub pin: Pa4<Input<Floating>>,
    // pub tof_sensor: VL53L0x<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
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
        eic.button_debounce_pins(&[self.pin.id()]);

        let mut p = self.pin.into_floating_ei(port);
        p.sense(&mut eic, Sense::FALL);
        p.enable_interrupt(&mut eic);
        InterruptController {
            _eic: eic.finalize(),
            p,
            // tof_sensor: self.tof_sensor
        }
    }
}

struct InterruptController {
    _eic: eic::EIC,
    p: ExtInt4<Pa4<Interrupt<Floating>>>,
    // tof_sensor: VL53L0x<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
}

macro_rules! isr {
    ($Handler:ident, $($Event:expr, $Pin:ident),+) => {
        pub fn $Handler(&mut self) -> Option<InterruptEvent> {
            $(
                {
                    let p = &mut self.$Pin;
                    if p.is_interrupt() {
                        p.clear_interrupt();
                        let distance = unsafe {
                            let tof_sensor = TOF.as_mut().unwrap();
                            tof_sensor.read_range_mm().unwrap()
                        };
                        return Some(InterruptEvent {
                            distance
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
        rx: pins.rxd,
    };
    let mut serial = uart.init(
        &mut clocks,
        115200.hz(),
        peripherals.SERCOM2,
        &mut peripherals.MCLK,
        &mut pins.port,
    );

    unsafe {
        let i2c: I2CMaster3<Sercom3Pad0<Pa17<PfD>>, Sercom3Pad1<Pa16<PfD>>> = I2CMaster3::new(
            &clocks.sercom3_core(&gclk0).unwrap(),
            400.khz(),
            peripherals.SERCOM3,
            &mut peripherals.MCLK,
            pins.i2c1_sda.into_pad(&mut pins.port),
            pins.i2c1_scl.into_pad(&mut pins.port),
        );
        let i2c_switch = Xca9548a::new(i2c, SlaveAddr::default());
        I2C_SWITCH = Some(i2c_switch);
        let i2c_ports = I2C_SWITCH.as_mut().unwrap().split();

        writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

        delay.delay_ms(2000u32);
        let mut tof_sensor = VL53L0x::new(i2c_ports.i2c0).unwrap();
        tof_sensor.start_continuous(0).unwrap();
        TOF = Some(tof_sensor);
    }
    writeln!(&mut serial, "VL53L0x intialized.\r").unwrap();

    let interrupt_pins = InterruptPins {
        pin: pins.a6_d6,
        // tof_sensor
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
        if let Some(interrupt_event) = consumer.dequeue() {
            writeln!(&mut serial, "Distance: {}.\r", interrupt_event.distance).unwrap();
        }
    }
}

static mut I2C_SWITCH: Option<Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>> = None;
static mut TOF: Option<
    VL53L0x<
        I2cSlave<
            Xca9548a<
                I2CMaster3<
                    Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,
                    Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>,
                >,
            >,
            I2CMaster3<
                Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,
                Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>,
            >,
        >,
    >,
> = None;
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
