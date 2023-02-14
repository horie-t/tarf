#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;

use heapless::consts::*;
use heapless::spsc::Queue;

use bno055;
use bno055::Bno055;

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::gpio::*;
use wio::hal::gpio::v1::{Port, Pa4, Pa6, Pa16, Pa17, Pa21, Pb7, Pb12, Pb13, Pb14, PfD};
use wio::hal::gpio::v2::{Alternate, D, Floating, Input, Interrupt, PA16, PA17};
use wio::hal::sercom::v2::{Pad0, Pad1};
use wio::hal::sercom::Pad;
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::hal::timer::TimerCounter;
use wio::pac::{SERCOM3, CorePeripherals, interrupt, Peripherals, TC5};
use wio::prelude::*;
use wio::{entry, Pins, UART};

use xca9548a::{I2cSlave, SlaveAddr, Xca9548a};


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

type SensorI2C = I2CMaster3<Pad<SERCOM3, Pad0, v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, v1::Pin<PA16, Alternate<D>>>>;

struct NineAxisSensor<'a> {
    led: Led,
    sensor: Bno055<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    tc5: TimerCounter<TC5>,
}
static mut CTX: Option<NineAxisSensor> = None;

pub struct SensorEvent {
    pub x: f32
}

static mut I2C_SWITCH: Option<Xca9548a<SensorI2C>> = None;
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
    let gclk0 = &clocks.gclk0();
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
        let i2c_switch = Xca9548a::new(i2c, SlaveAddr::default());
        I2C_SWITCH = Some(i2c_switch);
        let i2c_ports = I2C_SWITCH.as_mut().unwrap().split();

        let mut imu = Bno055::new(i2c_ports.i2c6).with_alternative_address();
        imu.init(&mut delay).unwrap();
        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();

        CTX = Some(NineAxisSensor {
            led: Led::new(pins.user_led, &mut pins.port),
            sensor: imu,
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