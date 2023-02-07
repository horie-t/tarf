#![no_std]
#![no_main]

use bno055::BNO055Interrupt;
use panic_halt as _;
use core::fmt::Write;
use embedded_hal::blocking::delay::DelayMs;
use cortex_m::peripheral::NVIC;
use cortex_m::interrupt::free as disable_interrupts;

use heapless::String;
use heapless::Vec;
use heapless::consts::*;
use heapless::spsc::Queue;

use xca9548a::{I2cSlave, SlaveAddr, Xca9548a};

use bno055;
use bno055::Bno055;

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::{ExtInt4, ExtInt5, ExtInt6, ExtInt7, ExtInt12, ExtInt13, ExtInt14, Sense};
use wio::hal::delay::Delay;
use wio::hal::eic::ConfigurableEIC;
use wio::hal::gpio;
use wio::hal::gpio::v1::{Port, Pa4, Pa6, Pa16, Pa17, Pa21, Pb7, Pb12, Pb13, Pb14, PfD};
use wio::hal::gpio::v2::{Alternate, D, Floating, Input, Interrupt, PA16, PA17};
use wio::hal::sercom::v2::{Pad0, Pad1};
use wio::hal::sercom::Pad;
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::pac::{SERCOM3, interrupt};
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::{Pins, UART, entry};

pub struct SensorEvent {
    pub c: f32
}

pub type SensorI2C = I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>;

pub struct NineAxisSensors<'a> {
    pub sensor0_gpio1: ExtInt5<Pa21<Interrupt<Floating>>>,
    pub sensor0_i2c: Bno055<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
}

impl<'a> NineAxisSensors<'a> {
    pub fn new(
        sensor0_gpio1: Pa21<Input<Floating>>,
        mut sensor0_i2c: Bno055<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
        eic: &mut ConfigurableEIC,
        port: &mut Port,
    ) -> NineAxisSensors<'a> {
        let mut sensor0_gpio1 = sensor0_gpio1.into_floating_ei(port);
        sensor0_gpio1.sense(eic, Sense::RISE);
        sensor0_gpio1.enable_interrupt(eic);
        //sensor0_i2c.start_continuous(0).unwrap();

        NineAxisSensors {
            sensor0_gpio1,
            sensor0_i2c,
        }
    }

    pub fn enable(&self, nvic: &mut NVIC, interrupt0: interrupt) {
        unsafe {
            nvic.set_priority(interrupt0, 1);
            NVIC::unmask(interrupt0);
        }
    }
}

#[macro_export]
macro_rules! nine_axis_interrupt {
    ($Sensors:ident, $Pin:ident, $I2c:ident, $SensorId:expr, $Interrupt:ident) => {
        #[interrupt]
        fn $Interrupt() {
            disable_interrupts(|_cs| unsafe {
                let sensors = $Sensors.as_mut().unwrap();
                if sensors.$Pin.is_interrupt() {
                    sensors.$I2c.clear_interrupts().unwrap();
                    sensors.$Pin.clear_interrupt();

                    let mut q = EVENT_QUEUE.split().0;
                    if let Some(angles) = sensors.$I2c.euler_angles().ok() {
                        q.enqueue(SensorEvent {
                            c: angles.c
                        }).ok();
                    }
                }
            });
        }
    };
}

static mut I2C_SWITCH: Option<Xca9548a<SensorI2C>> = None;
static mut NINE_AXI_SENSORS: Option<NineAxisSensors> = None;
static mut EVENT_QUEUE: Queue<SensorEvent, U16> = Queue(heapless::i::Queue::new());

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
    let gclk1 = clocks.gclk1();

    let mut configurable_eic = eic::init_with_ulp32k(&mut peripherals.MCLK, clocks.eic(&gclk1).unwrap(), peripherals.EIC);
    let nvic = &mut core.NVIC;

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

    let i2c_switch = Xca9548a::new(i2c, SlaveAddr::default());
    unsafe {
        I2C_SWITCH = Some(i2c_switch);
        let i2c_ports = I2C_SWITCH.as_mut().unwrap().split();

        let mut imu = Bno055::new(i2c_ports.i2c6).with_alternative_address();
        imu.init(&mut delay).unwrap();
        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
        imu.set_interrupts_enabled(BNO055Interrupt::ACC_AM).unwrap();
        
        let nine_axis_sensors = NineAxisSensors::new(
            pins.i2s_sdin, 
            imu,
            &mut configurable_eic,
            &mut pins.port
        );
        disable_interrupts(|_| {
            nine_axis_sensors.enable(nvic, interrupt::EIC_EXTINT_5);
        });
        NINE_AXI_SENSORS = Some(nine_axis_sensors);
    }
    nine_axis_interrupt!(NINE_AXI_SENSORS, sensor0_gpio1, sensor0_i2c, 0u16, EIC_EXTINT_5);
    let mut consumer = unsafe { EVENT_QUEUE.split().1 };

    delay.delay_ms(1000u32);
    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

    // let mut imu = bno055::Bno055::new(i2c_ports.i2c6).with_alternative_address();
    // imu.init(&mut delay).unwrap();
    writeln!(&mut serial, "BNO055 intialized.\r").unwrap();

    // imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
    writeln!(&mut serial, "BNO055 NDOF Mode.\r").unwrap();

    loop {
        // センサ情報を取得
        if let Some(interrupt_event) = consumer.dequeue() {
            writeln!(&mut serial, "c: {}\r", interrupt_event.c).unwrap();
        } else {
            writeln!(&mut serial, "No data.\r").unwrap();
        }
        delay.delay_ms(2000u32);
    }
}
