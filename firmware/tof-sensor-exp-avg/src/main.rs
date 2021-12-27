#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;
use heapless::consts::*;
use heapless::spsc::Queue;
use heapless::String;
use panic_halt as _;
use vl53l0x::VL53L0x;
use xca9548a::{I2cSlave, SlaveAddr, Xca9548a};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyle},
    text::Text,
};

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
use wio::{entry, Display, Pins, UART};
use wio_terminal as wio;

struct SensorEvent {
    id: u16,
    distance: u16,
}

struct TofSensors<'a> {
    sensor1_gpio1: Pa4<Input<Floating>>,
    sensor1_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,

    sensor2_gpio1: Pb7<Input<Floating>>,
    sensor2_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,

    sensor3_gpio1: Pa6<Input<Floating>>,
    sensor3_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,

    sensor4_gpio1: Pb14<Input<Floating>>,
    sensor4_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,

    sensor5_gpio1: Pb12<Input<Floating>>,
    sensor5_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,

    sensor6_gpio1: Pb13<Input<Floating>>,
    sensor6_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
}

impl<'a> TofSensors<'a> {
    pub fn init(
        mut self,
        eic: EIC,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> TofSensorController<'a> {
        let clk = clocks.gclk1();
        let mut eic = eic::init_with_ulp32k(mclk, clocks.eic(&clk).unwrap(), eic);

        let mut sensor1_gpio1 = self.sensor1_gpio1.into_floating_ei(port);
        sensor1_gpio1.sense(&mut eic, Sense::FALL);
        sensor1_gpio1.enable_interrupt(&mut eic);
        self.sensor1_i2c.start_continuous(0).unwrap();

        let mut sensor2_gpio1 = self.sensor2_gpio1.into_floating_ei(port);
        sensor2_gpio1.sense(&mut eic, Sense::FALL);
        sensor2_gpio1.enable_interrupt(&mut eic);
        self.sensor2_i2c.start_continuous(0).unwrap();

        let mut sensor3_gpio1 = self.sensor3_gpio1.into_floating_ei(port);
        sensor3_gpio1.sense(&mut eic, Sense::FALL);
        sensor3_gpio1.enable_interrupt(&mut eic);
        self.sensor3_i2c.start_continuous(0).unwrap();

        let mut sensor4_gpio1 = self.sensor4_gpio1.into_floating_ei(port);
        sensor4_gpio1.sense(&mut eic, Sense::FALL);
        sensor4_gpio1.enable_interrupt(&mut eic);
        self.sensor4_i2c.start_continuous(0).unwrap();

        let mut sensor5_gpio1 = self.sensor5_gpio1.into_floating_ei(port);
        sensor5_gpio1.sense(&mut eic, Sense::FALL);
        sensor5_gpio1.enable_interrupt(&mut eic);
        self.sensor5_i2c.start_continuous(0).unwrap();

        let mut sensor6_gpio1 = self.sensor6_gpio1.into_floating_ei(port);
        sensor6_gpio1.sense(&mut eic, Sense::FALL);
        sensor6_gpio1.enable_interrupt(&mut eic);
        self.sensor6_i2c.start_continuous(0).unwrap();

        TofSensorController {
            _eic: eic.finalize(),
            sensor1_gpio1,
            sensor1_i2c: self.sensor1_i2c,
            sensor2_gpio1,
            sensor2_i2c: self.sensor2_i2c,
            sensor3_gpio1,
            sensor3_i2c: self.sensor3_i2c,
            sensor4_gpio1,
            sensor4_i2c: self.sensor4_i2c,
            sensor5_gpio1,
            sensor5_i2c: self.sensor5_i2c,
            sensor6_gpio1,
            sensor6_i2c: self.sensor6_i2c,
        }
    }
}

struct TofSensorController<'a> {
    _eic: eic::EIC,
    sensor1_gpio1: ExtInt4<Pa4<Interrupt<Floating>>>,
    sensor1_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
    sensor2_gpio1: ExtInt7<Pb7<Interrupt<Floating>>>,
    sensor2_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
    sensor3_gpio1: ExtInt6<Pa6<Interrupt<Floating>>>,
    sensor3_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
    sensor4_gpio1: ExtInt14<Pb14<Interrupt<Floating>>>,
    sensor4_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
    sensor5_gpio1: ExtInt12<Pb12<Interrupt<Floating>>>,
    sensor5_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
    sensor6_gpio1: ExtInt13<Pb13<Interrupt<Floating>>>,
    sensor6_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>>>>,
}

macro_rules! isr {
    ($Handler:ident, $($Event:expr, $Pin:ident, $I2c:ident, $SensorId:expr),+) => {
        pub fn $Handler(&mut self) -> Option<SensorEvent> {
            $(
                {
                    let p = &mut self.$Pin;
                    if p.is_interrupt() {
                        p.clear_interrupt();
                        return Some(SensorEvent {
                            id: $SensorId,
                            distance: self.$I2c.read_range_mm().unwrap()
                        })
                    }
                }
            )+

            None
        }
    };
}

impl<'a> TofSensorController<'a> {
    pub fn enable(&self, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::EIC_EXTINT_4, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_4);
            nvic.set_priority(interrupt::EIC_EXTINT_7, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_7);
            nvic.set_priority(interrupt::EIC_EXTINT_6, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_6);
            nvic.set_priority(interrupt::EIC_EXTINT_14, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_14);
            nvic.set_priority(interrupt::EIC_EXTINT_12, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_12);
            nvic.set_priority(interrupt::EIC_EXTINT_13, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_13);
        }
    }

    isr!(interrupt_extint4, SensorEvent, sensor1_gpio1, sensor1_i2c, 0u16);
    isr!(interrupt_extint7, SensorEvent, sensor2_gpio1, sensor2_i2c, 1u16);
    isr!(interrupt_extint6, SensorEvent, sensor3_gpio1, sensor3_i2c, 2u16);
    isr!(interrupt_extint14, SensorEvent, sensor4_gpio1, sensor4_i2c, 3u16);
    isr!(interrupt_extint12, SensorEvent, sensor5_gpio1, sensor5_i2c, 4u16);
    isr!(interrupt_extint13, SensorEvent, sensor6_gpio1, sensor6_i2c, 5u16);
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
    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();
    delay.delay_ms(1000u32);

    // LCDの初期化
    let (mut display, _blacklight) = (Display {
        miso: pins.lcd_miso,
        mosi: pins.lcd_mosi,
        sck: pins.lcd_sck,
        cs: pins.lcd_cs,
        dc: pins.lcd_dc,
        reset: pins.lcd_reset,
        backlight: pins.lcd_backlight
    }).init(&mut clocks, peripherals.SERCOM7, &mut peripherals.MCLK, &mut pins.port, 58.mhz(), &mut delay)
    .ok().unwrap();

    // 背景を黒にする
    let fill = PrimitiveStyle::with_fill(Rgb565::BLACK);
    display
        .bounding_box()
        .into_styled(fill)
        .draw(&mut display).unwrap();

    // 文字を表示
    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(
        "Hello, Tarf!",
        Point::new(10, 20),
        character_style)
    .draw(&mut display).unwrap();    

    let i2c: I2CMaster3<Sercom3Pad0<Pa17<PfD>>, Sercom3Pad1<Pa16<PfD>>> = I2CMaster3::new(
        &clocks.sercom3_core(&gclk0).unwrap(),
        400.khz(),
        peripherals.SERCOM3,
        &mut peripherals.MCLK,
        pins.i2c1_sda.into_pad(&mut pins.port),
        pins.i2c1_scl.into_pad(&mut pins.port),
    );
    let i2c_switch = Xca9548a::new(i2c, SlaveAddr::default());

    unsafe {
        I2C_SWITCH = Some(i2c_switch);
        let i2c_ports = I2C_SWITCH.as_mut().unwrap().split();

        let interrupt_pins = TofSensors {
            sensor1_gpio1: pins.a6_d6,
            sensor1_i2c: VL53L0x::new(i2c_ports.i2c0).unwrap(),
            sensor2_gpio1: pins.a7_d7,
            sensor2_i2c: VL53L0x::new(i2c_ports.i2c1).unwrap(),
            sensor3_gpio1: pins.a8_d8,
            sensor3_i2c: VL53L0x::new(i2c_ports.i2c2).unwrap(),
            sensor4_gpio1: pins.gpclk0,
            sensor4_i2c: VL53L0x::new(i2c_ports.i2c3).unwrap(),
            sensor5_gpio1: pins.gpclk1,
            sensor5_i2c: VL53L0x::new(i2c_ports.i2c4).unwrap(),
            sensor6_gpio1: pins.gpclk2,
            sensor6_i2c: VL53L0x::new(i2c_ports.i2c5).unwrap(),
        };
        let interrupt_controller = interrupt_pins.init(
            peripherals.EIC,
            &mut clocks,
            &mut peripherals.MCLK,
            &mut pins.port,
        );
        let nvic = &mut core.NVIC;
        disable_interrupts(|_| {
            interrupt_controller.enable(nvic);
            TOF_SENSOR_CTRLR = Some(interrupt_controller);
        });
    }
    writeln!(&mut serial, "VL53L0x intialized.\r").unwrap();

    let mut consumer = unsafe { EVENT_QUEUE.split().1 };

    let mut distances = [0_f32; 6];

    loop {
        if let Some(interrupt_event) = consumer.dequeue() {
            let id = interrupt_event.id as usize;
            distances[id] = 0.2_f32 * interrupt_event.distance as f32 + 0.8 * distances[id];

            let mut text: String<U40> = String::new();
            for distance in distances.iter() {
                write!(text, "{}, ", (*distance as i16)).unwrap();
            }

            Rectangle::new(Point::new(10, 21), Size::new(320, 21))
            .into_styled(fill)
            .draw(&mut display).unwrap();
    
            Text::new(
                text.as_str(),
                Point::new(10, 40),
                character_style)
            .draw(&mut display).unwrap();
    
            serial.write_str(text.as_str()).unwrap();
            write!(&mut serial, "\r\n").unwrap();            
        }
    }
}

static mut I2C_SWITCH: Option<
    Xca9548a<
        I2CMaster3<
            Pad<SERCOM3, Pad0, Pin<PA17, Alternate<D>>>,
            Pad<SERCOM3, Pad1, Pin<PA16, Alternate<D>>>,
        >,
    >,
> = None;
static mut TOF_SENSOR_CTRLR: Option<TofSensorController> = None;
static mut EVENT_QUEUE: Queue<SensorEvent, U16> = Queue(heapless::i::Queue::new());

macro_rules! ext_interrupt {
    ($controller:ident, unsafe fn $func_name:ident ($cs:ident: $cstype:ty, $event:ident: SensorEvent) $code:block) => {
        unsafe fn $func_name($cs: $cstype, $event: SensorEvent) {
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
        _ext_interrupt_handler!(EIC_EXTINT_7, interrupt_extint7);
        _ext_interrupt_handler!(EIC_EXTINT_6, interrupt_extint6);
        _ext_interrupt_handler!(EIC_EXTINT_14, interrupt_extint14);
        _ext_interrupt_handler!(EIC_EXTINT_12, interrupt_extint12);
        _ext_interrupt_handler!(EIC_EXTINT_13, interrupt_extint13);
    };
}

ext_interrupt!(
    TOF_SENSOR_CTRLR,
    unsafe fn on_interrupt_event(_cs: &CriticalSection, event: SensorEvent) {
        let mut q = EVENT_QUEUE.split().0;
        q.enqueue(event).ok();
    }
);
