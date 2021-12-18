#![no_std]
#![no_main]

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;
use heapless::consts::{U8, U16};
use heapless::spsc::Queue;
use vl53l0x::VL53L0x;
use xca9548a::{I2cSlave, SlaveAddr, Xca9548a};

use panic_halt as _;

use wio::hal::eic::ConfigurableEIC;
use wio_terminal as wio;
use wio::{entry, Pins};
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::{ExtInt4, ExtInt6, ExtInt7, ExtInt10, ExtInt12, ExtInt13, ExtInt14, ExternalInterrupt, Sense};
use wio::hal::gpio;
use wio::hal::gpio::v1::{Port, Pa4, Pa16, Pa17, Pa6, Pb7, Pb12, Pb13, Pb14, Pc26, PfD};
use wio::hal::gpio::v2::{Alternate, D, Floating, Input, Interrupt, Output, PA07, PA16, PA17, PB04, PB05, PB06, PB08, PB09, Pin, PinId, PushPull};
use wio::hal::sercom::v2::{Pad0, Pad1};
use wio::hal::sercom::Pad;
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::hal::timer::{Count16, TimerCounter};
use wio::pac::{CorePeripherals, Peripherals, SERCOM3, TC2, TC3, TC4, interrupt};
use wio::prelude::*;

/* 
 * 走行装置系
 */
#[derive(Clone, Copy)]
enum WheelRotateDirection {
    ClockWise,
    CounterClockWise,
}

struct Wheel<S: PinId, D: PinId, T: Count16> {
    id: u32,
    step_pin: Pin<S, Output<PushPull>>,
    direction_pin: Pin<D, Output<PushPull>>,
    timer_counter: TimerCounter<T>,
}

impl<S: PinId, D: PinId, T: Count16> Wheel<S, D, T> {
    fn new(id: u32, step_pin: Pin<S, Input<Floating>>, direction_pin: Pin<D, Input<Floating>>,
        timer_counter: TimerCounter<T>, interrupt: interrupt) -> Wheel<S, D, T>{
            let mut wheel = Wheel {
                id,
                step_pin: step_pin.into_push_pull_output(),
                direction_pin: direction_pin.into_push_pull_output(),
                timer_counter
            };
            wheel.set_rotate_direction(WheelRotateDirection::ClockWise);

            unsafe {
                NVIC::unmask(interrupt);
            }

            wheel
    }

    fn step(&mut self) {
        self.step_pin.toggle().unwrap();
    }

    fn set_rotate_direction(&mut self, dir: WheelRotateDirection) {
        match dir {
            WheelRotateDirection::ClockWise => {
                self.direction_pin.set_high().unwrap();
            },
            WheelRotateDirection::CounterClockWise => {
                self.direction_pin.set_low().unwrap();
            }
        }
    }

    fn start(&mut self, rpm: f32) {
        let pulse_hz = (((2_f32 * 200_f32) / 60_f32 * rpm) as u32).hz();
        self.timer_counter.start(pulse_hz);
        self.timer_counter.enable_interrupt();
    }
}

macro_rules! wheel_interrupt {
    ($RunningSystem:ident, $Handler:ident, $Wheel:ident) => {
        #[interrupt]
        fn $Handler() {
            disable_interrupts(|_cs| unsafe {
                let running_system = $RunningSystem.as_mut().unwrap();
                running_system.$Wheel.step();
                running_system.$Wheel.timer_counter.wait().unwrap();
            });
        }
    };
}

struct RunningSystem<S0: PinId, D0: PinId, T0: Count16, S1: PinId, D1: PinId, T1: Count16, S2: PinId, D2: PinId, T2: Count16> {
    wheel_0: Wheel<S0, D0, T0>,
    wheel_1: Wheel<S1, D1, T1>,
    wheel_2: Wheel<S2, D2, T2>,
}

static mut RUNNING_SYSTEM: Option<RunningSystem<PB08, PB09, TC2, PA07, PB04, TC3, PB05, PB06, TC4>> = None;

/* 
 * スタート・ボタン系
 */
struct Button {
    queue: Queue<ButtonEvent, U8>,
    pin: ExtInt10<Pc26<Interrupt<Floating>>>,
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

struct ButtonEvent {
    pressed: bool,
}

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

static mut START_BUTTON: Option<Button> = None;

/* 
 * センサ系
 */
struct SensorEvent {
    id: u16,
    distance: u16,
}

struct TofSensors<'a> {
    sensor1_gpio1: Pa4<Input<Floating>>,
    sensor1_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,

    sensor2_gpio1: Pb7<Input<Floating>>,
    sensor2_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,

    sensor3_gpio1: Pa6<Input<Floating>>,
    sensor3_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,

    sensor4_gpio1: Pb14<Input<Floating>>,
    sensor4_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,

    sensor5_gpio1: Pb12<Input<Floating>>,
    sensor5_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,

    sensor6_gpio1: Pb13<Input<Floating>>,
    sensor6_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
}

impl<'a> TofSensors<'a> {
    pub fn init(
        mut self,
        eic: &mut ConfigurableEIC,
        port: &mut Port,
    ) -> TofSensorController<'a> {
        let mut sensor1_gpio1 = self.sensor1_gpio1.into_floating_ei(port);
        sensor1_gpio1.sense(eic, Sense::FALL);
        sensor1_gpio1.enable_interrupt(eic);
        self.sensor1_i2c.start_continuous(0).unwrap();

        let mut sensor2_gpio1 = self.sensor2_gpio1.into_floating_ei(port);
        sensor2_gpio1.sense(eic, Sense::FALL);
        sensor2_gpio1.enable_interrupt(eic);
        self.sensor2_i2c.start_continuous(0).unwrap();

        let mut sensor3_gpio1 = self.sensor3_gpio1.into_floating_ei(port);
        sensor3_gpio1.sense(eic, Sense::FALL);
        sensor3_gpio1.enable_interrupt(eic);
        self.sensor3_i2c.start_continuous(0).unwrap();

        let mut sensor4_gpio1 = self.sensor4_gpio1.into_floating_ei(port);
        sensor4_gpio1.sense(eic, Sense::FALL);
        sensor4_gpio1.enable_interrupt(eic);
        self.sensor4_i2c.start_continuous(0).unwrap();

        let mut sensor5_gpio1 = self.sensor5_gpio1.into_floating_ei(port);
        sensor5_gpio1.sense(eic, Sense::FALL);
        sensor5_gpio1.enable_interrupt(eic);
        self.sensor5_i2c.start_continuous(0).unwrap();

        let mut sensor6_gpio1 = self.sensor6_gpio1.into_floating_ei(port);
        sensor6_gpio1.sense(eic, Sense::FALL);
        sensor6_gpio1.enable_interrupt(eic);
        self.sensor6_i2c.start_continuous(0).unwrap();

        TofSensorController {
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
    sensor1_gpio1: ExtInt4<Pa4<Interrupt<Floating>>>,
    sensor1_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
    sensor2_gpio1: ExtInt7<Pb7<Interrupt<Floating>>>,
    sensor2_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
    sensor3_gpio1: ExtInt6<Pa6<Interrupt<Floating>>>,
    sensor3_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
    sensor4_gpio1: ExtInt14<Pb14<Interrupt<Floating>>>,
    sensor4_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
    sensor5_gpio1: ExtInt12<Pb12<Interrupt<Floating>>>,
    sensor5_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
    sensor6_gpio1: ExtInt13<Pb13<Interrupt<Floating>>>,
    sensor6_i2c: VL53L0x<I2cSlave<
                    'a, Xca9548a<I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>,
                    I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>>>,
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

    isr!(interrupt_extint4, SensorEvent, sensor1_gpio1, sensor1_i2c, 1u16);
    isr!(interrupt_extint7, SensorEvent, sensor2_gpio1, sensor2_i2c, 2u16);
    isr!(interrupt_extint6, SensorEvent, sensor3_gpio1, sensor3_i2c, 3u16);
    isr!(interrupt_extint14, SensorEvent, sensor4_gpio1, sensor4_i2c, 4u16);
    isr!(interrupt_extint12, SensorEvent, sensor5_gpio1, sensor5_i2c, 5u16);
    isr!(interrupt_extint13, SensorEvent, sensor6_gpio1, sensor6_i2c, 6u16);
}

static mut I2C_SWITCH: Option<
    Xca9548a<
        I2CMaster3<
            Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>,
            Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>,
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

    let gclk5 = clocks.get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5)
        .unwrap();
    let tc2_tc3 = clocks.tc2_tc3(&gclk5).unwrap();
    let tc4_tc5 = clocks.tc4_tc5(&gclk5).unwrap();

    let mut pins = Pins::new(peripherals.PORT);
    
    // 走行装置の初期化
    let mut running_system = RunningSystem {
        wheel_0: Wheel::new(0, pins.a0_d0.into(), pins.a1_d1.into(),
            TimerCounter::tc2_(&tc2_tc3, peripherals.TC2, &mut peripherals.MCLK), interrupt::TC2),
        wheel_1: Wheel::new(1, pins.a2_d2.into(), pins.a3_d3.into(),
            TimerCounter::tc3_(&tc2_tc3, peripherals.TC3, &mut peripherals.MCLK), interrupt::TC3),
        wheel_2: Wheel::new(2, pins.a4_d4.into(), pins.a5_d5.into(),
            TimerCounter::tc4_(&tc4_tc5, peripherals.TC4, &mut peripherals.MCLK), interrupt::TC4),
    };
    running_system.wheel_1.set_rotate_direction(WheelRotateDirection::CounterClockWise);

    unsafe {
        RUNNING_SYSTEM = Some(running_system);
        wheel_interrupt!(RUNNING_SYSTEM, TC2, wheel_0);
        wheel_interrupt!(RUNNING_SYSTEM, TC3, wheel_1);
        wheel_interrupt!(RUNNING_SYSTEM, TC4, wheel_2);
    }

    // スタートボタンの初期化
    let start_button = Button::new(
        pins.button1,
        Queue::new(),
        &mut configurable_eic,
        &mut pins.port,
    );
    let nvic = &mut core.NVIC;
    disable_interrupts(|_| unsafe {
        start_button.enable(nvic, interrupt::EIC_EXTINT_10);
        START_BUTTON = Some(start_button);
    });
    button_interrupt!(START_BUTTON, EIC_EXTINT_10);
    let mut consumer = unsafe { START_BUTTON.as_mut().unwrap().queue.split().1 };

    // センサの初期化
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
            &mut configurable_eic,
            &mut pins.port,
        );
        let nvic = &mut core.NVIC;
        disable_interrupts(|_| {
            interrupt_controller.enable(nvic);
            TOF_SENSOR_CTRLR = Some(interrupt_controller);
        });
    }

    // 初期化の後処理
    configurable_eic.finalize();

    loop {
        if let Some(event) = consumer.dequeue() {
            if event.pressed {
                unsafe {
                    let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                    running_system.wheel_0.start(10_f32);
                    running_system.wheel_1.start(20_f32);
                    running_system.wheel_2.start(10_f32);
                }
            }
        }
    }
}
