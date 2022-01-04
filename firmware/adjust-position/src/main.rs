#![no_std]
#![no_main]

use core::f32::consts::{PI, FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};
use core::fmt::Write;

use cortex_m::interrupt::free as disable_interrupts;
use cortex_m::peripheral::NVIC;
use heapless::String;
use heapless::consts::{U8, U16, U40};
use heapless::spsc::Queue;
use micromath::F32Ext;
use vl53l0x::VL53L0x;
use xca9548a::{I2cSlave, SlaveAddr, Xca9548a};

use panic_halt as _;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Rectangle, PrimitiveStyle};
use embedded_graphics::text::Text;

use nalgebra as na;
use na::{Matrix3, Vector3, matrix, vector, Vector2};
use na::base::SVector;

use wio::hal::eic::ConfigurableEIC;
use wio_terminal as wio;
use wio::{entry, Display, Pins};
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
 * オムニホイール系
 */
#[derive(Clone, Copy)]
enum WheelRotateDirection {
    ClockWise,
    CounterClockWise,
}

struct Wheel<S: PinId, D: PinId, T: Count16> {
    id: u32,
    direction: WheelRotateDirection,
    step_pin: Pin<S, Output<PushPull>>,
    direction_pin: Pin<D, Output<PushPull>>,
    timer_counter: TimerCounter<T>,
    radius: f32,
}

impl<S: PinId, D: PinId, T: Count16> Wheel<S, D, T> {
    const RADIUS: f32 = 24.0_f32;

    fn new(id: u32, step_pin: Pin<S, Input<Floating>>, direction_pin: Pin<D, Input<Floating>>,
        timer_counter: TimerCounter<T>, interrupt: interrupt) -> Wheel<S, D, T>{
            let mut wheel = Wheel {
                id,
                direction: WheelRotateDirection::ClockWise,
                step_pin: step_pin.into_push_pull_output(),
                direction_pin: direction_pin.into_push_pull_output(),
                timer_counter,
                radius: Self::RADIUS,
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
        self.direction = dir;
        match dir {
            WheelRotateDirection::ClockWise => {
                self.direction_pin.set_high().unwrap();
            },
            WheelRotateDirection::CounterClockWise => {
                self.direction_pin.set_low().unwrap();
            }
        }
    }

    fn start_with_rps(&mut self, rotate_per_sec: f32) {
        if rotate_per_sec < 0_f32 {
            self.direction = WheelRotateDirection::CounterClockWise;
            self.set_rotate_direction(WheelRotateDirection::CounterClockWise)
        } else {
            self.direction = WheelRotateDirection::ClockWise;
            self.set_rotate_direction(WheelRotateDirection::ClockWise)
        }
        let pulse_hz = ((2_f32 * 200_f32 * rotate_per_sec.abs()) as u32).hz();
        self.timer_counter.start(pulse_hz);
        self.timer_counter.enable_interrupt();
    }

    fn start_with_speed(&mut self, speed_mm_per_sec: f32) {
        if -0.1_f32< speed_mm_per_sec && speed_mm_per_sec < 0.1 {
            self.stop();
        } else {
            self.start_with_rps(speed_mm_per_sec / (2.0_f32 * PI * self.radius));
        }
    }

    fn stop(&mut self) {
        self.timer_counter.disable_interrupt();
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

                let mut q = WHEEL_MOVED_EVENT_QUEUE.split().0;
                let distance = {
                    match running_system.$Wheel.direction {
                        WheelRotateDirection::ClockWise => {
                            (2.0_f32 * PI * running_system.$Wheel.radius) / (2.0_f32 * 200.0_f32)
                        },
                        WheelRotateDirection::CounterClockWise => {
                            (- 2.0_f32 * PI * running_system.$Wheel.radius) / (2.0_f32 * 200.0_f32)
                        }
                    }
                };
                q.enqueue(WheelMovedEvent {
                    id: running_system.$Wheel.id,
                    distance: distance
                }).ok();
            });
        }
    };
}

struct WheelMovedEvent {
    id: u32,
    distance: f32,
}

/*
 * 走行装置系
 */
struct RunningEvent {

}

struct RunningSystem<S0: PinId, D0: PinId, T0: Count16, 
        S1: PinId, D1: PinId, T1: Count16, S2: PinId, D2: PinId, T2: Count16> {
    wheel_0: Wheel<S0, D0, T0>,
    wheel_1: Wheel<S1, D1, T1>,
    wheel_2: Wheel<S2, D2, T2>,
    mat_for_wheel_v: Matrix3<f32>,
    mat_for_odometry: Matrix3<f32>,
    velocity: Vector3<f32>,
    target_point: Vector3<f32>,
    trip_vec: Vector3<f32>,
}

impl <S0: PinId, D0: PinId, T0: Count16, S1: PinId, D1: PinId, T1: Count16, S2: PinId, D2: PinId, T2: Count16> RunningSystem<S0, D0, T0, S1, D1, T1, S2, D2, T2> {
    /// * `v` - 移動ベクトル。ロボット座標系。zは回転(rad/sec)を表す。
    fn run(&mut self, v: Vector3<f32>) {
        self.velocity = v;
        let mut wheels_v = self.mat_for_wheel_v * v;

        // 車輪の1つが進行方向と同じ向きに回転していると、滑りが発生せずその車輪だけ進み過ぎてしまうのを抑制する。
        if v.x * v.x + v.y * v.y > 0.0_f32 || v.x != 0.0_f32 {
            // 超信地旋回ではない、もしくはy軸方向への移動ではない場合

            // 進行方向が第2, 3象限の場合は、180度回転させた方向で判定しても同じ結果になる
            let direction_rad = 
                if v.x > 0.0_f32 {
                    (v.y / v.x).atan()
                } else {
                    (- v.y / v.x).atan()
                };
            
            if (direction_rad + FRAC_PI_3).abs() < FRAC_PI_8 {
                wheels_v[0] = wheels_v[0] * (1.0_f32 - 0.070 * (FRAC_PI_8 - (direction_rad + FRAC_PI_3).abs()) / FRAC_PI_8);
            } else if direction_rad.abs() < FRAC_PI_8 {
                wheels_v[1] = wheels_v[1] * (1.0_f32 - 0.070 * (FRAC_PI_8 - direction_rad.abs()              ) / FRAC_PI_8);
            } else if (direction_rad - FRAC_PI_3).abs() < FRAC_PI_8 {
                wheels_v[2] = wheels_v[2] * (1.0_f32 - 0.070 * (FRAC_PI_8 - (direction_rad - FRAC_PI_3).abs()) / FRAC_PI_8);
            }
        }

        self.wheel_0.start_with_speed(wheels_v[0]);
        self.wheel_1.start_with_speed(wheels_v[1]);
        self.wheel_2.start_with_speed(wheels_v[2]);
    }

    fn stop(&mut self) {
        self.wheel_0.stop();
        self.wheel_1.stop();
        self.wheel_2.stop();
    }

    fn move_to(&mut self, target_point: Vector3<f32>) {
        self.target_point = target_point;
        self.trip_vec = Vector3::zeros();

        let rotate_v = if target_point.z > 0.0_f32 {
            0.50_f32
        } else if target_point.z < 0.0_f32 {
            -0.50_f32
        } else {
            0.0_f32
        };

        let v = if target_point.xy().eq(&Vector2::<f32>::zeros()) {
            vector![0.0_f32, 0.0_f32, rotate_v]
        } else {
            let trans_normalized = target_point.xy().normalize();
            vector![25.0_f32 * trans_normalized.x, 25.0_f32 * trans_normalized.y, rotate_v]
        };

        self.run(v);
    }

    fn on_moved(&mut self, moved_event: WheelMovedEvent) -> bool {
        self.trip_vec += self.wheel_step_to_vec(moved_event);
        let distance = (self.target_point.xy() - self.trip_vec.xy()).magnitude();
        let diff_angle = (self.target_point.z - self.trip_vec.z).abs();
        if distance < 1.0_f32 && diff_angle < (PI / 90.0_f32) {
            // 目的地に到着
            self.stop();
            return true
        } else if distance < 1.0_f32 {
            self.run(vector![0.0_f32, 0.0_f32, self.velocity.z]);
        } else if diff_angle < (PI / 90.0_f32) {
            self.run(vector![self.velocity.x, self.velocity.y, 0.0_f32]);
        }

        false
    }

    fn wheel_step_to_vec(&self, moved_event: WheelMovedEvent) -> Vector3<f32> {
        let mut array = [0.0f32, 0.0f32, 0.0f32];
        array[moved_event.id as usize] = moved_event.distance;
        self.mat_for_odometry * SVector::from(array)
    }
}

static mut WHEEL_MOVED_EVENT_QUEUE: Queue<WheelMovedEvent, U16> = Queue(heapless::i::Queue::new());
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

type SensorI2C = I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>;

struct TofSensors<'a> {
    sensor0_gpio1: ExtInt4<Pa4<Interrupt<Floating>>>,
    sensor0_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    sensor1_gpio1: ExtInt7<Pb7<Interrupt<Floating>>>,
    sensor1_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    sensor2_gpio1: ExtInt6<Pa6<Interrupt<Floating>>>,
    sensor2_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    sensor3_gpio1: ExtInt14<Pb14<Interrupt<Floating>>>,
    sensor3_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    sensor4_gpio1: ExtInt12<Pb12<Interrupt<Floating>>>,
    sensor4_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    sensor5_gpio1: ExtInt13<Pb13<Interrupt<Floating>>>,
    sensor5_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
}

impl<'a> TofSensors<'a> {
    pub fn new(
        sensor0_gpio1: Pa4<Input<Floating>>,
        mut sensor0_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor1_gpio1: Pb7<Input<Floating>>,
        mut sensor1_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor2_gpio1: Pa6<Input<Floating>>,
        mut sensor2_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor3_gpio1: Pb14<Input<Floating>>,
        mut sensor3_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor4_gpio1: Pb12<Input<Floating>>,
        mut sensor4_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor5_gpio1: Pb13<Input<Floating>>,
        mut sensor5_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
        eic: &mut ConfigurableEIC,
        port: &mut Port,
        nvic: &mut NVIC,
    ) -> TofSensors<'a> {
        let mut sensor0_gpio1 = sensor0_gpio1.into_floating_ei(port);
        sensor0_gpio1.sense(eic, Sense::FALL);
        sensor0_gpio1.enable_interrupt(eic);
        sensor0_i2c.start_continuous(0).unwrap();

        let mut sensor1_gpio1 = sensor1_gpio1.into_floating_ei(port);
        sensor1_gpio1.sense(eic, Sense::FALL);
        sensor1_gpio1.enable_interrupt(eic);
        sensor1_i2c.start_continuous(0).unwrap();

        let mut sensor2_gpio1 = sensor2_gpio1.into_floating_ei(port);
        sensor2_gpio1.sense(eic, Sense::FALL);
        sensor2_gpio1.enable_interrupt(eic);
        sensor2_i2c.start_continuous(0).unwrap();

        let mut sensor3_gpio1 = sensor3_gpio1.into_floating_ei(port);
        sensor3_gpio1.sense(eic, Sense::FALL);
        sensor3_gpio1.enable_interrupt(eic);
        sensor3_i2c.start_continuous(0).unwrap();

        let mut sensor4_gpio1 = sensor4_gpio1.into_floating_ei(port);
        sensor4_gpio1.sense(eic, Sense::FALL);
        sensor4_gpio1.enable_interrupt(eic);
        sensor4_i2c.start_continuous(0).unwrap();

        let mut sensor5_gpio1 = sensor5_gpio1.into_floating_ei(port);
        sensor5_gpio1.sense(eic, Sense::FALL);
        sensor5_gpio1.enable_interrupt(eic);
        sensor5_i2c.start_continuous(0).unwrap();

        TofSensors {
            sensor0_gpio1,
            sensor0_i2c,
            sensor1_gpio1,
            sensor1_i2c, 
            sensor2_gpio1,
            sensor2_i2c,
            sensor3_gpio1,
            sensor3_i2c,
            sensor4_gpio1,
            sensor4_i2c,
            sensor5_gpio1,
            sensor5_i2c,
        }
    }

    pub fn enable(&self, nvic: &mut NVIC, interrupt0: interrupt, interrupt1: interrupt, interrupt2: interrupt,
            interrupt3: interrupt, interrupt4: interrupt, interrupt5: interrupt) {
        unsafe {
            nvic.set_priority(interrupt0, 1);
            NVIC::unmask(interrupt0);
            nvic.set_priority(interrupt1, 1);
            NVIC::unmask(interrupt1);
            nvic.set_priority(interrupt2, 1);
            NVIC::unmask(interrupt2);
            nvic.set_priority(interrupt3, 1);
            NVIC::unmask(interrupt3);
            nvic.set_priority(interrupt4, 1);
            NVIC::unmask(interrupt4);
            nvic.set_priority(interrupt5, 1);
            NVIC::unmask(interrupt5);
        }
    }
}

macro_rules! tof_interrupt {
    ($Sensors:ident, $Pin:ident, $I2c:ident, $SensorId:expr, $Interrupt:ident) => {
        #[interrupt]
        fn $Interrupt() {
            disable_interrupts(|_cs| unsafe {
                let sensors = $Sensors.as_mut().unwrap();
                if sensors.$Pin.is_interrupt() {
                    sensors.$Pin.clear_interrupt();

                    let mut q = EVENT_QUEUE.split().0;
                    q.enqueue(SensorEvent {
                        id: $SensorId,
                        distance: sensors.$I2c.read_range_mm().unwrap()
                    }).ok();
                }
            });
        }
    };
}

static mut I2C_SWITCH: Option<Xca9548a<SensorI2C>> = None;
static mut TOF_SENSORS: Option<TofSensors> = None;
static mut EVENT_QUEUE: Queue<SensorEvent, U16> = Queue(heapless::i::Queue::new());

enum VehicleState {
    Idle,
    PosSet,
    Path1,
    Arrive,
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
    let gclk1 = clocks.gclk1();
    let gclk5 = clocks.get_gclk(wio::pac::gclk::pchctrl::GEN_A::GCLK5).unwrap();
    let tc2_tc3 = clocks.tc2_tc3(&gclk5).unwrap();
    let tc4_tc5 = clocks.tc4_tc5(&gclk5).unwrap();

    let mut configurable_eic = eic::init_with_ulp32k(&mut peripherals.MCLK, clocks.eic(&gclk1).unwrap(), peripherals.EIC);
    let nvic = &mut core.NVIC;

    let mut pins = Pins::new(peripherals.PORT);
    
    // 走行装置の初期化
    let wheel_mat = matrix![
        0.5_f32, - 3.0_f32.sqrt() / 2.0_f32, 49.0_f32;
      - 1.0_f32,   0.0_f32                 , 49.0_f32;
        0.5_f32,   3.0_f32.sqrt() / 2.0_f32, 49.0_f32;
    ];
    let running_system = RunningSystem {
        wheel_0: Wheel::new(0, pins.a0_d0.into(), pins.a1_d1.into(),
            TimerCounter::tc2_(&tc2_tc3, peripherals.TC2, &mut peripherals.MCLK), interrupt::TC2),
        wheel_1: Wheel::new(1, pins.a2_d2.into(), pins.a3_d3.into(),
            TimerCounter::tc3_(&tc2_tc3, peripherals.TC3, &mut peripherals.MCLK), interrupt::TC3),
        wheel_2: Wheel::new(2, pins.a4_d4.into(), pins.a5_d5.into(),
            TimerCounter::tc4_(&tc4_tc5, peripherals.TC4, &mut peripherals.MCLK), interrupt::TC4),
        mat_for_wheel_v: wheel_mat,
        mat_for_odometry: wheel_mat.try_inverse().unwrap(),
        velocity: Vector3::zeros(),
        target_point: Vector3::repeat(f32::MAX),
        trip_vec: Vector3::zeros(),
    };

    unsafe { RUNNING_SYSTEM = Some(running_system); }
    wheel_interrupt!(RUNNING_SYSTEM, TC2, wheel_0);
    wheel_interrupt!(RUNNING_SYSTEM, TC3, wheel_1);
    wheel_interrupt!(RUNNING_SYSTEM, TC4, wheel_2);

    let mut wheel_event_queue = unsafe { WHEEL_MOVED_EVENT_QUEUE.split().1 };

    // スタートボタンの初期化
    let start_button = Button::new(
        pins.button1,
        Queue::new(),
        &mut configurable_eic,
        &mut pins.port,
    );
    disable_interrupts(|_| {
        start_button.enable(nvic, interrupt::EIC_EXTINT_10);
    });
    unsafe { START_BUTTON = Some(start_button); }
    button_interrupt!(START_BUTTON, EIC_EXTINT_10);
    let mut start_event_queue = unsafe { START_BUTTON.as_mut().unwrap().queue.split().1 };

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
        let tof_sensors = TofSensors::new(
            pins.a6_d6,
            VL53L0x::new(i2c_ports.i2c0).unwrap(),
            pins.a7_d7,
            VL53L0x::new(i2c_ports.i2c1).unwrap(),
            pins.a8_d8,
            VL53L0x::new(i2c_ports.i2c2).unwrap(),
            pins.gpclk0,
            VL53L0x::new(i2c_ports.i2c3).unwrap(),
            pins.gpclk1,
            VL53L0x::new(i2c_ports.i2c4).unwrap(),
            pins.gpclk2,
            VL53L0x::new(i2c_ports.i2c5).unwrap(),
            &mut configurable_eic,
            &mut pins.port,
            nvic,
        );
        disable_interrupts(|_| {
            tof_sensors.enable(nvic, interrupt::EIC_EXTINT_4, interrupt::EIC_EXTINT_7, interrupt::EIC_EXTINT_6,
                interrupt::EIC_EXTINT_14, interrupt::EIC_EXTINT_12, interrupt::EIC_EXTINT_13);
        });
        TOF_SENSORS = Some(tof_sensors);
    }
    tof_interrupt!(TOF_SENSORS, sensor0_gpio1, sensor0_i2c, 0u16, EIC_EXTINT_4);
    tof_interrupt!(TOF_SENSORS, sensor1_gpio1, sensor1_i2c, 1u16, EIC_EXTINT_7);
    tof_interrupt!(TOF_SENSORS, sensor2_gpio1, sensor2_i2c, 2u16, EIC_EXTINT_6);
    tof_interrupt!(TOF_SENSORS, sensor3_gpio1, sensor3_i2c, 3u16, EIC_EXTINT_14);
    tof_interrupt!(TOF_SENSORS, sensor4_gpio1, sensor4_i2c, 4u16, EIC_EXTINT_12);
    tof_interrupt!(TOF_SENSORS, sensor5_gpio1, sensor5_i2c, 5u16, EIC_EXTINT_13);
    let mut consumer = unsafe { EVENT_QUEUE.split().1 };
    let mut distances = [0_f32; 6];
    let calibration_values = [17.2_f32, 5.9_f32, 4.1_f32, 4.2_f32, 7.13_f32, 22.6_f32];

    // 初期化の後処理
    configurable_eic.finalize();

    let mut vehicle_state = VehicleState::Idle;
    let mut position = Vector3::<f32>::zeros();
    let velocity = 50.0_f32;
    let direction_rad = PI;

    // ホイールのステップ回数
    let mut step_count = 1;

    loop {
        // センサ情報を取得
        if let Some(interrupt_event) = consumer.dequeue() {
            let id = interrupt_event.id as usize;
            let calibrated_distance = interrupt_event.distance as f32 - calibration_values[id];
            distances[id] = 0.5_f32 * calibrated_distance + 0.5_f32 * distances[id];
        }

        // 走行制御
        match vehicle_state {
            VehicleState::Idle => {
                if let Some(event) = start_event_queue.dequeue() {
                    let len = 111.0_f32 + distances[3] + distances[0];
                    let diff_beside = distances[3] - distances[0];

                    let diff_back_front = distances[0] - distances[5];
                    let rotate = if diff_back_front > 0.0_f32 { 2.0_f32 * PI / 180.0_f32} else { - 2.0_f32 * PI / 180.0_f32};

                    let move_len = (diff_beside * 168.0_f32 / len) / 2.0_f32;
                    let mut text: String<U40> = String::new();
                    write!(text, "{}, ", diff_back_front).unwrap();
    
                    Rectangle::new(Point::new(10, 21), Size::new(320, 21))
                    .into_styled(fill)
                    .draw(&mut display).unwrap();
            
                    Text::new(
                        text.as_str(),
                        Point::new(10, 40),
                        character_style)
                    .draw(&mut display).unwrap();

                    if event.pressed {
                        unsafe {
                            let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                            running_system.move_to(vector![0.0_f32, 0.0_f32, rotate]);
                        }
                        vehicle_state = VehicleState::PosSet;
                    }
                }
            },
            VehicleState::PosSet => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    step_count += 1;
                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        if running_system.on_moved(moved) {
                            let diff_back_front = distances[0] - distances[5];
                            if diff_back_front.abs() < 1.0_f32 {
                                vehicle_state = VehicleState::Arrive;
                            } else {
                                let rotate = if diff_back_front > 0.0_f32 { 2.0_f32 * PI / 180.0_f32} else { - 2.0_f32 * PI / 180.0_f32};
                                running_system.move_to(vector![0.0_f32, 0.0_f32, rotate]);
                            }
                        }
                    }
                }
            },
            VehicleState::Path1 => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    step_count += 1;
                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        position += running_system.wheel_step_to_vec(moved);

                        // 時々、方向を調整する。
                        if step_count % 128 == 0 {
                            let beside_diff = distances[2] - distances[0];
                            let rotate_diff = distances[5] - distances[0];
                            running_system.run(vector![velocity * direction_rad.cos(),
                                velocity * direction_rad.sin() + 0.2_f32 * beside_diff,
                                0.02_f32 * rotate_diff]);
                        }
                    }

                    // 壁が近づいたら到着
                    if distances[1] < 30.0_f32 {
                        unsafe {
                            let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                            running_system.stop();
                        }
                        vehicle_state = VehicleState::Arrive;
                    }
                }
            },
            VehicleState::Arrive => {
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

                position = Vector3::zeros();
                vehicle_state = VehicleState::Idle;
            }
        }
    }
}
