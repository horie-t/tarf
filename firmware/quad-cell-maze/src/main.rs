#![no_std]
#![no_main]

use core::f32::consts::{PI, FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};
use core::fmt::Write;

use cortex_m::interrupt::free as disable_interrupts;
use heapless::String;
use heapless::consts::*;
use heapless::spsc::Queue;
use micromath::F32Ext;
use vl53l0x::VL53L0x;
use xca9548a::{SlaveAddr, Xca9548a};

use panic_halt as _;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Rectangle, PrimitiveStyle};
use embedded_graphics::text::Text;

use nalgebra as na;
use na::{Vector3, matrix, vector};

use wio_terminal as wio;
use wio::{entry, Display, Pins};
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::common::eic;
use wio::hal::gpio::v1::{Pa16, Pa17, PfD};
use wio::hal::gpio::v2::{PA07, PB04, PB05, PB06, PB08, PB09};
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::hal::timer::TimerCounter;
use wio::pac::{CorePeripherals, Peripherals, TC2, TC3, TC4, interrupt};
use wio::prelude::*;

mod console;
use console::{Button, ButtonEvent};

mod runningsystem;
use runningsystem::{RunningSystem, Wheel, WheelMovedEvent, WheelRotateDirection};

mod sensor;
use sensor::{SensorEvent, SensorI2C, TofSensors};

static mut WHEEL_MOVED_EVENT_QUEUE: Queue<WheelMovedEvent, U16> = Queue(heapless::i::Queue::new());
static mut RUNNING_SYSTEM: Option<RunningSystem<PB08, PB09, TC2, PA07, PB04, TC3, PB05, PB06, TC4>> = None;

static mut START_BUTTON: Option<Button> = None;
static mut I2C_SWITCH: Option<Xca9548a<SensorI2C>> = None;
static mut TOF_SENSORS: Option<TofSensors> = None;
static mut EVENT_QUEUE: Queue<SensorEvent, U16> = Queue(heapless::i::Queue::new());

enum VehicleState {
    Idle,
    AdjustDir,
    AdjustCenter,
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
    let velocity = 50.0_f32;
    let direction_rad = PI;

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
                    let diff_back_front = distances[0] - distances[5];
                    let rotate = if diff_back_front > 0.0_f32 { 2.0_f32 * PI / 180.0_f32} else { - 2.0_f32 * PI / 180.0_f32};

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
                        vehicle_state = VehicleState::AdjustDir;
                    }
                }
            },
            VehicleState::AdjustDir => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        if running_system.on_moved(moved) {
                            let diff_back_front = distances[0] - distances[5];
                            if diff_back_front.abs() < 1.0_f32 {
                                let len = 111.0_f32 + distances[3] + distances[0];
                                let diff_beside = distances[3] - distances[0];
            
                                let move_len = (diff_beside * 168.0_f32 / len) / 2.0_f32;
                                running_system.move_to(vector![0.0_f32, move_len, 0.0_f32]);
                                vehicle_state = VehicleState::AdjustCenter;
                            } else {
                                let rotate = if diff_back_front > 0.0_f32 { 2.0_f32 * PI / 180.0_f32} else { - 2.0_f32 * PI / 180.0_f32};
                                running_system.move_to(vector![0.0_f32, 0.0_f32, rotate]);
                            }
                        }
                    }
                }
            },
            VehicleState::AdjustCenter => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        if running_system.on_moved(moved) {
                            let diff_beside = distances[3] - distances[0];
                
                            if diff_beside.abs() < 1.5_f32 {
                                running_system.run(vector![velocity * direction_rad.cos(), velocity * direction_rad.sin(), 0.0_f32]);
                                vehicle_state = VehicleState::Path1;
                            } else {
                                let move_len = if diff_beside > 0.0_f32 { 0.5_f32 } else { - 0.5_f32 };
                                running_system.move_to(vector![0.0_f32, move_len, 0.0_f32]);
                            }
                        }
                    }
                }
            },
            VehicleState::Path1 => {
                if let Some(_moved) = wheel_event_queue.dequeue() {
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

                vehicle_state = VehicleState::Idle;
            }
        }
    }
}
