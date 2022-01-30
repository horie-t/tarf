#![no_std]
#![no_main]

use bitfield::bitfield;
use core::f32::consts::{PI, FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};
use core::fmt::Write;
use core::iter::FromIterator;

use cortex_m::interrupt::free as disable_interrupts;
use heapless::String;
use heapless::Vec;
use heapless::consts::*;
use heapless::spsc::Queue;
use micromath::F32Ext;
use vl53l0x::VL53L0x;
use xca9548a::{SlaveAddr, Xca9548a};

use panic_halt as _;

use embedded_graphics::prelude::*;

use nalgebra as na;
use na::{Vector3, matrix, vector, Matrix3, Vector2};

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
use console::{Button, ButtonEvent, MapView, clear_display, println_display, print_current_cell, print_current_pose, SideWallView};

mod runningsystem;
use runningsystem::{RunningSystem, Wheel, WheelMovedEvent, WheelRotateDirection};

mod sensor;
use sensor::{SensorEvent, SensorI2C, TofSensors};

/*
 * 迷路関連
 */
bitfield! {
    /// 迷路の壁
    #[derive(Clone, Copy)]
    pub struct MazeCell(u8);
    pub north, set_north: 7, 6;
    pub east, set_east: 5, 4;
    pub south, set_south: 3, 2;
    pub west, set_west: 1, 0;
}

bitfield! {
    /// 側面の壁
    #[derive(Clone, Copy)]
    pub struct SideWall(u8);
    pub front_right, set_front_right: 7, 6;
    pub back_right, set_back_right: 5, 4;
    pub front_left, set_front_left: 3, 2;
    pub back_left, set_back_left: 1, 0;
}

const NO_WALL: u8 = 0;
const WALL: u8 = 1;
const UNKNOWN_WALL: u8 = 3;

const CELL_____: MazeCell = MazeCell(0b00000000);
const CELL_N___: MazeCell = MazeCell(0b01000000);
const CELL__E__: MazeCell = MazeCell(0b00010000);
const CELL_NE__: MazeCell = MazeCell(0b01010000);
const CELL___S_: MazeCell = MazeCell(0b00000100);
const CELL_N_S_: MazeCell = MazeCell(0b01000100);
const CELL__ES_: MazeCell = MazeCell(0b00010100);
const CELL_NES_: MazeCell = MazeCell(0b01010100);
const CELL____W: MazeCell = MazeCell(0b00000001);
const CELL_N__W: MazeCell = MazeCell(0b01000001);
const CELL__E_W: MazeCell = MazeCell(0b00010001);
const CELL_NE_W: MazeCell = MazeCell(0b01010001);
const CELL___SW: MazeCell = MazeCell(0b00000101);
const CELL_N_SW: MazeCell = MazeCell(0b01000101);
const CELL__ESW: MazeCell = MazeCell(0b00010101);
const CELL_NESW: MazeCell = MazeCell(0b01010101);

pub type Maze = Vec<Vec<MazeCell, U16>, U16>;

const MAZE_CELL_SIZE_MM: f32 = 180.0_f32;

pub fn get_maze_cell_in(pose: &Vector3<f32>) -> Vector2<i32> {
    vector![(pose.x / MAZE_CELL_SIZE_MM) as i32, (pose.y / MAZE_CELL_SIZE_MM) as i32]
}

pub fn get_fine_maze_cell_in(pose: &Vector3<f32>) -> Vector2<i32> {
    vector![((pose.x - MAZE_CELL_SIZE_MM / 4.0_f32) / (MAZE_CELL_SIZE_MM / 2.0_f32)) as i32,
        ((pose.y - MAZE_CELL_SIZE_MM / 4.0_f32) / (MAZE_CELL_SIZE_MM / 2.0_f32)) as i32]
}

pub fn calc_side_wall(maze: &Maze, pose: &Vector3<f32>) -> SideWall {
    let cell = get_fine_maze_cell_in(pose);

    if cell.x & 1 == 0 && cell.y & 1 == 0 {
        let maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2) as usize];
        let direction = pose.z + FRAC_PI_2;

        if FRAC_PI_4 < direction && direction <= FRAC_PI_4 * 3.0_f32 {
            // 北向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_right(maze_cell.east());
            side_wall.set_back_right(maze_cell.east());
            side_wall.set_front_left(maze_cell.west());
            side_wall.set_back_left(maze_cell.west());

            side_wall
        } else if -FRAC_PI_4 < direction && direction <= FRAC_PI_4 {
            // 東向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_left(maze_cell.north());
            side_wall.set_back_left(maze_cell.north());
            side_wall.set_front_right(maze_cell.south());
            side_wall.set_back_right(maze_cell.south());

            side_wall
        } else if FRAC_PI_4 * 3.0_f32 < direction && direction <= FRAC_PI_4 * 5.0_f32 {
            // 西向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_left(maze_cell.south());
            side_wall.set_back_left(maze_cell.south());
            side_wall.set_front_right(maze_cell.north());
            side_wall.set_back_right(maze_cell.north());

            side_wall
        } else {
            // 南向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_right(maze_cell.west());
            side_wall.set_back_right(maze_cell.west());
            side_wall.set_front_left(maze_cell.east());
            side_wall.set_back_left(maze_cell.east());

            side_wall
        }
    } else {
        SideWall(0xff_u8)
    }
}

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
    Run,
    Turn,
    AdjustTrun,
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
        - 3.0_f32.sqrt() / 2.0_f32, - 0.5_f32, 49.0_f32;
          0.0_f32,                    1.0_f32, 49.0_f32;
          3.0_f32.sqrt() / 2.0_f32, - 0.5_f32, 49.0_f32;
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
    clear_display(&mut display);

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
    let mut calibrated_distances = [0_f32; 6];
    let calibration_values = [17.2_f32, 5.9_f32, 4.1_f32, 4.2_f32, 7.13_f32, 22.6_f32];

    // 迷路の初期化(探索してないけどマッピングは終了していることにする)
    // 定義は左上を起点にしている
    let mut maze = Maze::from_iter(
        //探索・走行時は、迷路は左下を起点(maze[0][0])にして扱いたいので、revする。
        [
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL_N__W, CELL_N___, CELL_NE__, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL__E_W, CELL_NE_W, CELL__E_W, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
            Vec::from_iter([CELL__ESW, CELL___SW, CELL__ES_, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____, CELL_____].iter().cloned()),
        ]
        .iter().rev().cloned()
    );

    // ルートの初期化
    // 左下を原点とする。
    let route = [
        (vector![0.0_f32, 0.0_f32], vector![0.0_f32, 2.0_f32]),
        (vector![0.0_f32, 2.0_f32], vector![2.0_f32, 2.0_f32]),
        (vector![2.0_f32, 2.0_f32], vector![2.0_f32, 0.0_f32]),
        (vector![2.0_f32, 0.0_f32], vector![1.0_f32, 0.0_f32]),
        (vector![1.0_f32, 0.0_f32], vector![1.0_f32, 1.0_f32])
        ];
    // スタート時点のリンクは0番目
    let mut link_index = 0;

    // 地図のViewの初期化
    let map_view = MapView {
        top_left: Point::new(150, 50),
        size: Size::new(161, 161)
    };
    map_view.draw_maze(&mut display, &mut maze);
    map_view.draw_route(&mut display, &route);

    let side_wall_view = SideWallView {
        top_left: Point::new(20, 80),
        size: Size::new(11, 11)
    };

    // 初期化の後処理
    configurable_eic.finalize();

    let mut vehicle_pose = vector![90.0_f32, 90.0_f32, 0.0_f32];
    let mut vehicle_state = VehicleState::Idle;
    let velocity = 50.0_f32;

    let mut moved_count = 0_u64;
    let mut drawed_moved_count = 0_u64; // FIXME: きれいな実装ではない。
    map_view.draw_vehicle(&mut display, &vehicle_pose);

    println_display(&mut display, "Initialiezed");

    loop {
        // センサ情報を取得
        if let Some(interrupt_event) = consumer.dequeue() {
            let id = interrupt_event.id as usize;
            let calibrated_distance = interrupt_event.distance as f32 - calibration_values[id];
            distances[id] = 0.5_f32 * interrupt_event.distance as f32 + 0.5_f32 * distances[id];
            calibrated_distances[id] = 0.5_f32 * calibrated_distance + 0.5_f32 * calibrated_distances[id];
        }

        // 走行制御
        match vehicle_state {
            VehicleState::Idle => {
                if let Some(event) = start_event_queue.dequeue() {
                    let diff_back_front = distances[0] - distances[5];
                    let rotate = if diff_back_front > 0.0_f32 { 2.0_f32 * PI / 180.0_f32} else { - 2.0_f32 * PI / 180.0_f32};

                    let mut text: String<U40> = String::new();
                    write!(text, "{}, ", diff_back_front).unwrap();
                    println_display(&mut display, text.as_str());

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
                    moved_count += 1;

                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                        vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);

                        if running_system.on_moved(&moved) {
                            let diff_back_front = distances[0] - distances[5];
                            if diff_back_front.abs() < 1.0_f32 {
                                let len = 111.0_f32 + distances[3] + distances[0];
                                let diff_beside = distances[3] - distances[0];
            
                                let move_len = (diff_beside * 168.0_f32 / len) / 2.0_f32;
                                running_system.move_to(vector![move_len, 0.0_f32, 0.0_f32]);
                                vehicle_state = VehicleState::AdjustCenter;
                            } else {
                                let rotate = if diff_back_front > 0.0_f32 { PI / 180.0_f32} else { - PI / 180.0_f32};
                                running_system.move_to(vector![0.0_f32, 0.0_f32, rotate]);
                            }
                        }
                    }
                }
            },
            VehicleState::AdjustCenter => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    moved_count += 1;

                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                        vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);

                        if running_system.on_moved(&moved) {
                            let diff_beside = distances[3] - distances[0];
                
                            if diff_beside.abs() < 1.5_f32 {
                                running_system.run(vector![0.0_f32, velocity, 0.0_f32]);
                                vehicle_state = VehicleState::Run;
                            } else {
                                let move_len = if diff_beside > 0.0_f32 { 0.5_f32 } else { - 0.5_f32 };
                                running_system.move_to(vector![move_len, 0.0_f32, 0.0_f32]);
                            }
                        }
                    }
                }
            },
            VehicleState::Run => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    moved_count += 1;

                    if distances[1] < 40.0_f32 {
                        // 壁が近づいたら
                        if link_index == route.len() - 1 {
                            // ゴールに到着
                            unsafe {
                                let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                                let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                                vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);
                                running_system.stop();
                            }
                            vehicle_state = VehicleState::Arrive;
                        } else {
                            // 次のリンクを走行開始
                            let current_link = route[link_index];
                            let current_link_vec = (current_link.1 - current_link.0).normalize();
                            let next_link = route[link_index + 1];
                            let next_link_vec = (next_link.1 - next_link.0).normalize();
                            let dot = current_link_vec.dotc(&next_link_vec);
                            let det = current_link_vec.perp(&next_link_vec);
                            let rad = det.atan2(dot);

                            let mut text: String<U40> = String::new();
                            write!(text, "Turn {}, ", rad).unwrap();
                            println_display(&mut display, text.as_str());
        
                            unsafe {
                                let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                                let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                                vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);
                                running_system.move_to(vector![0.0_f32, 0.0_f32, rad]);
                            }

                            vehicle_state = VehicleState::Turn;
                        }
                    } else if moved_count % (3 * 64) == 0 {
                        let bias = (distances[0] + distances[5] - 120.0_f32) / 2.0_f32;
                        let rotate_diff = distances[5] - distances[0];
                        unsafe {
                            let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                            let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                            vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);
                            running_system.run(vector![- bias, velocity, - 0.02_f32 * rotate_diff]);
                        }
                    } else {
                        unsafe {
                            let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                            let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                            vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);
                        }
                    }
                }
            },
            VehicleState::Turn => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    moved_count += 1;

                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                        vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);

                        if running_system.on_moved(&moved) {
                            println_display(&mut display, "Turned.");
                            let diff_back_front = distances[0] - distances[5];
                            let rotate = if diff_back_front > 0.0_f32 { PI / 180.0_f32} else { - PI / 180.0_f32};
                            running_system.move_to(vector![0.0_f32, 0.0_f32, rotate]);
                            vehicle_state = VehicleState::AdjustTrun;
                        }
                    }
                }
            },
            VehicleState::AdjustTrun => {
                if let Some(moved) = wheel_event_queue.dequeue() {
                    moved_count += 1;

                    unsafe {
                        let running_system = RUNNING_SYSTEM.as_mut().unwrap();
                        let vehicle_rotate = Matrix3::new_rotation(vehicle_pose.z);
                        vehicle_pose += vehicle_rotate * running_system.wheel_step_to_vec(&moved);
                        if running_system.on_moved(&moved) {
                            println_display(&mut display, "Turn Adjusted.");
                            running_system.run(vector![0.0_f32, velocity, 0.0_f32]);
                            link_index += 1;
                            let current_link = route[link_index];
                            let current_link_vec = (current_link.1 - current_link.0).normalize();
                            vehicle_pose.x = current_link.0.x * 180.0_f32 + 90.0_f32;
                            vehicle_pose.y = current_link.0.y * 180.0_f32 + 90.0_f32;
                            vehicle_pose.z = current_link_vec.y.atan2(current_link_vec.x) - FRAC_PI_2;
                            vehicle_state = VehicleState::Run;
                        }
                    }
                }
            },
            VehicleState::Arrive => {
                let mut text: String<U40> = String::new();
                write!(text, "Arrive!").unwrap();
                println_display(&mut display, text.as_str());

                link_index = 0;
                vehicle_state = VehicleState::Idle;
            }
        }

        // 画面を更新
        if moved_count % 0x80 == 0 && drawed_moved_count != moved_count {
            map_view.clear_maze(&mut display);
            map_view.draw_maze(&mut display, &mut maze);
            map_view.draw_route(&mut display, &route);
            map_view.draw_vehicle(&mut display, &vehicle_pose);

            // print_current_cell(&mut display, &vehicle_pose.xy());
            print_current_pose(&mut display, &vehicle_pose);
            let vehicle_fine_cell = get_fine_maze_cell_in(&vehicle_pose);
            let mut text: String<U40> = String::new();
            write!(text, "({}, {})", vehicle_fine_cell.x, vehicle_fine_cell.y).unwrap();
            println_display(&mut display, text.as_str());

            side_wall_view.draw_wall(&mut display, &calc_side_wall(&maze, &vehicle_pose));

            drawed_moved_count = moved_count;
        }
    }
}
