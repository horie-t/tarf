#![no_std]
#![no_main]

use bitfield::bitfield;
use core::fmt::Write;
use core::iter::FromIterator;

use heapless::String;
use heapless::Vec;
use heapless::consts::*;

use panic_halt as _;

use embedded_graphics::prelude::*;

use nalgebra as na;
use na::{Vector2, Vector3, matrix, vector};

use wio_terminal as wio;
use wio::{entry, Display, LCD, Pins};
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;

mod console;
use console::{MapView, clear_display, println_display};

/*
 * 迷路関連
 */
bitfield! {
    #[derive(Clone, Copy)]
    pub struct MazeCell(u8);
    pub north, set_north: 7, 6;
    pub east, set_east: 5, 4;
    pub south, set_south: 3, 2;
    pub west, set_west: 1, 0;
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

type Maze = Vec<Vec<MazeCell, U16>, U16>;

#[entry]
fn main() -> ! {
    // 初期化処理
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut pins = Pins::new(peripherals.PORT);

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

    clear_display(&mut display);

    println_display(&mut display, "Initialiezed");

    let mut vehicle_pose = vector![180.0_f32, 180.0_f32, 0.0_f32]; // ゴールに到達
        
    loop {
        let map_view = MapView {
            top_left: Point::new(80, 50),
            size: Size::new(161, 161)
        };

        map_view.draw(&mut display).ok();
        map_view.draw_maze(&mut display, &mut maze);
        map_view.draw_route(&mut display, &route);
        map_view.draw_vehicle(&mut display, &vehicle_pose);
    }
}
