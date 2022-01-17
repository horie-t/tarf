#![no_std]
#![no_main]

use bitfield::bitfield;
use core::fmt::Write;

use heapless::String;

use panic_halt as _;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Rectangle, PrimitiveStyle};
use embedded_graphics::text::Text;

use nalgebra as na;
use na::{Vector3, matrix, vector};

use wio_terminal as wio;
use wio::{entry, Display, LCD, Pins};
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;

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

fn clear_display(display: &mut LCD) {
    // 背景を黒にする
    let fill = PrimitiveStyle::with_fill(Rgb565::BLACK);
    display
        .bounding_box()
        .into_styled(fill)
        .draw(display).unwrap();

    // 文字を表示
    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(
        "Hello, Tarf!",
        Point::new(10, 20),
        character_style)
    .draw(display).unwrap();
}

fn println_display(display: &mut LCD, text: &str) {
    let fill = PrimitiveStyle::with_fill(Rgb565::BLACK);
    Rectangle::new(Point::new(10, 21), Size::new(320, 21))
    .into_styled(fill)
    .draw(display).unwrap();

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(
        text,
        Point::new(10, 40),
        character_style)
    .draw(display).unwrap();
}


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

    // ルートの初期化(探索してないけどマッピングは終了していることにする)
    // 左下を原点とする。
    let links = [
        (vector![0.0_f32, 0.0_f32], vector![0.0_f32, 2.0_f32]),
        (vector![0.0_f32, 2.0_f32], vector![2.0_f32, 2.0_f32]),
        (vector![2.0_f32, 2.0_f32], vector![2.0_f32, 0.0_f32]),
        (vector![2.0_f32, 0.0_f32], vector![1.0_f32, 0.0_f32]),
        (vector![1.0_f32, 0.0_f32], vector![1.0_f32, 1.0_f32])
        ];

    loop {

    }
}
