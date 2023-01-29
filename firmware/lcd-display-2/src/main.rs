#![no_std]
#![no_main]

use panic_halt as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::PrimitiveStyle,
    text::{Alignment, Text},
};

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::{Pins, entry};


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
    let sets = Pins::new(peripherals.PORT).split();

    // LCDの初期化
    let (mut display, _blacklight) = sets.display
        .init(&mut clocks, peripherals.SERCOM7, &mut peripherals.MCLK, 58.mhz(), &mut delay).unwrap();

    // 背景を黒にする
    let fill = PrimitiveStyle::with_fill(Rgb565::BLACK);
    display
        .bounding_box()
        .into_styled(fill)
        .draw(&mut display).unwrap();

    // 文字を表示
    let text = "Hello, Tarf!";
    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::with_alignment(
        text,
        display.bounding_box().center() + Point::new(0, 15),
        character_style,
        Alignment::Center,
    )
    .draw(&mut display).unwrap();

    loop {
    }
}
