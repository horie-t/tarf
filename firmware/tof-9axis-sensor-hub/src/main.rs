#![no_std]
#![no_main]

use panic_halt as _;
use core::fmt::Write;
use embedded_hal::blocking::delay::DelayMs;
use heapless::String;
use heapless::consts::*;
use vl53l0x::VL53L0x;
use xca9548a::{SlaveAddr, Xca9548a};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyle},
    text::Text,
};

use bno055;

use wio_terminal as wio;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::gpio::*;
use wio::hal::sercom::{I2CMaster3, PadPin, Sercom3Pad0, Sercom3Pad1};
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::{Display, Pins, UART, entry};


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
    let gclk0 = &clocks.gclk0();

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
    writeln!(&mut serial, "Hello, {}!\r", "tarf").unwrap();

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

    let i2c_switch = Xca9548a::new(i2c, SlaveAddr::default());
    let i2c_ports = i2c_switch.split();

    let mut sensors = [
        VL53L0x::new(i2c_ports.i2c0).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c1).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c2).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c3).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c4).ok().unwrap(),
        VL53L0x::new(i2c_ports.i2c5).ok().unwrap(),
    ];
    writeln!(&mut serial, "VL53L0x intialized.\r").unwrap();
    delay.delay_ms(1000u32);

    let mut imu = bno055::Bno055::new(i2c_ports.i2c6).with_alternative_address();
    imu.init(&mut delay).unwrap();
    writeln!(&mut serial, "BNO055 intialized.\r").unwrap();

    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
    writeln!(&mut serial, "BNO055 NDOF Mode.\r").unwrap();

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

    loop {
        let mut text: String<U40> = String::new();
        let mut text2: String<U40> = String::new();
        for i in 0..sensors.len() {
            if let Some(distance) = sensors[i].read_range_single_millimeters_blocking().ok() {
                write!(text, "{}, ", distance).unwrap();
            }
        }
        Rectangle::new(Point::new(10, 21), Size::new(320, 42))
        .into_styled(fill)
        .draw(&mut display).unwrap();

        Text::new(
            text.as_str(),
            Point::new(10, 40),
            character_style)
        .draw(&mut display).unwrap();

        serial.write_str(text.as_str()).unwrap();
        write!(&mut serial, "\r\n").unwrap();

        if let Some(quat) = imu.quaternion().ok() {
//            write!(text2, "x: {}, y: {}, z: {}, s{}", quat.v.x, quat.v.y, quat.v.z, quat.s).unwrap();
            write!(text2, "x: {}", quat.v.x).unwrap();
        };
        Text::new(
            text2.as_str(),
            Point::new(10, 60),
            character_style)
        .draw(&mut display).unwrap();

        serial.write_str(text2.as_str()).unwrap();
        write!(&mut serial, "\r\n").unwrap();

        delay.delay_ms(2000u32);
    }
}
