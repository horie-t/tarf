#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal as wio;

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::hal::gpio::*;
use wio::hal::hal::spi;
use wio::hal::sercom::*;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;

fn l6470_send(spi: &mut SPIMaster5<Sercom5Pad2<Pb0<PfD>>, Sercom5Pad0<Pb2<PfD>>, Sercom5Pad1<Pb3<PfD>>>,
              b : u8,
              cs: &mut Pb1<Output<PushPull>>) -> () {
    cs.set_low().unwrap();
    spi.write(&[b]).unwrap();
    cs.set_high().unwrap();
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

    let mut pins = wio::Pins::new(peripherals.PORT);
    let gclk0 = &clocks.gclk0();

    // SPIを初期化
    let mut spi: SPIMaster5<
        Sercom5Pad2<Pb0<PfD>>,
        Sercom5Pad0<Pb2<PfD>>,
        Sercom5Pad1<Pb3<PfD>>,
    > = SPIMaster5::new(
        &clocks.sercom5_core(&gclk0).unwrap(),
        4.mhz(),
        spi::MODE_3,
        peripherals.SERCOM5,
        &mut peripherals.MCLK,
        (
            pins.spi_miso.into_pad(&mut pins.port),
            pins.spi_mosi.into_pad(&mut pins.port),
            pins.spi_sck.into_pad(&mut pins.port),
        ),
    );
    let mut cs = pins.spi_cs.into_push_pull_output(&mut pins.port);
    cs.set_high().unwrap();
    let mut motor_clk = pins.i2c1_sda.into_push_pull_output(&mut pins.port);

    /* ステッピングモータの初期化
     */
    // しばらく何もしない(NOP)
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    // HOMEポジションへ
    l6470_send(&mut spi, 0xC0, &mut cs);
    // ステップ・クロックモードで動作
    l6470_send(&mut spi, 0x59, &mut cs);

    loop {
        motor_clk.toggle();
        delay.delay_us(25u16);
    }
}
