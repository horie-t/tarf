#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal::entry;
use wio_terminal::hal::clock::GenericClockController;
use wio_terminal::hal::delay::Delay;
use wio_terminal::hal::gpio::*;
use wio_terminal::hal::hal::spi;
use wio_terminal::hal::sercom::*;
use wio_terminal::pac::{CorePeripherals, Peripherals};
use wio_terminal::prelude::*;

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

    let mut pins = wio_terminal::Pins::new(peripherals.PORT);
    let gclk0 = &clocks.gclk0();
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

    // 中途半端なコマンドをフラッシュ
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    // HOMEポジションへ
    l6470_send(&mut spi, 0xC0, &mut cs);

    // 最大回転速度設定
    l6470_send(&mut spi, 0x07, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x60, &mut cs);

    // モータ停止中の電圧
    l6470_send(&mut spi, 0x09, &mut cs);
    l6470_send(&mut spi, 0xFF, &mut cs);

    // モータ定速回転中の電圧
    l6470_send(&mut spi, 0x0A, &mut cs);
    l6470_send(&mut spi, 0xFF, &mut cs);

    // モータ加速中の電圧
    l6470_send(&mut spi, 0x0B, &mut cs);
    l6470_send(&mut spi, 0xFF, &mut cs);

    // モータ減速中の電圧
    l6470_send(&mut spi, 0x0C, &mut cs);
    l6470_send(&mut spi, 0xFF, &mut cs);

    // ステップモード
    l6470_send(&mut spi, 0x16, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);
    l6470_send(&mut spi, 0x00, &mut cs);

    loop {
        /*
        * 6秒おきに2回転させる
        */
        // Moveコマンドを正転で実行
        l6470_send(&mut spi, 0x41, &mut cs);
        // 400ステップ(2回転)(400 = 0x190)
        l6470_send(&mut spi, 0x00, &mut cs);
        l6470_send(&mut spi, 0x01, &mut cs);
        l6470_send(&mut spi, 0x90, &mut cs);
        
        // 6秒待つ
        delay.delay_ms(6000u16);
    }
}
