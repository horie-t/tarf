#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal::entry;
use wio_terminal::hal::clock::GenericClockController;
use wio_terminal::hal::gpio::*;
use wio_terminal::hal::hal::spi;
use wio_terminal::hal::sercom::*;
use wio_terminal::pac::{CorePeripherals, Peripherals};
use wio_terminal::prelude::*;

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

    loop {

    }
}
