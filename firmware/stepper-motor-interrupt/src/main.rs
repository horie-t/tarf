#![no_std]
#![no_main]

use panic_halt as _;
use wio_terminal as wio;

use wio::hal::clock::GenericClockController;
use wio::hal::common::eic;
use wio::hal::common::eic::pin::*;
use wio::hal::gpio::*;
use wio::hal::hal::spi;
use wio::hal::sercom::*;
use wio::pac::{interrupt, CorePeripherals, Peripherals, EIC, MCLK};
use wio::prelude::*;
use wio::{entry, Button, ButtonEvent, Pins};

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use cortex_m::peripheral::NVIC;

use heapless::spsc::Queue;

struct L6470 {
    spi: SPIMaster5<Sercom5Pad2<Pb0<PfD>>, Sercom5Pad0<Pb2<PfD>>, Sercom5Pad1<Pb3<PfD>>>,
    cs: Pb1<Output<PushPull>>,
}

impl L6470 {
    fn send(&mut self, b: u8) -> () {
        self.cs.set_low().unwrap();
        self.spi.write(&[b]).unwrap();
        self.cs.set_high().unwrap();
    }
}

struct InterruptPins {
    pub interrupt_pin: Pb8<Input<Floating>>,
}

impl InterruptPins {
    pub fn init(
        self,
        eic: EIC,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
        port: &mut Port,
    ) -> InterruptController {
        let clk = clocks.gclk1();
        let mut eic = eic::init_with_ulp32k(mclk, clocks.eic(&clk).unwrap(), eic);
        eic.button_debounce_pins(&[self.interrupt_pin.id()]);

        let mut interrupt_pin = self.interrupt_pin.into_floating_ei(port);
        interrupt_pin.sense(&mut eic, Sense::BOTH);
        interrupt_pin.enable_interrupt(&mut eic);
        InterruptController {
            _eic: eic.finalize(),
            interrupt_pin,
        }
    }
}

pub struct InterruptController {
    _eic: eic::EIC,
    interrupt_pin: ExtInt8<Pb8<Interrupt<Floating>>>,
}

macro_rules! isr {
    ($Handler:ident, $($Event:expr, $Pin:ident),+) => {
        pub fn $Handler(&mut self) -> Option<ButtonEvent> {
            $(
                {
                    let pin = &mut self.$Pin;
                    if pin.is_interrupt() {
                        pin.clear_interrupt();
                        return Some(ButtonEvent {
                            button: $Event,
                            down: !pin.state(),
                        })
                    }
                }
            )+

            None
        }
    };
}

impl InterruptController {
    pub fn enable(&self, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::EIC_EXTINT_8, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_8);
        }
    }

    isr!(interrupt_extint8, Button::TopLeft, interrupt_pin);
}

static mut L6470_GLOBAL: Option<L6470> = None;

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
    let gclk0 = &clocks.gclk0();

    let mut pins = Pins::new(peripherals.PORT);

    let interrupt_pins = InterruptPins { interrupt_pin: pins.a0_d0 };
    let interrupt_pin_ctrlr = interrupt_pins.init(
        peripherals.EIC,
        &mut clocks,
        &mut peripherals.MCLK,
        &mut pins.port,
    );

    let nvic = &mut core.NVIC;
    disable_interrupts(|_| unsafe {
        interrupt_pin_ctrlr.enable(nvic);
        INTERRUPT_CTRLR = Some(interrupt_pin_ctrlr);
    });

    let mut consumer = unsafe { Q.split().1 };

    let spi: SPIMaster5<Sercom5Pad2<Pb0<PfD>>, Sercom5Pad0<Pb2<PfD>>, Sercom5Pad1<Pb3<PfD>>> =
        SPIMaster5::new(
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
    unsafe {
        L6470_GLOBAL = Some(L6470 { spi, cs });

        let l6470 = L6470_GLOBAL.as_mut().unwrap();

        // 中途半端なコマンドをフラッシュ
        l6470.send(0x00);
        l6470.send(0x00);
        l6470.send(0x00);
        l6470.send(0x00);
        // HOMEポジションへ
        l6470.send(0xC0);

        // 最大回転速度設定
        l6470.send(0x07);
        l6470.send(0x00);
        l6470.send(0x60);

        // モータ停止中の電圧
        l6470.send(0x09);
        l6470.send(0xFF);

        // モータ定速回転中の電圧
        l6470.send(0x0A);
        l6470.send(0xFF);

        // モータ加速中の電圧
        l6470.send(0x0B);
        l6470.send(0xFF);

        // モータ減速中の電圧
        l6470.send(0x0C);
        l6470.send(0xFF);

        // ステップモード
        l6470.send(0x16);
        l6470.send(0x00);
        l6470.send(0x00);
        l6470.send(0x00);

        // Moveコマンドを正転で実行
        l6470.send(0x41);
        //ステップ(2回転)(400 = 0x190)
        l6470.send(0x00);
        l6470.send(0x01);
        l6470.send(0x90);
    }

    loop {
        if let Some(press) = consumer.dequeue() {
            if !press.down {
                unsafe {
                    let l6470 = L6470_GLOBAL.as_mut().unwrap();
                    // Moveコマンドを正転で実行
                    l6470.send(0x41);
                    //ステップ(2回転)(400 = 0x190)
                    l6470.send(0x00);
                    l6470.send(0x01);
                    l6470.send(0x90);
                }
            }
        }
    }
}

static mut INTERRUPT_CTRLR: Option<InterruptController> = None;
static mut Q: Queue<ButtonEvent, 8> = Queue::new();

macro_rules! pin_interrupt {
    ($controller:ident, unsafe fn $func_name:ident ($cs:ident: $cstype:ty, $event:ident: ButtonEvent ) $code:block) => {
        unsafe fn $func_name($cs: $cstype, $event: ButtonEvent) {
            $code
        }

        macro_rules! _pin_interrupt_handler {
            ($Interrupt:ident, $Handler:ident) => {
                #[interrupt]
                fn $Interrupt() {
                    disable_interrupts(|cs| unsafe {
                        $controller.as_mut().map(|ctrlr| {
                            if let Some(event) = ctrlr.$Handler() {
                                $func_name(cs, event);
                            }
                        });
                    });
                }
            };
        }

        _pin_interrupt_handler!(EIC_EXTINT_8, interrupt_extint8);
    };
}

pin_interrupt!(
    INTERRUPT_CTRLR,
    unsafe fn on_button_event(_cs: &CriticalSection, event: ButtonEvent) {
        let mut q = Q.split().0;
        q.enqueue(event).ok();
    }
);
