use cortex_m::peripheral::NVIC;

use vl53l0x::VL53L0x;
use xca9548a::{I2cSlave, Xca9548a};

use wio_terminal as wio;
use wio::hal::common::eic::pin::{ExtInt4, ExtInt6, ExtInt7, ExtInt12, ExtInt13, ExtInt14, Sense};
use wio::hal::eic::ConfigurableEIC;
use wio::hal::gpio;
use wio::hal::gpio::v1::{Port, Pa4, Pa6, Pb7, Pb12, Pb13, Pb14};
use wio::hal::gpio::v2::{Alternate, D, Floating, Input, Interrupt, PA16, PA17};
use wio::hal::sercom::v2::{Pad0, Pad1};
use wio::hal::sercom::Pad;
use wio::hal::sercom::I2CMaster3;
use wio::pac::{SERCOM3, interrupt};
use wio::prelude::*;


/* 
 * センサ系
 */
pub struct SensorEvent {
    pub id: u16,
    pub distance: u16,
}

pub type SensorI2C = I2CMaster3<Pad<SERCOM3, Pad0, gpio::v1::Pin<PA17, Alternate<D>>>, Pad<SERCOM3, Pad1, gpio::v1::Pin<PA16, Alternate<D>>>>;

pub struct TofSensors<'a> {
    pub sensor0_gpio1: ExtInt4<Pa4<Interrupt<Floating>>>,
    pub sensor0_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    pub sensor1_gpio1: ExtInt7<Pb7<Interrupt<Floating>>>,
    pub sensor1_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    pub sensor2_gpio1: ExtInt6<Pa6<Interrupt<Floating>>>,
    pub sensor2_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    pub sensor3_gpio1: ExtInt14<Pb14<Interrupt<Floating>>>,
    pub sensor3_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    pub sensor4_gpio1: ExtInt12<Pb12<Interrupt<Floating>>>,
    pub sensor4_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,

    pub sensor5_gpio1: ExtInt13<Pb13<Interrupt<Floating>>>,
    pub sensor5_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
}

impl<'a> TofSensors<'a> {
    pub fn new(
        sensor0_gpio1: Pa4<Input<Floating>>,
        mut sensor0_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor1_gpio1: Pb7<Input<Floating>>,
        mut sensor1_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor2_gpio1: Pa6<Input<Floating>>,
        mut sensor2_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor3_gpio1: Pb14<Input<Floating>>,
        mut sensor3_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor4_gpio1: Pb12<Input<Floating>>,
        mut sensor4_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
    
        sensor5_gpio1: Pb13<Input<Floating>>,
        mut sensor5_i2c: VL53L0x<I2cSlave<'a, Xca9548a<SensorI2C>, SensorI2C>>,
        eic: &mut ConfigurableEIC,
        port: &mut Port,
    ) -> TofSensors<'a> {
        let mut sensor0_gpio1 = sensor0_gpio1.into_floating_ei(port);
        sensor0_gpio1.sense(eic, Sense::FALL);
        sensor0_gpio1.enable_interrupt(eic);
        sensor0_i2c.start_continuous(0).unwrap();

        let mut sensor1_gpio1 = sensor1_gpio1.into_floating_ei(port);
        sensor1_gpio1.sense(eic, Sense::FALL);
        sensor1_gpio1.enable_interrupt(eic);
        sensor1_i2c.start_continuous(0).unwrap();

        let mut sensor2_gpio1 = sensor2_gpio1.into_floating_ei(port);
        sensor2_gpio1.sense(eic, Sense::FALL);
        sensor2_gpio1.enable_interrupt(eic);
        sensor2_i2c.start_continuous(0).unwrap();

        let mut sensor3_gpio1 = sensor3_gpio1.into_floating_ei(port);
        sensor3_gpio1.sense(eic, Sense::FALL);
        sensor3_gpio1.enable_interrupt(eic);
        sensor3_i2c.start_continuous(0).unwrap();

        let mut sensor4_gpio1 = sensor4_gpio1.into_floating_ei(port);
        sensor4_gpio1.sense(eic, Sense::FALL);
        sensor4_gpio1.enable_interrupt(eic);
        sensor4_i2c.start_continuous(0).unwrap();

        let mut sensor5_gpio1 = sensor5_gpio1.into_floating_ei(port);
        sensor5_gpio1.sense(eic, Sense::FALL);
        sensor5_gpio1.enable_interrupt(eic);
        sensor5_i2c.start_continuous(0).unwrap();

        TofSensors {
            sensor0_gpio1,
            sensor0_i2c,
            sensor1_gpio1,
            sensor1_i2c, 
            sensor2_gpio1,
            sensor2_i2c,
            sensor3_gpio1,
            sensor3_i2c,
            sensor4_gpio1,
            sensor4_i2c,
            sensor5_gpio1,
            sensor5_i2c,
        }
    }

    pub fn enable(&self, nvic: &mut NVIC, interrupt0: interrupt, interrupt1: interrupt, interrupt2: interrupt,
            interrupt3: interrupt, interrupt4: interrupt, interrupt5: interrupt) {
        unsafe {
            nvic.set_priority(interrupt0, 1);
            NVIC::unmask(interrupt0);
            nvic.set_priority(interrupt1, 1);
            NVIC::unmask(interrupt1);
            nvic.set_priority(interrupt2, 1);
            NVIC::unmask(interrupt2);
            nvic.set_priority(interrupt3, 1);
            NVIC::unmask(interrupt3);
            nvic.set_priority(interrupt4, 1);
            NVIC::unmask(interrupt4);
            nvic.set_priority(interrupt5, 1);
            NVIC::unmask(interrupt5);
        }
    }
}

#[macro_export]
macro_rules! tof_interrupt {
    ($Sensors:ident, $Pin:ident, $I2c:ident, $SensorId:expr, $Interrupt:ident) => {
        #[interrupt]
        fn $Interrupt() {
            disable_interrupts(|_cs| unsafe {
                let sensors = $Sensors.as_mut().unwrap();
                if sensors.$Pin.is_interrupt() {
                    sensors.$Pin.clear_interrupt();

                    let mut q = EVENT_QUEUE.split().0;
                    q.enqueue(SensorEvent {
                        id: $SensorId,
                        distance: sensors.$I2c.read_range_mm().unwrap()
                    }).ok();
                }
            });
        }
    };
}

pub struct SensorMeasuredValue {
    id: u32,
    value: f32,
    values: [f32; 3],
    index: usize,
    calibration_value: f32
}

impl SensorMeasuredValue {
    pub fn new(id: u32, calibration_value: f32) -> SensorMeasuredValue {
        SensorMeasuredValue {
            id,
            value: 0.0_f32,
            values: [0.0_f32; 3],
            index: 0,
            calibration_value
        }
    }

    pub fn append_value(&mut self, value: f32) {
        self.values[self.index] = value + self.calibration_value;
        self.index += 1;
        self.index %= self.values.len();
        self.value = self.values.iter().sum::<f32>() / (self.values.len() as f32);
    }

    pub fn get_value(&self) -> f32 {
        self.value
    }
}