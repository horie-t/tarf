use core::f32::consts::{PI, FRAC_PI_3, FRAC_PI_8};

use cortex_m::peripheral::NVIC;
use micromath::F32Ext;

use nalgebra as na;
use na::{Matrix3, Vector3, vector, Vector2};
use na::base::SVector;

use wio_terminal as wio;
use wio::hal::gpio::v2::{Floating, Input, Output, Pin, PinId, PushPull};
use wio::hal::timer::{Count16, TimerCounter};
use wio::pac::interrupt;
use wio::prelude::*;

/* 
 * オムニホイール系
 */
#[derive(Clone, Copy)]
pub enum WheelRotateDirection {
    ClockWise,
    CounterClockWise,
}

pub struct Wheel<S: PinId, D: PinId, T: Count16> {
    pub id: u32,
    pub direction: WheelRotateDirection,
    pub step_pin: Pin<S, Output<PushPull>>,
    pub direction_pin: Pin<D, Output<PushPull>>,
    pub timer_counter: TimerCounter<T>,
    pub radius: f32,
}

impl<S: PinId, D: PinId, T: Count16> Wheel<S, D, T> {
    const RADIUS: f32 = 24.0_f32;

    pub fn new(id: u32, step_pin: Pin<S, Input<Floating>>, direction_pin: Pin<D, Input<Floating>>,
        timer_counter: TimerCounter<T>, interrupt: interrupt) -> Wheel<S, D, T>{
            let mut wheel = Wheel {
                id,
                direction: WheelRotateDirection::ClockWise,
                step_pin: step_pin.into_push_pull_output(),
                direction_pin: direction_pin.into_push_pull_output(),
                timer_counter,
                radius: Self::RADIUS,
            };
            wheel.set_rotate_direction(WheelRotateDirection::ClockWise);

            unsafe {
                NVIC::unmask(interrupt);
            }

            wheel
    }

    pub fn step(&mut self) {
        self.step_pin.toggle().unwrap();
    }

    pub fn set_rotate_direction(&mut self, dir: WheelRotateDirection) {
        self.direction = dir;
        match dir {
            WheelRotateDirection::ClockWise => {
                self.direction_pin.set_high().unwrap();
            },
            WheelRotateDirection::CounterClockWise => {
                self.direction_pin.set_low().unwrap();
            }
        }
    }

    pub fn start_with_rps(&mut self, rotate_per_sec: f32) {
        if rotate_per_sec < 0_f32 {
            self.direction = WheelRotateDirection::CounterClockWise;
            self.set_rotate_direction(WheelRotateDirection::CounterClockWise)
        } else {
            self.direction = WheelRotateDirection::ClockWise;
            self.set_rotate_direction(WheelRotateDirection::ClockWise)
        }
        let pulse_hz = ((2_f32 * 200_f32 * rotate_per_sec.abs()) as u32).hz();
        self.timer_counter.start(pulse_hz);
        self.timer_counter.enable_interrupt();
    }

    pub fn start_with_speed(&mut self, speed_mm_per_sec: f32) {
        if -0.1_f32< speed_mm_per_sec && speed_mm_per_sec < 0.1 {
            self.stop();
        } else {
            self.start_with_rps(speed_mm_per_sec / (2.0_f32 * PI * self.radius));
        }
    }

    pub fn stop(&mut self) {
        self.timer_counter.disable_interrupt();
    }
}

#[macro_export]
macro_rules! wheel_interrupt {
    ($RunningSystem:ident, $Handler:ident, $Wheel:ident) => {
        #[interrupt]
        fn $Handler() {
            disable_interrupts(|_cs| unsafe {
                let running_system = $RunningSystem.as_mut().unwrap();
                running_system.$Wheel.step();
                running_system.$Wheel.timer_counter.wait().unwrap();

                let mut q = WHEEL_MOVED_EVENT_QUEUE.split().0;
                let distance = {
                    match running_system.$Wheel.direction {
                        WheelRotateDirection::ClockWise => {
                            (2.0_f32 * PI * running_system.$Wheel.radius) / (2.0_f32 * 200.0_f32)
                        },
                        WheelRotateDirection::CounterClockWise => {
                            (- 2.0_f32 * PI * running_system.$Wheel.radius) / (2.0_f32 * 200.0_f32)
                        }
                    }
                };
                q.enqueue(WheelMovedEvent {
                    id: running_system.$Wheel.id,
                    distance: distance
                }).ok();
            });
        }
    };
}

pub struct WheelMovedEvent {
    pub id: u32,
    pub distance: f32,
}

pub struct RunningSystem<S0: PinId, D0: PinId, T0: Count16, 
        S1: PinId, D1: PinId, T1: Count16, S2: PinId, D2: PinId, T2: Count16> {
    pub wheel_0: Wheel<S0, D0, T0>,
    pub wheel_1: Wheel<S1, D1, T1>,
    pub wheel_2: Wheel<S2, D2, T2>,
    pub mat_for_wheel_v: Matrix3<f32>,
    pub mat_for_odometry: Matrix3<f32>,
    pub velocity: Vector3<f32>,
    pub target_point: Vector3<f32>,
    pub trip_vec: Vector3<f32>,
}

impl <S0: PinId, D0: PinId, T0: Count16, S1: PinId, D1: PinId, T1: Count16, S2: PinId, D2: PinId, T2: Count16> RunningSystem<S0, D0, T0, S1, D1, T1, S2, D2, T2> {
    /// * `v` - 移動ベクトル。ロボット座標系。zは回転(rad/sec)を表す。
    pub fn run(&mut self, v: Vector3<f32>) {
        self.velocity = v;
        let mut wheels_v = self.mat_for_wheel_v * v;

        // 車輪の1つが進行方向と同じ向きに回転していると、滑りが発生せずその車輪だけ進み過ぎてしまうのを抑制する。
        if v.x * v.x + v.y * v.y > 0.0_f32 || v.x != 0.0_f32 {
            // 超信地旋回ではない、もしくはy軸方向への移動ではない場合

            // 進行方向が第2, 3象限の場合は、180度回転させた方向で判定しても同じ結果になる
            let direction_rad = 
                if v.x > 0.0_f32 {
                    (v.y / v.x).atan()
                } else {
                    (- v.y / v.x).atan()
                };
            
            if (direction_rad + FRAC_PI_3).abs() < FRAC_PI_8 {
                wheels_v[0] = wheels_v[0] * (1.0_f32 - 0.070 * (FRAC_PI_8 - (direction_rad + FRAC_PI_3).abs()) / FRAC_PI_8);
            } else if direction_rad.abs() < FRAC_PI_8 {
                wheels_v[1] = wheels_v[1] * (1.0_f32 - 0.070 * (FRAC_PI_8 - direction_rad.abs()              ) / FRAC_PI_8);
            } else if (direction_rad - FRAC_PI_3).abs() < FRAC_PI_8 {
                wheels_v[2] = wheels_v[2] * (1.0_f32 - 0.070 * (FRAC_PI_8 - (direction_rad - FRAC_PI_3).abs()) / FRAC_PI_8);
            }
        }

        self.wheel_0.start_with_speed(wheels_v[0]);
        self.wheel_1.start_with_speed(wheels_v[1]);
        self.wheel_2.start_with_speed(wheels_v[2]);
    }

    pub fn stop(&mut self) {
        self.wheel_0.stop();
        self.wheel_1.stop();
        self.wheel_2.stop();
    }

    pub fn move_to(&mut self, target_point: Vector3<f32>) {
        self.target_point = target_point;
        self.trip_vec = Vector3::zeros();

        let rotate_v = if target_point.z > 0.0_f32 {
            0.50_f32
        } else if target_point.z < 0.0_f32 {
            -0.50_f32
        } else {
            0.0_f32
        };

        let v = if target_point.xy().eq(&Vector2::<f32>::zeros()) {
            vector![0.0_f32, 0.0_f32, rotate_v]
        } else {
            let trans_normalized = target_point.xy().normalize();
            vector![20.0_f32 * trans_normalized.x, 20.0_f32 * trans_normalized.y, rotate_v]
        };

        self.run(v);
    }

    pub fn on_moved(&mut self, moved_event: WheelMovedEvent) -> bool {
        self.trip_vec += self.wheel_step_to_vec(moved_event);
        let distance = (self.target_point.xy() - self.trip_vec.xy()).magnitude();
        let diff_angle = (self.target_point.z - self.trip_vec.z).abs();
        if distance < 1.0_f32 && diff_angle < (PI / 90.0_f32) {
            // 目的地に到着
            self.stop();
            return true
        } else if distance < 1.0_f32 {
            self.run(vector![0.0_f32, 0.0_f32, self.velocity.z]);
        } else if diff_angle < (PI / 90.0_f32) {
            self.run(vector![self.velocity.x, self.velocity.y, 0.0_f32]);
        }

        false
    }

    pub fn wheel_step_to_vec(&self, moved_event: WheelMovedEvent) -> Vector3<f32> {
        let mut array = [0.0f32, 0.0f32, 0.0f32];
        array[moved_event.id as usize] = moved_event.distance;
        self.mat_for_odometry * SVector::from(array)
    }
}
