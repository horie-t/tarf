use core::f32::consts::PI;
use core::fmt::Write;
use cortex_m::peripheral::NVIC;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::primitives::Circle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, Rectangle, PrimitiveStyle};
use embedded_graphics::text::Text;

use heapless::String;
use heapless::consts::*;
use heapless::spsc::Queue;

use micromath::F32Ext;
use nalgebra as na;
use na::{Vector2, Vector3, matrix, vector};

use wio_terminal as wio;
use wio::LCD;
use wio::hal::common::eic::pin::{ExtInt10, ExternalInterrupt, Sense};
use wio::hal::eic::ConfigurableEIC;
use wio::hal::gpio::v1::{Port, Pc26};
use wio::hal::gpio::v2::{Floating, Input, Interrupt};
use wio::pac::interrupt;
use wio::prelude::*;

use super::{Maze, SideWall, WALL};

/* 
 * スタート・ボタン系
 */
pub struct Button {
    pub queue: Queue<ButtonEvent, U8>,
    pub pin: ExtInt10<Pc26<Interrupt<Floating>>>,
}

impl Button {
    pub fn new(
        pin: Pc26<Input<Floating>>,
        queue: Queue<ButtonEvent, U8>,
        eic: &mut ConfigurableEIC,
        port: &mut Port,
    ) -> Button {
        eic.button_debounce_pins(&[pin.id()]);

        let mut pin = pin.into_floating_ei(port);
        pin.sense(eic, Sense::FALL);
        pin.enable_interrupt(eic);

        Button {
            pin, queue
        }
    }

    pub fn enable(&self, nvic: &mut NVIC, interrupt: interrupt) {
        unsafe {
            nvic.set_priority(interrupt, 1);
            NVIC::unmask(interrupt);
        }
    }
}

pub struct ButtonEvent {
    pub pressed: bool,
}

#[macro_export]
macro_rules! button_interrupt {
    ($Button:ident, $Interrupt:ident) => {
        #[interrupt]
        fn $Interrupt() {
            disable_interrupts(|_cs| unsafe {
                let button = $Button.as_mut().unwrap();
                if button.pin.is_interrupt() {
                    button.pin.clear_interrupt();
                    let mut q = button.queue.split().0;
                    q.enqueue(ButtonEvent { pressed: true }).ok();
                }
            });
        }
    };
}

/* 
 * 画面系
 */
pub struct MapView {
    pub top_left: Point,
    pub size: Size,
}

impl MapView {
    const MAP_CELL_LENGTH_PIXEL: i32 = 10;
    const MAP_CELL_COUNT_Y: i32 = 16;

    pub fn draw_maze<D>(&self, target: &mut D, maze: &Maze) 
    where
        D: DrawTarget<Color = Rgb565> {
            for (y, row) in maze.iter().rev().enumerate() {  // mazeは左下を原点になっているので、画面を描きやすいようにrev()
                for (x, maze_cell) in row.iter().enumerate() {
                    let x = x as i32;
                    let y = y as i32;
                    if maze_cell.north() == WALL {
                        Line::new(Point::new(self.top_left.x + x * 10, self.top_left.y + y * 10),
                         Point::new(self.top_left.x + x * 10 + 10, self.top_left.y + y * 10))
                         .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                         .draw(target).ok();
                    }
                    if maze_cell.east() == WALL {
                        Line::new(Point::new(self.top_left.x + x * 10 + 10, self.top_left.y + y * 10),
                         Point::new(self.top_left.x + x * 10 + 10, self.top_left.y + y * 10 + 10))
                         .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                         .draw(target).ok();
                    }
                    if maze_cell.south() == WALL {
                        Line::new(Point::new(self.top_left.x + x * 10, self.top_left.y + y * 10 + 10),
                         Point::new(self.top_left.x + x * 10 + 10, self.top_left.y + y * 10 + 10))
                         .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                         .draw(target).ok();
                    }
                    if maze_cell.west() == WALL {
                        Line::new(Point::new(self.top_left.x + x * 10, self.top_left.y + y * 10),
                         Point::new(self.top_left.x + x * 10, self.top_left.y + y * 10 + 10))
                         .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                         .draw(target).ok();
                    }
                }
            }
    }

    pub fn clear_maze<D>(&self, target: &mut D)
    where
        D: DrawTarget<Color = Rgb565> {
            Rectangle::new(self.top_left, self.size)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(target).ok();
    }

    pub fn draw_route<D>(&self, target: &mut D, route: &[(Vector2<f32>, Vector2<f32>)])
    where
        D: DrawTarget<Color = Rgb565> {
            let origin = self.top_left + Point::new(Self::MAP_CELL_LENGTH_PIXEL / 2, Self::MAP_CELL_LENGTH_PIXEL / 2);
            for link in route {
                let start_node = Point::new(link.0.x as i32, (Self::MAP_CELL_COUNT_Y - 1) - link.0.y as i32) * Self::MAP_CELL_LENGTH_PIXEL;
                let end_node = Point::new(link.1.x as i32, (Self::MAP_CELL_COUNT_Y - 1) - link.1.y as i32) * Self::MAP_CELL_LENGTH_PIXEL;

                Line::new(start_node + origin, end_node + origin)
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
                .draw(target).ok();
            }
    }

    pub fn draw_vehicle<D>(&self, target: &mut D, vehicle_pose: &Vector3<f32>)
    where
        D: DrawTarget<Color = Rgb565> {
            let mut position = vehicle_pose.xy() / 18.0_f32;
            position.y = (Self::MAP_CELL_COUNT_Y * Self::MAP_CELL_LENGTH_PIXEL) as f32 - position.y;

            Circle::with_center(Point::new(self.top_left.x + position.x as i32, self.top_left.y + position.y as i32), 4)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
            .draw(target).ok();
    }
}

pub struct SideWallView {
    pub top_left: Point,
    pub size: Size,
}

impl SideWallView {
    pub const MAP_CELL_LENGTH_PIXEL: i32 = 10;
    
    pub fn draw_wall<D>(&self, target: &mut D, wall: &SideWall) 
    where
        D: DrawTarget<Color = Rgb565> {
            Rectangle::new(self.top_left, self.size)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(target).ok();

            if wall.front_left() == WALL {
                Line::new(Point::new(self.top_left.x, self.top_left.y),
                Point::new(self.top_left.x, self.top_left.y + Self::MAP_CELL_LENGTH_PIXEL / 2))
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                .draw(target).ok();
            }
            if wall.front_right() == WALL {
                Line::new(Point::new(self.top_left.x + Self::MAP_CELL_LENGTH_PIXEL, self.top_left.y),
                Point::new(self.top_left.x + Self::MAP_CELL_LENGTH_PIXEL, self.top_left.y + Self::MAP_CELL_LENGTH_PIXEL / 2))
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                .draw(target).ok();
            }
            if wall.back_left() == WALL {
                Line::new(Point::new(self.top_left.x, self.top_left.y + Self::MAP_CELL_LENGTH_PIXEL / 2),
                Point::new(self.top_left.x, self.top_left.y + Self::MAP_CELL_LENGTH_PIXEL))
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                .draw(target).ok();
            }
            if wall.back_right() == WALL {
                Line::new(Point::new(self.top_left.x + Self::MAP_CELL_LENGTH_PIXEL, self.top_left.y + Self::MAP_CELL_LENGTH_PIXEL / 2),
                Point::new(self.top_left.x + Self::MAP_CELL_LENGTH_PIXEL, self.top_left.y + Self::MAP_CELL_LENGTH_PIXEL))
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                .draw(target).ok();
            }
    }
}

pub fn clear_display(display: &mut LCD) {
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

pub fn println_display(display: &mut LCD, text: &str) {
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

pub fn print_current_cell(display: &mut LCD, pose: &Vector2<f32>) {
    let cell = pose / 180.0_f32;

    let fill = PrimitiveStyle::with_fill(Rgb565::BLACK);
    Rectangle::new(Point::new(10, 42), Size::new(140, 21))
    .into_styled(fill)
    .draw(display).unwrap();


    let mut text: String<U40> = String::new();
    write!(text, "({}, {})", cell.x.trunc() as i32, cell.y.trunc() as i32).unwrap();

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(
        text.as_str(),
        Point::new(10, 60),
        character_style)
    .draw(display).unwrap();
}

pub fn print_current_pose(display: &mut LCD, pose: &Vector3<f32>) {
    let fill = PrimitiveStyle::with_fill(Rgb565::BLACK);
    Rectangle::new(Point::new(10, 42), Size::new(140, 21))
    .into_styled(fill)
    .draw(display).unwrap();


    let mut text: String<U40> = String::new();
    write!(text, "({}, {}, {})", pose.x.trunc() as i32, pose.y.trunc() as i32, (pose.z / PI * 180.0_f32).trunc() as i32).unwrap();

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new(
        text.as_str(),
        Point::new(10, 60),
        character_style)
    .draw(display).unwrap();
}
