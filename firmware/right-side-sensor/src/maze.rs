use bitfield::bitfield;
use core::f32::consts::{PI, FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};

use heapless::Vec;
use heapless::consts::*;

use nalgebra as na;
use na::{Vector3, vector, Vector2};

/*
 * 迷路関連
 */
bitfield! {
    /// 迷路の壁
    #[derive(Clone, Copy)]
    pub struct MazeCell(u8);
    pub north, set_north: 7, 6;
    pub east, set_east: 5, 4;
    pub south, set_south: 3, 2;
    pub west, set_west: 1, 0;
}

bitfield! {
    /// 側面の壁
    #[derive(Clone, Copy)]
    pub struct SideWall(u8);
    pub front_right, set_front_right: 7, 6;
    pub back_right, set_back_right: 5, 4;
    pub front_left, set_front_left: 3, 2;
    pub back_left, set_back_left: 1, 0;
}

pub const NO_WALL: u8 = 0;
pub const WALL: u8 = 1;
pub const UNKNOWN_WALL: u8 = 3;

pub const CELL_____: MazeCell = MazeCell(0b00000000);
pub const CELL_N___: MazeCell = MazeCell(0b01000000);
pub const CELL__E__: MazeCell = MazeCell(0b00010000);
pub const CELL_NE__: MazeCell = MazeCell(0b01010000);
pub const CELL___S_: MazeCell = MazeCell(0b00000100);
pub const CELL_N_S_: MazeCell = MazeCell(0b01000100);
pub const CELL__ES_: MazeCell = MazeCell(0b00010100);
pub const CELL_NES_: MazeCell = MazeCell(0b01010100);
pub const CELL____W: MazeCell = MazeCell(0b00000001);
pub const CELL_N__W: MazeCell = MazeCell(0b01000001);
pub const CELL__E_W: MazeCell = MazeCell(0b00010001);
pub const CELL_NE_W: MazeCell = MazeCell(0b01010001);
pub const CELL___SW: MazeCell = MazeCell(0b00000101);
pub const CELL_N_SW: MazeCell = MazeCell(0b01000101);
pub const CELL__ESW: MazeCell = MazeCell(0b00010101);
pub const CELL_NESW: MazeCell = MazeCell(0b01010101);

pub type Maze = Vec<Vec<MazeCell, U16>, U16>;

pub const MAZE_CELL_SIZE_MM: f32 = 180.0_f32;

pub fn get_maze_cell_in(pose: &Vector3<f32>) -> Vector2<i32> {
    vector![(pose.x / MAZE_CELL_SIZE_MM) as i32, (pose.y / MAZE_CELL_SIZE_MM) as i32]
}

pub fn get_fine_maze_cell_in(pose: &Vector3<f32>) -> Vector2<i32> {
    vector![((pose.x - MAZE_CELL_SIZE_MM / 4.0_f32) / (MAZE_CELL_SIZE_MM / 2.0_f32)) as i32,
        ((pose.y - MAZE_CELL_SIZE_MM / 4.0_f32) / (MAZE_CELL_SIZE_MM / 2.0_f32)) as i32]
}

pub fn calc_side_wall(maze: &Maze, pose: &Vector3<f32>) -> SideWall {
    let cell = get_fine_maze_cell_in(pose);
    let direction = pose.z + FRAC_PI_2;

    if cell.x & 1 == 0 && cell.y & 1 == 0 {
        // マス目の中央付近にいる場合
        let maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2) as usize];

        if FRAC_PI_4 < direction && direction <= FRAC_PI_4 * 3.0_f32 {
            // 北向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_right(maze_cell.east());
            side_wall.set_back_right(maze_cell.east());
            side_wall.set_front_left(maze_cell.west());
            side_wall.set_back_left(maze_cell.west());

            side_wall
        } else if -FRAC_PI_4 < direction && direction <= FRAC_PI_4 {
            // 東向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_left(maze_cell.north());
            side_wall.set_back_left(maze_cell.north());
            side_wall.set_front_right(maze_cell.south());
            side_wall.set_back_right(maze_cell.south());

            side_wall
        } else if FRAC_PI_4 * 3.0_f32 < direction && direction <= FRAC_PI_4 * 5.0_f32 {
            // 西向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_left(maze_cell.south());
            side_wall.set_back_left(maze_cell.south());
            side_wall.set_front_right(maze_cell.north());
            side_wall.set_back_right(maze_cell.north());

            side_wall
        } else {
            // 南向き
            let mut side_wall = SideWall(0_u8);
            side_wall.set_front_right(maze_cell.west());
            side_wall.set_back_right(maze_cell.west());
            side_wall.set_front_left(maze_cell.east());
            side_wall.set_back_left(maze_cell.east());

            side_wall
        }
    } else {
        // マス目の境目近辺
        if cell.y & 1 != 0 {
            // 南北方向の境目
            if FRAC_PI_4 < direction && direction <= FRAC_PI_4 * 3.0_f32 {
                // 北向き
                let front_maze_cell = maze[(cell.y / 2 + 1) as usize][(cell.x / 2) as usize];
                let back_maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2) as usize];

                let mut side_wall = SideWall(0_u8);
                side_wall.set_front_right(front_maze_cell.east());
                side_wall.set_back_right(back_maze_cell.east());
                side_wall.set_front_left(front_maze_cell.west());
                side_wall.set_back_left(back_maze_cell.west());
    
                side_wall
            } else {
                // 南向き
                let front_maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2) as usize];
                let back_maze_cell = maze[(cell.y / 2 + 1) as usize][(cell.x / 2) as usize];

                let mut side_wall = SideWall(0_u8);
                side_wall.set_front_right(front_maze_cell.west());
                side_wall.set_back_right(back_maze_cell.west());
                side_wall.set_front_left(front_maze_cell.east());
                side_wall.set_back_left(back_maze_cell.east());
    
                side_wall
            }
        } else {
            // 東西方向の境目
            if -FRAC_PI_4 < direction && direction <= FRAC_PI_4 {
                // 東向き
                let front_maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2 + 1) as usize];
                let back_maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2) as usize];

                let mut side_wall = SideWall(0_u8);
                side_wall.set_front_left(front_maze_cell.north());
                side_wall.set_back_left(back_maze_cell.north());
                side_wall.set_front_right(front_maze_cell.south());
                side_wall.set_back_right(back_maze_cell.south());
    
                side_wall
            } else {
                // 西向き
                let front_maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2) as usize];
                let back_maze_cell = maze[(cell.y / 2) as usize][(cell.x / 2 + 1) as usize];

                let mut side_wall = SideWall(0_u8);
                side_wall.set_front_left(front_maze_cell.south());
                side_wall.set_back_left(back_maze_cell.south());
                side_wall.set_front_right(front_maze_cell.north());
                side_wall.set_back_right(back_maze_cell.north());
    
                side_wall
            }
        }
    }
}
