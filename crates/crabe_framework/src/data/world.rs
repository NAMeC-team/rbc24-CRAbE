use serde::{Deserialize, Serialize};

pub const MAX_ROBOTS: usize = 16;

#[derive(Debug)]
pub struct Field {
    pub width: f32,
    pub length: f32,
}

#[derive(Debug)]
pub struct Goal {
    pub width: f32,
    pub depth: f32,
}

#[derive(Debug)]
pub struct Penalty {
    pub width: f32,
    pub depth: f32,
}

#[derive(Debug)]
pub struct Circle {
    // center ?
    pub radius: f32,
}

#[derive(Debug)]
pub struct Geometry {
    pub field : Field,
    pub ally_goal: Goal,
    pub opponent_goal: Goal,
    pub ally_penalty: Penalty,
    pub opponent_penalty: Penalty,
    pub center: Circle,
}

pub struct AllyInfo;
pub struct EnemyInfo;

pub struct Robot<T> {
    id: u32,
    // pos : 2D
    orientation: f32,
    has_ball: bool,
    robot_info: T,
}

pub struct Ball {
    // pos : 3D
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum TeamColor {
    BLUE,
    YELLOW,
}

pub struct Team {
    color: TeamColor,
    name: String,
}

pub struct GameState {
    pub ally: Team,
    pub enemy: Team,
    pub blue_positive_half: bool,
}

pub struct World {
    pub state: GameState,
    pub geometry: Geometry, // TODO : Add default value
    pub allies_bot: [Option<Robot<AllyInfo>>; MAX_ROBOTS],
    pub enemies_bot: [Option<Robot<EnemyInfo>>; MAX_ROBOTS],
    pub ball: Ball,
}