use crate::data::FilterData;
use crate::post_filter::PostFilter;
use crabe_framework::data::world::{Ball, TeamColor, World, BallTouchInfo};

pub struct BallFilter;

fn calculate_last_touch(filter_data: &FilterData, ball: &mut Ball, world: &World) {
    let ball_world = match &world.ball {
        Some(b) => b,
        None => {
            ball.last_touch = None; return;}
    };
    ball.last_touch = ball_world.last_touch.clone();
    if dbg!(ball_world.acceleration.norm()) < 1. {
        return;
    }

    // If the ball is moving, we assume it was touched by a robot
    // TODO check closest robot to ball
    ball.last_touch = Some(BallTouchInfo {
        robot_id: 0,
        team_color: TeamColor::Blue,
        timestamp: filter_data.ball.data.timestamp,
        position: ball.position,
    });
}

impl PostFilter for BallFilter {
    fn step(&mut self, filter_data: &FilterData, world: &mut World) {
        let mut ball = filter_data.ball.data.clone();
        calculate_last_touch(&filter_data, &mut ball, &world);
        world.ball = Some(ball);
    }
}