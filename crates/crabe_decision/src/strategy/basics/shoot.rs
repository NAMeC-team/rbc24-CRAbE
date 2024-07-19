use std::time::Instant;
use crate::action::move_to::MoveTo;
use crabe_framework::data::output::Kick;
use crabe_framework::data::world::{AllyInfo, Ball, Robot, World};
use crabe_math::shape::Line;
use crabe_math::vectors;
use nalgebra::Point2;

const GO_BEHIND_BALL_DIST: f64 = 0.3;

/// Shoot the ball to the target_shooting_position inside the enemy goal
/// (before kicking he makes sure to be aligned with the enemy goal)
/// 
/// # Arguments
/// - `robot` : The robot that will shoot the ball
/// - `ball` : The ball
/// - `target_shooting_position` : The position inside the enemy goal where the robot will shoot the ball
/// - `world` : The current world state
/// 
/// # Returns
/// A `MoveTo` action that will make the robot shoot the ball to the target_shooting_position
pub fn shoot(
    robot: &Robot<AllyInfo>,
    ball: &Ball,
    target_shooting_position: &Point2<f64>,
    world: &World,
) -> MoveTo {
    let robot_position = robot.pose.position;
    let robot_direction = vectors::vector_from_angle(robot.pose.orientation);
    let ball_position = ball.position_2d();
    let robot_to_ball = ball_position - robot_position;
    let dot_with_ball = robot_direction.normalize().dot(&robot_to_ball.normalize());
    let dist_to_ball = robot_to_ball.norm();

    // Calculate the position behind the ball to prepare the shoot

            
    // Check if the shooting trajectory will score
    let robot_shooting_trajectory = Line::new(robot_position, robot_position + robot_to_ball * 100.);
    let shooting_trajectory_will_score = match robot_shooting_trajectory.intersection_segments(&world.geometry.enemy_goal.line) {
        Ok(_) => true,
        Err(_) => false,
    };

    let mut dribbler = if dist_to_ball < 1. {200.} else {0.};

    if shooting_trajectory_will_score && dot_with_ball > 0.75{
        let kick: Option<Kick> = if dist_to_ball < (world.geometry.robot_radius + world.geometry.ball_radius) {
            dribbler = 0.;
            Some(Kick::StraightKick {  power: 4. })
        }else {
            None
        };
        return MoveTo::new(ball_position, vectors::angle_to_point(robot_position,*target_shooting_position), dribbler,  true, kick, true);
    }

    let goal_to_ball = target_shooting_position - ball_position;
    let robot_to_ball = robot_position - ball_position;

    let line_goal_ball = Line::new(*target_shooting_position,ball_position);
    let mut correction_target = line_goal_ball.closest_point_on_line(&robot_position);

    if dot_with_ball < 0.5 || goal_to_ball.normalize().dot(&robot_to_ball.normalize()) > 0. {
        correction_target = ball_position + (ball_position - target_shooting_position).normalize() * GO_BEHIND_BALL_DIST
    }

    MoveTo::new(correction_target, vectors::angle_to_point(robot_position, *target_shooting_position), dribbler, false, None, true)
}