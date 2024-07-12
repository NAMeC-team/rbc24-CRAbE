use crate::action::move_to::MoveTo;
use crabe_framework::data::output::Kick;
use crabe_framework::data::world::{AllyInfo, Ball, Robot, World};
use crabe_math::shape::Line;
use crabe_math::vectors;
use nalgebra::Point2;

const GO_BEHIND_BALL_DIST: f64 = 0.3;

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
    let behind_ball_position = ball_position + (ball_position - target_shooting_position).normalize() * GO_BEHIND_BALL_DIST; 
            
    // Check if the shooting trajectory will score
    let robot_shooting_trajectory = Line::new(robot_position, robot_position + robot_to_ball * 100.);
    let shooting_trajectory_will_score = match robot_shooting_trajectory.intersection_segments(&world.geometry.enemy_goal.line) {
        Ok(_) => true,
        Err(_) => false,
    };

    if shooting_trajectory_will_score && dot_with_ball > 0.95{
        let kick: Option<Kick> = if dist_to_ball < (world.geometry.robot_radius + world.geometry.ball_radius + 0.002) { 
            Some(Kick::StraightKick {  power: 4. }) 
        }else {None};
        MoveTo::new(ball_position, vectors::angle_to_point(robot_position,*target_shooting_position), 1.,  true, kick, true)
    }else{
        MoveTo::new(behind_ball_position, vectors::angle_to_point(robot_position, *target_shooting_position), 0., false, None, true)
    }

}
