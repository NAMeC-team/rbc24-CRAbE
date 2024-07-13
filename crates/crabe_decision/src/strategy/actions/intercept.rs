use crate::action::move_to::MoveTo;
use crabe_framework::data::world::{AllyInfo, Ball, Robot};
use crabe_math::shape::Line;
use crabe_math::vectors;


pub fn intercept(
    robot: &Robot<AllyInfo>,
    ball: &Ball,
) -> MoveTo {
    if ball.velocity.norm() < 0.4 {
        return MoveTo::new(ball.position_2d(), robot.pose.orientation, 0., false, None, true);
    }
    let ball_position = ball.position_2d();
    let trajectory = Line::new(ball_position, ball_position + ball.velocity.xy().normalize() * 100.);
    let target = trajectory.closest_point_on_line(&robot.pose.position);
    let mut dribbler = 0.;
    if robot.distance(&ball_position) < 0.2 {
        dribbler = 1.;
    }
    MoveTo::new(target, vectors::angle_to_point(robot.pose.position,ball_position), dribbler,  false, None, true)
}
