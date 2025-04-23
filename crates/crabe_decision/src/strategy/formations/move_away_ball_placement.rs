use std::backtrace;

use crate::action::order_raw::RawOrder;
use crate::action::ActionWrapper;
use crate::message::Message;
use crate::message::MessageData;
use crate::strategy::basics::move_away;
use crate::strategy::Strategy;
use crabe_framework::data::tool::ToolData;
use crabe_framework::data::world::World;
use crabe_math::shape::Line;
use nalgebra::Point2;
use crabe_framework::data::output::Command;


const CHECK_DISTANCE: f64 = 0.1;

/// Strategy to stop the robots (sending Command with 0 movements)
#[derive(Default)]
pub struct MoveAwayBallPlacement {
    ids: Vec<u8>,
    target_ball_placement: Point2<f64>,
    messages: Vec<MessageData>,
}

impl MoveAwayBallPlacement {
    /// Creates a new MoveAwayBallPlacement instance with the desired robot id.
    pub fn new(ids: Vec<u8>, target_ball_placement: Point2<f64>) -> Self {
        Self {
            ids,
            target_ball_placement,
            messages: vec![],
        }
    }
}

impl Strategy for MoveAwayBallPlacement {
    fn name(&self) -> &'static str {
        "MoveAwayBallPlacement"
    }

    fn get_messages(&self) -> &Vec<MessageData> {
        &self.messages
    }
    fn get_ids(&self) -> Vec<u8> {
        self.ids.clone()
    }
    fn put_ids(&mut self, ids: Vec<u8>) {
        self.ids = ids;
    }
    #[allow(unused_variables)]
    fn step(
        &mut self,
        world: &World,
        tools_data: &mut ToolData,
        action_wrapper: &mut ActionWrapper,
    ) -> bool {
        self.messages.clear();
        let ball = if let Some(ball) = &world.ball {
            ball
        } else {
            return false;
        };
        let dir = ball.position_2d() - self.target_ball_placement;
        let line_trajectory = Line::new(ball.position_2d(), self.target_ball_placement);
        self.ids.iter().enumerate().for_each(|(_, id)| {
            let robot = if let Some(robot) = world.allies_bot.get(&id) {
                robot
            } else {
                return;
            };
            let closest_point = line_trajectory.closest_point_on_segment(&robot.pose.position);
            let distance = (closest_point - robot.pose.position).norm();
            action_wrapper.clear(robot.id);
            match move_away(robot.pose.position, closest_point, world, 1.){
                Some(m) => {
                    action_wrapper.push(
                        robot.id,
                        m,
                    );
                },
                None => {},
            };
        });
        false
    }
}
