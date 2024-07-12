use crate::action::move_to::MoveTo;
use crate::action::ActionWrapper;
use crate::message::MessageData;
use crate::strategy::Strategy;
use crate::utils::closest_bot_to_point;
use crate::utils::constants::{DEFENDER1_ID, DEFENDER2_ID};

use crabe_framework::data::tool::ToolData;
use crabe_framework::data::world::World;
use crabe_math::vectors;
use nalgebra::Point2;

/// The PrepareGoalKick struct represents a strategy that commands the team to set in the PrepareGoalKick formation
#[derive(Default)]
pub struct PrepareGoalKick{
    ally: bool,
    messages: Vec<MessageData>,
    ids: Vec<u8>,
}

impl PrepareGoalKick{
    pub fn new(ally: bool) -> Self {
        Self {
            ally, 
            messages: vec![],
            ids: vec![],
        }
    }
}

impl Strategy for PrepareGoalKick{
    /// Executes the PrepareKickOffstrategy.
    ///
    /// This strategy commands all the robots to move in position for
    ///
    /// # Arguments
    ///
    /// * world: The current state of the game world.
    /// * tools_data: A collection of external tools used by the strategy, such as a viewer.    
    /// * action_wrapper: An `ActionWrapper` instance used to issue actions to the robot.
    ///
    /// # Returns
    ///
    /// A boolean value indicating whether the strategy is finished or not.
    
    fn name(&self) -> &'static str {
        "PrepareGoalKick"
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

        for robot in self.ids.iter() {
            action_wrapper.clear(*robot);
        }

        let ball_pos = match world.ball.clone() {
            None => {
                Point2::new(-3.0,0.0)
            }
            Some(ball) => {
                ball.position.xy()
            }
        };

        if self.ally {
            action_wrapper.push(0, MoveTo::new(Point2::new(-1., 2.), vectors::angle_to_point(Point2::new(-1., 2.), ball_pos), 0.0, false, None, false));
            action_wrapper.push(1, MoveTo::new(Point2::new(-0.0, 0.0), vectors::angle_to_point(Point2::new(-0.0, 0.0), ball_pos), 0.0, false, None, false));
            action_wrapper.push(2, MoveTo::new(Point2::new(-1., -2.), vectors::angle_to_point(Point2::new(-1., -2.), ball_pos), 0.0, false, None, false));
            action_wrapper.push(3, MoveTo::new(Point2::new(-3., 2.5), vectors::angle_to_point(Point2::new(-3., 2.5), ball_pos), 0.0, false, None, false));
            action_wrapper.push(4, MoveTo::new(Point2::new(-3., -2.5), vectors::angle_to_point(Point2::new(-3., -2.5), ball_pos), 0.0, false, None, false));
            action_wrapper.push(5, MoveTo::new(Point2::new(-4.0, 0.0), vectors::angle_to_point(Point2::new(-4.0, 0.0), ball_pos), 0.0, false, None, false));
            return false;
        } else {

            let enemy_goal = closest_bot_to_point(world.enemies_bot.values().collect(), ball_pos);

            if let Some(enemy_goal) = enemy_goal {
                let enemy_goal_pos = enemy_goal.pose.position;
                let enemy_goal_angle = vectors::angle_to_point(enemy_goal_pos, ball_pos);


                let ally_closest = closest_bot_to_point(world.allies_bot.values().collect(), ball_pos);

                if let Some(ally_closest) = ally_closest {
    
                    if ally_closest.id != DEFENDER1_ID && ally_closest.id != DEFENDER2_ID {
                        action_wrapper.push(ally_closest.id, MoveTo::new(Point2::new(ball_pos.x - 1., ball_pos.y), vectors::angle_to_point(Point2::new(ball_pos.x - 1., ball_pos.y), ball_pos), 0.0, false, None, false));
                    }
                    
                    action_wrapper.push(DEFENDER1_ID, MoveTo::new(Point2::new(-3., 0.2), vectors::angle_to_point(Point2::new(-3., 0.2), ball_pos), 0.0, false, None, false));
                    action_wrapper.push(DEFENDER2_ID, MoveTo::new(Point2::new(-3., -0.2), vectors::angle_to_point(Point2::new(-3., -0.2), ball_pos), 0.0, false, None, false));
                }
            }

        }
        false
    }
}
