use crate::action::move_to::MoveTo;
use crate::action::ActionWrapper;
use crate::message::{AttackerMessage, Message, MessageData};
use crate::strategy::actions::intercept;
use crate::strategy::Strategy;
use crabe_framework::data::tool::ToolData;
use crabe_framework::data::world::World;
use crabe_math::shape::Line;
use crabe_math::vectors;

pub struct Receiver {
    id: u8,
    passer_id: u8,
    passing_trajectory: Line,
    messages: Vec<MessageData>,
}

impl Receiver {
    /// Creates a new Receiver instance with the desired robot id.
    pub fn new(id: u8, passer_id: u8, passing_trajectory: Line) -> Self {
        Self { id, passer_id, passing_trajectory, messages: vec![]}
    }
}

impl Strategy for Receiver {

    fn name(&self) -> &'static str {
        return "Receiver";
    }
    
    fn get_messages(&self) -> &Vec<MessageData>  {
        &self.messages
    }
    fn get_ids(&self) -> Vec<u8> {
        vec![self.id]
    }
    fn put_ids(&mut self, ids: Vec<u8>) {
        if ids.len() == 1{
            self.id = ids[0];
        }
    }

    /// # Arguments
    ///
    /// * world: The current state of the game world.
    /// * tools_data: A collection of external tools used by the strategy, such as a viewer.    
    /// * action_wrapper: An `ActionWrapper` instance used to issue actions to the robot.
    ///
    /// # Returns
    ///
    /// A boolean value indicating whether the strategy is finished or not.
    #[allow(unused_variables)]
    fn step(
        &mut self,
        world: &World,
        tools_data: &mut ToolData,
        action_wrapper: &mut ActionWrapper,
    ) -> bool {
        // Clean the action wrapper otherwise the previous commands will still have to be runned before the one he will calculate now
        action_wrapper.clear(self.id);

        // Get the Receiver robot, otherwise exit the function
        let robot = match world.allies_bot.get(&self.id) {
            Some(robot) => robot,
            None => return false,
        };
        let passer_robot = match world.allies_bot.get(&self.passer_id) {
            Some(robot) => robot,
            None => return false,
        };
        let robot_position = robot.pose.position;
        let passer_position =passer_robot.pose.position;
        let robot_direction = vectors::vector_from_angle(robot.pose.orientation);
        
        // Get the ball position, otherwise exit the function
        let ball = match &world.ball {
            Some(ball) => ball,
            None => return false,
        };
        tools_data.annotations.add_line("passing_trajectory".to_string(), self.passing_trajectory);

        if ball.velocity.norm() > 1. {
            action_wrapper.push(self.id, intercept(robot, ball));   
            return false;         
        }
        let interception_point = self.passing_trajectory.closest_point_on_segment(&robot_position);
        let mut dribbler = 0.;
        if (interception_point - ball.position_2d()).norm() < 0.2 {
            dribbler = 1.;
        }
        action_wrapper.push(self.id, MoveTo::new(interception_point, vectors::angle_to_point(robot_position,ball.position_2d()), dribbler,  false, None, true));
        

        false
    }

}
