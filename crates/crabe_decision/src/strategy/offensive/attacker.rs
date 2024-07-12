use std::f64::consts::PI;
use crate::{action::ActionWrapper, strategy::actions::shoot};
use crate::message::MessageData;
use crate::strategy::Strategy;
use crabe_framework::data::tool::ToolData;
use crabe_framework::data::world::World;
use crabe_math::{shape::Line, vectors::{self, rotate_vector}};
use nalgebra::Point2;
/// The Attacker strategy is responsible for moving the robot to the ball and then try scoring a goal
pub struct Attacker {
    /// The id of the robot to move.
    id: u8,
    messages: Vec<MessageData>,
}

impl Attacker {
    /// Creates a new Attacker instance with the desired robot id.
    pub fn new(id: u8) -> Self {
        Self { id, messages: vec![]}
    }

    fn get_open_shoot_window(&self, shoot_start_position: &Point2<f64>, world: &World) -> Vec<Line> {
        let mut availables_targets: Vec<Line> = vec![world.geometry.enemy_goal.line.clone()];

        for enemy in world.enemies_bot.values() {
            let robot_to_enemy = enemy.pose.position - shoot_start_position;
            let perp = rotate_vector(robot_to_enemy.normalize(), PI/2.) * (world.geometry.robot_radius + world.geometry.ball_radius + 0.01);
            let dir_left_side = (enemy.pose.position + perp) - shoot_start_position;
            let dir_right_side = (enemy.pose.position - perp) - shoot_start_position;
            let to_left_side_enemy = Line::new(*shoot_start_position, shoot_start_position + dir_left_side * 100.);
            let to_right_side_enemy = Line::new(*shoot_start_position, shoot_start_position + dir_right_side * 100.);
            if let Ok(intersection_left) = to_left_side_enemy.intersection_segment_line(&world.geometry.enemy_goal.line) {
                if let Ok(intersection_right) = to_right_side_enemy.intersection_segment_line(&world.geometry.enemy_goal.line) {
                    // for each line ine availables_targets, cut off 
                    let mut new_targets: Vec<Line> = vec![];
                    for target_line in availables_targets {
                        let targets = target_line.cut_off_segment(&Line::new(intersection_left, intersection_right));
                        new_targets.extend(targets);
                    }
                    availables_targets = new_targets;
                }
            } 
        }
        return availables_targets;
    }
}

impl Strategy for Attacker {

    fn name(&self) -> &'static str {
        return "Attacker";
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

        // Get the Attacker robot, otherwise exit the function
        let robot = match world.allies_bot.get(&self.id) {
            Some(robot) => robot,
            None => return false,
        };
        let robot_position = robot.pose.position;
        let robot_direction = vectors::vector_from_angle(robot.pose.orientation);
        
        // Get the ball position, otherwise exit the function
        let ball = match &world.ball {
            Some(ball) => ball,
            None => return false,
        };

        let availables_targets = self.get_open_shoot_window(&ball.position_2d(), world);
        for target in &availables_targets {
            tools_data.annotations.add_line(target.start.to_string(), *target);
        }

        if availables_targets.len() == 0 {
            //need to pass the ball
            return false;
        }

        //grab longest target line norm
        let mut longest_target = &availables_targets[0];
        for target in &availables_targets {
            if target.norm() > longest_target.norm() {
                longest_target = &target;
            }
        }
        
        // Set the target shoot position to the center of the goal
        let target_shooting_position: Point2<f64> = longest_target.center();


        tools_data.annotations.add_point("robot_position".to_string(), target_shooting_position);


        action_wrapper.push(self.id, shoot(
            &robot,
            &ball,
            &target_shooting_position,
            world,
        ));

        false
    }

}
