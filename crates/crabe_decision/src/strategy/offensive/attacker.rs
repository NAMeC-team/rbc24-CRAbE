use std::f64::consts::PI;
use crate::action::move_to::MoveTo;
use crate::strategy::actions::{intercept, pass};
use crate::utils::{get_first_angle_free_trajectory, navigation, object_in_bot_trajectory, KEEPER_ID};
use crate::{action::ActionWrapper, strategy::actions::shoot};
use crate::message::{AttackerMessage, MessageData};
use crate::strategy::Strategy;
use crabe_framework::data::tool::{self, ToolData};
use crabe_framework::data::world::{AllyInfo, Ball, Robot, World};
use crabe_math::shape::Circle;
use crabe_math::{shape::Line, vectors::{self, rotate_vector}};
use nalgebra::Point2;
use crate::message::Message;
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

    /// Find the best ally to pass the ball to
    fn pass_ball(&mut self, world: &World, allies: &Vec<&Robot<AllyInfo>>, robot: &Robot<AllyInfo>, ball: &Ball) -> MoveTo{
        let robot_position = robot.pose.position;
        let mut closest_ally = &allies[0];
        let mut max_window_length = 0.;
        for ally in allies {
            if object_in_bot_trajectory(world, self.id, ally.pose.position, false, false, true).len() > 0{
                continue;
            }
            let shoot_windows = get_open_shoot_window(&ally.pose.position, world);
            let total_length = shoot_windows_total_length(&shoot_windows);
            if total_length <= max_window_length {
                continue;
            }
            max_window_length = total_length;
            closest_ally = ally;
        }
        let passing_trajectory = Line::new(robot_position, robot_position+(closest_ally.pose.position - robot_position).normalize() * 100.);
        if ball.acceleration.norm() > 4.{
            self.messages.push(MessageData::new(Message::AttackerMessage(AttackerMessage::BallPassed(closest_ally.id)), self.id));
        }else{
            self.messages.push(MessageData::new(Message::AttackerMessage(AttackerMessage::WantToPassBallTo(closest_ally.id, passing_trajectory)), self.id));
        }
        
        pass(&robot, &closest_ally, &ball, world)
    }
}

fn shoot_windows_total_length(shoot_windows: &Vec<Line>) -> f64 {
    let mut total_length = 0.;
    for window in shoot_windows {
        total_length += window.norm();
    }
    return total_length;
}

fn get_open_shoot_window(shoot_start_position: &Point2<f64>, world: &World) -> Vec<Line> {
    let mut available_targets: Vec<Line> = vec![world.geometry.enemy_goal.line.clone()];

    for enemy in world.enemies_bot.values() {
        let robot_to_enemy = enemy.pose.position - shoot_start_position;
        let perp = rotate_vector(robot_to_enemy.normalize(), PI/2.) * (world.geometry.robot_radius + world.geometry.ball_radius + 0.01);
        let dir_left_side = (enemy.pose.position + perp) - shoot_start_position;
        let dir_right_side = (enemy.pose.position - perp) - shoot_start_position;
        let to_left_side_enemy = Line::new(*shoot_start_position, shoot_start_position + dir_left_side * 100.);
        let to_right_side_enemy = Line::new(*shoot_start_position, shoot_start_position + dir_right_side * 100.);
        let intersection_left = to_left_side_enemy.intersection_segment_line(&world.geometry.enemy_goal.line);
        let intersection_right = to_right_side_enemy.intersection_segment_line(&world.geometry.enemy_goal.line);
        

        available_targets = match (intersection_left, intersection_right) {
            (Ok(left), Ok(right)) => {
                let mut new_targets: Vec<Line> = vec![];
                for target_line in available_targets {
                    let targets = target_line.cut_off_segment(&Line::new( left, right));
                    new_targets.extend(targets);
                }
                new_targets
            },
            (Ok(left), _) => {
                let mut new_targets: Vec<Line> = vec![];
                for target_line in available_targets {
                    let targets = target_line.cut_off_segment(&Line::new( left, world.geometry.enemy_goal.line.end));
                    new_targets.extend(targets);
                }
                new_targets
            },
            (_, Ok(right)) => {
                let mut new_targets: Vec<Line> = vec![];
                for target_line in available_targets {
                    let targets = target_line.cut_off_segment(&Line::new( world.geometry.enemy_goal.line.start, right));
                    new_targets.extend(targets);
                }
                new_targets
            },
            _ => available_targets,
        };
    }
    return available_targets;
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
        self.messages.clear();

        // Get the Attacker robot, otherwise exit the function
        let robot = match world.allies_bot.get(&self.id) {
            Some(robot) => robot,
            None => return false,
        };
        let robot_position = robot.pose.position;

        // Get the ball position, otherwise exit the function
        let ball = match &world.ball {
            Some(ball) => ball,
            None => return false,
        };

        if ball.velocity.norm() > 1. && ball.velocity.x < 0. {
            action_wrapper.push(self.id, intercept(
                &robot,
                &ball,
            ));
            return false;
        }

        let available_targets = get_open_shoot_window(&ball.position_2d(), world);
        for target in &available_targets {
            tools_data.annotations.add_line(target.start.to_string(), *target);
        }

        let allies_in_positive_x : Vec<&Robot<AllyInfo>> = world.allies_bot.values().filter(|ally| ally.pose.position.x > 0. && ally.id != self.id && ally.id != KEEPER_ID).collect();
        if available_targets.len() == 0 && allies_in_positive_x.len() != 0  { // if we can't shoot, pass the ball
            action_wrapper.push(self.id, self.pass_ball(world, &allies_in_positive_x, robot, ball));
        } else {
            self.messages.push(MessageData::new(Message::AttackerMessage(AttackerMessage::NoNeedReceiver), self.id));

            // Grab largest target goal line space to shoot
            let mut longest_target = &available_targets[0];
            for target in &available_targets {
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
        }
        false
    }

}
