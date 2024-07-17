use crate::{
    action::{move_to::MoveTo, ActionWrapper}, message::MessageData, strategy::Strategy
};
use crabe_framework::data::{
    output::Kick::StraightKick,
    tool::ToolData,
    world::World,
};
use nalgebra::{Matrix, Point2};

use crabe_math::{shape::Line, vectors::angle_to_point, shape::Circle};
use crate::utils::closest_bot_to_point;

const DISTANCE_FROM_ATTACKER:f64 = 0.5;
const INNACURACY:f64 = 0.01;
const DISTANCE_FROM_GOAL_TRAJECTORY:f64 = 0.25;
/// The BotMarking struct represents a strategy that commands a robot to move in a BotMarking shape
/// in a counter-clockwise. It is used for testing purposes.
pub struct BotMarking {
    /// The id of the robot to move.
    id: u8,
    messages: Vec<MessageData>,
    enemy_id: u8,
}

impl BotMarking {
    /// Creates a new BotMarking instance with the desired robot id.
    pub fn new(id: u8, enemy_id: u8) -> Self {
        Self { 
            id,
            messages: vec![],
            enemy_id,
        }
    }
}

impl Strategy for BotMarking {
    fn name(&self) -> &'static str {
        "BotMarking"
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

    #[allow(unused_variables)]
    fn step(
        &mut self,
        world: &World,
        tools_data: &mut ToolData,
        action_wrapper: &mut ActionWrapper,
        
    ) -> bool {
        action_wrapper.clear(self.id);
        let ball = match &world.ball {
            Some(b) => b,
            None => {
                return false;
            }
        };
        let ball_pos = ball.position_2d();

        let robot = &match world.allies_bot.get(&self.id) {
            Some(r) => r,
            None => {
                return false;
            }
        };
        let robot_pos = &robot.pose;

        let enemy = &match world.enemies_bot.get(&self.enemy_id) {
            Some(r) => r,
            None => {
                return false;
            }
        };
        let enemy_pos = &enemy.pose;

        let our_attacker =  &match closest_bot_to_point(world.allies_bot.values().collect(), ball_pos){
            Some(closest_ally) => closest_ally,
            None => {
                return false;
            }
        };

        let mut dribbler = 0.0;

        if robot.distance(&ball_pos) < 1. {
            dribbler = 1.0;
        }

        //ANGLE TO BALL
        let angle = angle_to_point(robot_pos.position, ball_pos);

        // VELOCITY CATCH
        let ball_velocity_trajectory = Line::new(ball_pos, ball_pos + ball.velocity.xy().normalize() * 100.);
        let intersect_goal = match world.geometry.enemy_goal.line.intersection_segments(&ball_velocity_trajectory){
            Ok(o) => true,
            Err(e) => false
        };
        if ball.velocity.norm() > 0.4 && (ball_velocity_trajectory.distance_to_point(&enemy_pos.position) < 1. || ball_velocity_trajectory.distance_to_point(&robot_pos.position) < 1.) && !intersect_goal {
            let target = ball_velocity_trajectory.closest_point_on_segment(&robot_pos.position);
            action_wrapper.push(self.id,  MoveTo::new(Point2::new(target.x, target.y), angle , dribbler , false , Some(StraightKick { power: 0.0 }), true ));
        } else {

            // VECTEUR BALL -> ENEMY
            let enemy_to_ball = ball_pos - enemy_pos.position;
            let enemy_ball_distance = enemy_to_ball.norm();
            let coef_distance_to_enemy: f64 = world.geometry.robot_radius + 0.2/enemy_ball_distance;
            let target = enemy_pos.position -  Point2::new(enemy_to_ball.x, enemy_to_ball.y)*(-coef_distance_to_enemy);
            let mut target_point = Point2::new(target.x, target.y);


            if our_attacker.id != self.id{
                if our_attacker.distance(&target_point) < DISTANCE_FROM_ATTACKER + world.geometry.robot_radius {
                    target_point = (target_point - (our_attacker.pose.position - target_point).normalize())*(DISTANCE_FROM_ATTACKER + world.geometry.robot_radius);;
                }
                let line_ball_handler_start_goal = Line::new(our_attacker.pose.position , world.geometry.enemy_goal.line.start);

                let line_ball_handler_end_goal = Line::new(our_attacker.pose.position , world.geometry.enemy_goal.line.end);


                let point_closest_start =line_ball_handler_start_goal.closest_point_on_segment(&target_point);
                let point_closest_end = line_ball_handler_end_goal.closest_point_on_segment(&target_point);

                let distance_start = (target_point - point_closest_start).norm();
                let distance_end = (target_point - point_closest_end).norm();

                let distance_start_end = (point_closest_start - point_closest_end).norm();

                if distance_start + distance_end-INNACURACY < distance_start_end && distance_start_end < distance_start + distance_end+INNACURACY {
                    if distance_start < distance_end {
                        target_point = Point2::new(target_point.x,target_point.y+DISTANCE_FROM_GOAL_TRAJECTORY);
                    } else {
                        target_point = Point2::new(target_point.x,target_point.y-DISTANCE_FROM_GOAL_TRAJECTORY);
                    }
                }

            }

            action_wrapper.push(self.id,  MoveTo::new(target_point, angle , dribbler , false , Some(StraightKick { power: 0.0 }), true ));
        }

        
        
        return false;

    }
    
}
