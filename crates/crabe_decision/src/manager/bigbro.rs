use std::vec;

use crate::action::ActionWrapper;
use crate::manager::Manager;
use crate::message::Message;
use crate::message::MessageData;
use crate::strategy::offensive::Attacker;
use crate::strategy::testing::GoLeft;
use crate::strategy::testing::GoRight;
use crate::strategy::defensive::{GoalKeeper, BotMarking, BotContesting};
use crate::strategy::Strategy;
use crate::utils::closest_bots_to_point;
use crate::utils::constants;
use crate::utils::get_enemy_keeper_id;
use clap::builder::Str;
use crabe_framework::data::tool::ToolData;
use crabe_framework::data::world::{AllyInfo, Ball, EnemyInfo, Robot, World, Team};

/// The `BigBro` struct represents a decision manager that executes strategies BigBroly
/// added to its list.
/// It's used for testing individual strategies only and not meant to be used during an actual game.
///
/// To add a strategy, simply create a new instance of the desired strategy and add it to the
/// `strategies` field in the `new()` method of the `BigBro` struct.
#[derive(Default)]
pub struct BigBro {
    strategies: Vec<Box<dyn Strategy>>,
}

impl BigBro {
    /// Creates a new `BigBro` instance with the desired strategies to test.
    pub fn new() -> Self {
        Self {
            strategies: vec![
                Box::new(GoalKeeper::new(constants::KEEPER_ID)),
            ],
        }
    }

    /// Moves a bot from its current strategy to an existing strategy.
    ///
    /// # Arguments
    /// - `bot_id`: The id of the bot to move.
    /// - `strategy_index`: The index of the strategy (in the strategies list) to move the bot to.
    pub fn move_bot_to_existing_strategy(&mut self, bot_id: u8, strategy_index: usize) {
        let mut new_strategy_ids = self.strategies[strategy_index].as_ref().get_ids();
        new_strategy_ids.push(bot_id);
        self.strategies[strategy_index].put_ids(new_strategy_ids);
        if let Some(bot_current_strategy_index) = self
            .strategies
            .iter()
            .position(|s| s.get_ids().contains(&bot_id)){

                let mut current_strategy_ids = self.strategies[bot_current_strategy_index]
                    .as_ref()
                    .get_ids();
                if current_strategy_ids.len() == 1 {
                    self.strategies.remove(bot_current_strategy_index);
                } else {
                    current_strategy_ids.retain(|&id| id != bot_id);
                    self.strategies[bot_current_strategy_index].put_ids(current_strategy_ids);
                }
        }
    }

    pub fn move_bot_to_new_strategy(&mut self, bot_id: u8, strategy: Box<dyn Strategy>) {
        if let Some(current_strategy_index) = self
            .strategies
            .iter()
            .position(|s| s.get_ids().contains(&bot_id)){
                let mut ids = self.strategies[current_strategy_index].as_ref().get_ids();
                let index_of_bot_in_slot_ids = ids.iter().position(|x| x == &bot_id).unwrap();
                ids.remove(index_of_bot_in_slot_ids);
                if ids.len() == 0 {
                    //if the bot was the alone in this strategy, we can replace it
                    self.strategies[current_strategy_index] = strategy;
                } else {
                    self.strategies[current_strategy_index].put_ids(ids);
                    self.strategies.push(strategy);
                }
        } else {
            self.strategies.push(strategy);
        }

    }

    /// Processes the messages received from the strategies and updates the strategies accordingly.
    ///
    /// # Arguments
    /// - `messages`: A list of `MessageData` instances containing the messages received from the strategies.
    pub fn process_messages(&mut self, messages: Vec<MessageData>) {
        messages.iter().for_each(|m| {
            match m.message {
                Message::WantToGoRight => {
                    let strategy = Box::new(GoRight::new(m.id));
                    self.move_bot_to_new_strategy(m.id, strategy);
                }
                Message::WantToGoLeft => {
                    let strategy = Box::new(GoLeft::new(m.id));
                    self.move_bot_to_new_strategy(m.id, strategy);
                }
                Message::WantToBeAligned => {
                    //find strategy index with name "Aligned"
                    let strategy_index = self
                        .strategies
                        .iter()
                        .position(|s| s.name() == "Aligned")
                        .unwrap();
                    self.move_bot_to_existing_strategy(m.id, strategy_index);
                }
                _ => {}
            }
        });
    }

    /// Get the robot current strategy.
    /// 
    /// # Arguments
    /// - `bot_id`: The id of the robot.
    ///     
    /// # Returns
    /// The strategy of the robot.
    pub fn get_bot_current_strategy(&self, bot_id: u8) -> Option<&Box<dyn Strategy>> {
        if let Some(strategy) = self.strategies.iter().find(|s| s.get_ids().contains(&bot_id)){
            return Some(strategy);
        }
        None
    }

    pub fn attribute_strategies(&mut self, world: &World, ball: &Ball) {
        //get robots in the defense wall  strategy
        let allies_in_defense_wall: Vec<u8> = if let Some(defense_wall_strategy) = self.strategies.iter().find(|s| s.name() == "DefenseWall"){
            defense_wall_strategy.get_ids()
        }else{
            vec![]
        };

        let allies_defensor = vec![allies_in_defense_wall, vec![constants::KEEPER_ID]].concat();

        let closest_allies_to_ball = closest_bots_to_point(world.allies_bot.values().collect(), ball.position_2d());
        let closest_attackers_allies_to_ball = filter_robots_not_in_ids(closest_allies_to_ball, &allies_defensor);
        
        // if there's already an attacker, we check if we need to change it 
        if let Some(attacker_strategy) = self.strategies.iter().find(|s| s.name() == "Attacker"){
            let attacker_id = attacker_strategy.get_ids()[0];
            if let Some(attacker_robot) = world.allies_bot.get(&attacker_id){
                let attacker_dist_to_ball = attacker_robot.distance(&ball.position_2d());
                if !(closest_attackers_allies_to_ball.len() > 0 && closest_attackers_allies_to_ball[0].id != attacker_id && attacker_dist_to_ball > 1.2) {
                    let allies_markers = filter_robots_not_in_ids(closest_attackers_allies_to_ball, &vec![attacker_id]);
                    self.attribute_marking_bots(world, allies_markers);
                    return;
                }
            }
        }
        if closest_attackers_allies_to_ball.len() > 0 {
            let new_attacker_id = closest_attackers_allies_to_ball[0].id;
            let strategy = Box::new(Attacker::new(new_attacker_id));
            self.move_bot_to_new_strategy(new_attacker_id, strategy);
            let allies_markers = filter_robots_not_in_ids(closest_attackers_allies_to_ball, &vec![new_attacker_id]);
            self.attribute_marking_bots(world, allies_markers);
        }
    }

    pub fn attribute_marking_bots(&mut self, world: &World, allies_markers: Vec<&Robot<AllyInfo>>) {
        let mut marked_bot = vec![];
        for ally_attacking in allies_markers {
            let closest_enemy_to_robot = closest_bots_to_point(world.enemies_bot.values().collect(), ally_attacking.pose.position);
            let markable_enemies = filter_robots_not_in_ids(closest_enemy_to_robot, &marked_bot);
            if markable_enemies.len() == 0 {
                continue;
            }
            marked_bot.push(markable_enemies[0].id);
            let strategy = Box::new(BotMarking::new(ally_attacking.id, markable_enemies[0].id));
            self.move_bot_to_new_strategy(ally_attacking.id, strategy);
        }
    }
}

// fn team_possessing_ball(ball: &Ball, allies: &Robot<AllyInfo>, enemies: &Robot<EnemyInfo>) -> Team {
//     let ball_pos = if 
// }

/// Filter robots by ids.
/// 
/// # Arguments
/// - `robots`: The list of robots to filter.
/// - `ids`: The list of ids to keep.
/// 
/// # Returns
/// A list of robots that are in the ids list.
pub fn filter_robots_in_ids<'a, T>(robots: Vec<&'a Robot<T>>, ids: &Vec<u8>) -> Vec<&'a Robot<T>> {
    robots.into_iter().filter(|r| ids.contains(&r.id)).collect()
}

/// Remove robots with id in the ids list.
/// 
/// # Arguments
/// - `robots`: The list of robots to filter.
/// - `ids`: The list of ids to keep.
/// 
/// # Returns
/// A list of robots that are not in the ids list.
pub fn filter_robots_not_in_ids<'a, T>(robots: Vec<&'a Robot<T>>, ids: &Vec<u8>) -> Vec<&'a Robot<T>> {
    robots.into_iter().filter(|r| !ids.contains(&r.id)).collect()
}

impl Manager for BigBro {
    /// Executes the list of strategies on the given `World` data, `ToolData`, and `ActionWrapper`.
    fn step(
        &mut self,
        world: &World,
        tools_data: &mut ToolData,
        action_wrapper: &mut ActionWrapper,
    ) {
        let enemy_keeper_id: u8 = get_enemy_keeper_id(world);
        if let Some(ball) = &world.ball{
            self.attribute_strategies(world, ball);
        }



        // mailbox to grab the messages
        // (we can't iter the strategies and modify them at the same time so we need to collect the messages first and then process them)
        let mut messages: Vec<MessageData> = vec![];

        // grab all the messages from the strategies
        self.strategies.iter().for_each(|s| {
            messages.extend(s.get_messages().clone());
        });

        // process the messages
        self.process_messages(messages);

        // execute the strategies
        self.strategies.iter_mut().for_each(|s| {
            s.step(world, tools_data, action_wrapper);
        });
    }
}
