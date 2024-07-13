use crate::data::FilterData;
use crate::post_filter::PostFilter;
use crabe_framework::data::world::{self, game_state::{GameState, RunningState}, Ball, TeamColor, World};
use crabe_decision::utils::closest_bot_to_point;

pub struct BallFilter;

const MIN_ACCELERATION_TO_SWITCH_POSSESSION: f64 = 1.;
const MIN_DISTANCE_DIFFERENCE_TO_SWITCH_POSSESSION: f64 = 0.1;
const MAX_DISTANCE_DIFFERENCE_TO_SWITCH_POSSESSION: f64 = 0.3;
const MAX_DIFFERENCE_VELOCITY_TO_SWITCH_POSSESSION: f64 = 0.1;
const DOT_DIFFERENCE_TO_SWITCH_POSSESSION: f64 = 0.75;


fn calculated_possession(ball: &mut Ball, world: &World) {
    let ball_world = match &world.ball {
        Some(b) => b,
        None => {
            ball.possession = None; return;}
    };
    ball.possession = ball_world.possession;
    let state = world.data.ref_orders.state;

    // ALLIES PART
    let bot_ally = match closest_bot_to_point(world.allies_bot.values().collect(), ball.position.xy()) {
        Some(bot) => bot,
        None => {ball.possession = None; return;}
    };
    let ally_color = world.team_color;

    // ENEMIES PART
    let bot_enemy = match closest_bot_to_point(world.enemies_bot.values().collect(), ball.position.xy()) {
        Some(bot) => bot,
        None => {ball.possession = None; return;}
    
    };
    let enemy_color = if ally_color == TeamColor::Yellow { TeamColor::Blue } else { TeamColor::Yellow };

    let ally_distance = bot_ally.distance(&ball.position.xy());
    let enemy_distance = bot_enemy.distance(&ball.position.xy());

    

    ///////////////////////////////////
    // CALCULATE POSSESSION BY BALL VELOCITY
    ///////////////////////////////////
    let ally_possession:bool = ally_distance<MAX_DISTANCE_DIFFERENCE_TO_SWITCH_POSSESSION 
                            && (bot_ally.velocity.linear - ball.velocity.xy()).norm() < MAX_DIFFERENCE_VELOCITY_TO_SWITCH_POSSESSION 
                            && ball.velocity.xy().dot(&bot_ally.velocity.linear) > DOT_DIFFERENCE_TO_SWITCH_POSSESSION;

    let enemy_possession:bool = enemy_distance<MAX_DISTANCE_DIFFERENCE_TO_SWITCH_POSSESSION
                            && (bot_enemy.velocity.linear - ball.velocity.xy()).norm() < MAX_DIFFERENCE_VELOCITY_TO_SWITCH_POSSESSION 
                            && ball.velocity.xy().dot(&bot_enemy.velocity.linear) > DOT_DIFFERENCE_TO_SWITCH_POSSESSION;
    
    if ally_possession && !enemy_possession{
        ball.possession = Some(ally_color);
    }
    else if enemy_possession && !ally_possession{
        ball.possession = Some(enemy_color);
    } else {
        ///////////////////////////////////
        // CALCULATE POSSESSION BY BALL ACCELERATION
        ///////////////////////////////////
        if ball.acceleration.norm() > MIN_ACCELERATION_TO_SWITCH_POSSESSION {

            // DETERMINE THE COLOR OF THE ROBOT THAT IS CLOSER TO THE BALL WHEN ACC IS HIGH
            if ally_distance + MIN_DISTANCE_DIFFERENCE_TO_SWITCH_POSSESSION < enemy_distance {
                ball.possession = Some(ally_color);
            } else {
                ball.possession = Some(enemy_color);
            }
        }
    }
    

    ///////////////
    // RE CALCULATE POSSESSION BY GAME STATE
    ///////////////
    if let GameState::Running(running_state) = state{
        match running_state {
            RunningState::FreeKick(val)
            | RunningState::KickOff(val)
            | RunningState::Penalty(val) => {
                ball.possession = Some(val);
            }
            _ => {}
        }
    }
}

impl PostFilter for BallFilter {
    fn step(&mut self, filter_data: &FilterData, world: &mut World) {
        let mut ball = filter_data.ball.data.clone();
        calculated_possession(&mut ball, &world);
        world.ball = Some(ball);
    }
}