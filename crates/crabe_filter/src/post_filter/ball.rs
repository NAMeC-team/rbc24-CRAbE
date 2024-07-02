use crate::data::FilterData;
use crate::post_filter::PostFilter;
use crabe_framework::data::world::World;

pub struct BallFilter;

impl PostFilter for BallFilter {
    fn step(&mut self, filter_data: &FilterData, world: &mut World) {
        let ball = filter_data.ball.as_ref().map(|ball| ball.data.clone());
        if ball.is_some() {
            world.ball = ball;
        }
    }
}
