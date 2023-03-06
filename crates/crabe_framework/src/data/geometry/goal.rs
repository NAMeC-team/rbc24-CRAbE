use nalgebra::Point2;
use serde::Serialize;
use uom::si::f32::Length;

// TODO : Document
#[derive(Serialize, Clone, Debug)]
pub struct Goal {
    pub width: Length,
    pub depth: Length,
    pub top_left_position: Point2<Length>,
}

// TODO : Implement some helper methods
