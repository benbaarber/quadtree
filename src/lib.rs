#![no_std]

mod quadtree;
pub mod shapes;
mod util;

pub use glam::{Vec2, vec2};
pub use quadtree::Quadtree;
pub use quadtree::barnes_hut::{BHQuadtree, WeightedPoint};

/// Trait for getting a 2d point position of data stored in the [`Quadtree`]
pub trait Point {
    /// Get 2d point position
    fn point(&self) -> Vec2;
}

impl Point for Vec2 {
    fn point(&self) -> Vec2 {
        *self
    }
}
