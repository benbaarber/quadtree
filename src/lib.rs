mod quadtree;
pub mod shapes;
mod util;

use nalgebra::Point2;
pub use quadtree::QuadTree;

type P2 = Point2<f64>;

/// Trait for getting a 2d point position of data stored in the [`QuadTree`]
pub trait Point {
    /// Get 2d point position
    fn point(&self) -> P2;
}

impl Point for P2 {
    fn point(&self) -> P2 {
        *self
    }
}
