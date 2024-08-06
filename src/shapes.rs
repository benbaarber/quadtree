use nalgebra::{self as na, vector};

use crate::{Point, P2};

/// A trait for shapes that can be used to query the QuadTree. Shapes must be able to
/// provide their start and end points, their center point, and check if they contain
/// a point. They must also be able to check if they intersect with another shape.
pub trait Shape {
    /// Get the start point of the shape
    fn start(&self) -> P2;
    /// Get the end point of the shape
    fn end(&self) -> P2;
    /// Get the center point of the shape
    fn center(&self) -> P2;
    /// Check if the shape contains a point
    fn contains(&self, point: &P2) -> bool;
    /// Check if the shape shares any space with another shape
    fn intersects(&self, other: &Self) -> bool;

    /// Get the bounding rect of the shape
    fn rect(&self) -> Rect {
        Rect::new(self.start(), self.end())
    }

    /// Check if the shape fully contains a given rect
    fn contains_rect(&self, rect: &Rect) -> bool {
        self.contains(&rect.start()) && self.contains(&rect.end())
    }
}

impl<T: Point> Shape for T {
    fn start(&self) -> P2 {
        self.point()
    }

    fn end(&self) -> P2 {
        self.point()
    }

    fn center(&self) -> P2 {
        self.point()
    }

    fn contains(&self, point: &P2) -> bool {
        self.point() == *point
    }

    fn intersects(&self, other: &Self) -> bool {
        self.point() == other.point()
    }
}

/// Represents an axis-aligned rectangle defined by two points: the start and the end.
/// It is used to define boundaries for QuadTree nodes and provides utility functions
/// for geometric calculations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rect {
    start: P2,
    center: P2,
    end: P2,
}

impl Rect {
    /// Create a new rect with a start and end point
    pub fn new(start: P2, end: P2) -> Self {
        Self {
            start,
            center: na::center(&start, &end),
            end,
        }
    }

    /// Set the start point of the rect
    pub fn set_start(&mut self, start: P2) {
        self.start = start;
        self.center = na::center(&self.start, &self.end);
    }

    /// Set the end point of the rect
    pub fn set_end(&mut self, end: P2) {
        self.end = end;
        self.center = na::center(&self.start, &self.end);
    }

    /// Quarter the rect to produce four smaller rects
    pub fn quarter(&self) -> [Self; 4] {
        let &Rect { start, center, end } = self;
        let diff = center - start;
        let diff_x = na::vector![diff.x, 0.];
        let diff_y = na::vector![0., diff.y];

        [
            Rect::new(start, center),
            Rect::new(start + diff_x, center + diff_x),
            Rect::new(start + diff_y, center + diff_y),
            Rect::new(center, end),
        ]
    }
}

impl Shape for Rect {
    fn start(&self) -> P2 {
        self.start
    }

    fn end(&self) -> P2 {
        self.end
    }

    fn center(&self) -> P2 {
        self.center
    }

    fn contains(&self, point: &P2) -> bool {
        *point >= self.start && *point <= self.end
    }

    fn intersects(&self, other: &Self) -> bool {
        !(self.end.x < other.start.x
            || self.start.x > other.end.x
            || self.end.y < other.start.y
            || self.start.y > other.end.y)
    }

    fn rect(&self) -> Rect {
        *self
    }
}

/// Represents a circle defined by a center point and radius. Provides utility functions
/// for geometric calculations, particularly for interactions with QuadTree.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle {
    center: P2,
    radius: f64,
    start: P2,
    end: P2,
}

impl Circle {
    /// Create a new circle with a center point and radius
    pub fn new(center: P2, radius: f64) -> Self {
        let v = vector![radius, radius];
        let start = center - v;
        let end = center + v;
        Self {
            center,
            radius,
            start,
            end,
        }
    }

    fn update_bounds(&mut self) {
        let v = vector![self.radius, self.radius];
        self.start = self.center - v;
        self.end = self.center + v;
    }

    /// Set the center point of the circle
    pub fn set_center(&mut self, center: P2) {
        self.center = center;
        self.update_bounds();
    }

    /// Set the radius of the circle
    pub fn set_radius(&mut self, radius: f64) {
        self.radius = radius;
        self.update_bounds();
    }
}

impl Shape for Circle {
    fn start(&self) -> P2 {
        self.start
    }

    fn end(&self) -> P2 {
        self.end
    }

    fn center(&self) -> P2 {
        self.center
    }

    fn contains(&self, point: &P2) -> bool {
        na::distance(&self.center, point) <= self.radius
    }

    fn intersects(&self, other: &Self) -> bool {
        na::distance(&self.center, &other.center) <= self.radius + other.radius
    }
}

#[cfg(test)]
mod tests {
    use crate::util::tests::{make_circle, make_rect};
    use nalgebra::point;

    use super::*;

    #[test]
    fn rect_properties() {
        let rect = make_rect(0.0, 0.0, 10.0, 10.0);
        assert_eq!(
            rect.start(),
            point![0.0, 0.0],
            "Start should be at (0.0, 0.0)"
        );
        assert_eq!(
            rect.end(),
            point![10.0, 10.0],
            "End should be at (10.0, 10.0)"
        );
        assert_eq!(
            rect.center(),
            point![5.0, 5.0],
            "Center should be at (5.0, 5.0)"
        );
    }

    #[test]
    fn rect_contains_point() {
        let rect = make_rect(0.0, 0.0, 10.0, 10.0);
        assert!(
            rect.contains(&point![5.0, 5.0]),
            "Rect should contain point (5.0, 5.0)"
        );
        assert!(
            !rect.contains(&point![-1.0, 5.0]),
            "Rect should not contain point (-1.0, 5.0)"
        );
        assert!(
            rect.contains(&point![0.0, 0.0]),
            "Rect should contain its start point (0.0, 0.0)"
        );
        assert!(
            rect.contains(&point![10.0, 10.0]),
            "Rect should contain its end point (10.0, 10.0)"
        );
    }

    #[test]
    fn rect_intersects_with_another_rect() {
        let rect1 = make_rect(0.0, 0.0, 10.0, 10.0);
        let rect2 = make_rect(5.0, 5.0, 15.0, 15.0);
        assert!(
            rect1.intersects(&rect2),
            "Rect1 should intersect with Rect2"
        );

        let rect3 = make_rect(10.0, 10.0, 20.0, 20.0);
        assert!(
            rect1.intersects(&rect3),
            "Rect1 should touch Rect3 at the edge, counting as intersect"
        );

        let rect4 = make_rect(11.0, 11.0, 21.0, 21.0);
        assert!(
            !rect1.intersects(&rect4),
            "Rect1 should not intersect with Rect4"
        );

        let rect5 = make_rect(-5.0, -5.0, -1.0, -1.0);
        assert!(
            !rect1.intersects(&rect5),
            "Rect1 should not intersect with Rect5"
        );

        let rect6 = make_rect(3.0, 3.0, 7.0, 7.0);
        assert!(
            rect1.intersects(&rect6),
            "Rect6 is entirely inside Rect1, should intersect"
        );

        let rect7 = make_rect(-10.0, 0.0, -1.0, 10.0);
        assert!(
            !rect1.intersects(&rect7),
            "Rect1 should not intersect with Rect7 on the left"
        );

        let rect8 = make_rect(0.0, -10.0, 10.0, -1.0);
        assert!(
            !rect1.intersects(&rect8),
            "Rect1 should not intersect with Rect8 below it"
        );

        let rect9 = make_rect(0.0, 10.0, 10.0, 20.0);
        assert!(
            rect1.intersects(&rect9),
            "Rect1 should touch Rect9 at the top, counting as intersect"
        );

        let rect10 = make_rect(10.0, 0.0, 20.0, 10.0);
        assert!(
            rect1.intersects(&rect10),
            "Rect1 should touch Rect10 on the right, counting as intersect"
        );

        let rect11 = make_rect(5.0, -5.0, 15.0, 5.0);
        assert!(
            rect1.intersects(&rect11),
            "Rect11 should intersect with the bottom part of Rect1"
        );
    }

    #[test]
    fn rect_contains_another_rect() {
        let outer_rect = make_rect(0.0, 0.0, 10.0, 10.0);
        let inner_rect = make_rect(1.0, 1.0, 9.0, 9.0);
        assert!(
            outer_rect.contains_rect(&inner_rect),
            "Outer rect should contain inner rect completely"
        );

        let outer_rect = make_rect(0.0, 0.0, 10.0, 10.0);
        let overlapping_rect = make_rect(5.0, 5.0, 15.0, 15.0);
        assert!(
            !outer_rect.contains_rect(&overlapping_rect),
            "Outer rect should not contain overlapping rect"
        );
    }

    #[test]
    fn quartering_rect() {
        let rect = make_rect(0.0, 0.0, 10.0, 10.0);
        let quarters = rect.quarter();
        // Ensure each quarter is correctly sized and positioned
        assert_eq!(
            quarters[0],
            make_rect(0.0, 0.0, 5.0, 5.0),
            "Top-left quarter should match expected dimensions"
        );
        assert_eq!(
            quarters[1],
            make_rect(5.0, 0.0, 10.0, 5.0),
            "Top-right quarter should match expected dimensions"
        );
        assert_eq!(
            quarters[2],
            make_rect(0.0, 5.0, 5.0, 10.0),
            "Bottom-left quarter should match expected dimensions"
        );
        assert_eq!(
            quarters[3],
            make_rect(5.0, 5.0, 10.0, 10.0),
            "Bottom-right quarter should match expected dimensions"
        );
    }

    #[test]
    fn circle_properties_and_bounds() {
        let circle = make_circle(5.0, 5.0, 5.0);
        assert_eq!(
            circle.center(),
            point![5.0, 5.0],
            "Center should be at (5.0, 5.0)"
        );
        assert_eq!(circle.radius, 5.0, "Radius should be 5.0");
        assert_eq!(
            circle.start(),
            point![0.0, 0.0],
            "Start should be at (0.0, 0.0)"
        );
        assert_eq!(
            circle.end(),
            point![10.0, 10.0],
            "End should be at (10.0, 10.0)"
        );
    }

    #[test]
    fn circle_contains_point() {
        let circle = make_circle(5.0, 5.0, 5.0);
        assert!(
            circle.contains(&point![5.0, 5.0]),
            "Circle should contain its center point"
        );
        assert!(
            circle.contains(&point![0.0, 5.0]),
            "Circle should contain point on its perimeter"
        );
        assert!(
            !circle.contains(&point![0.0, 0.0]),
            "Circle should not contain points outside its boundary"
        );
    }

    #[test]
    fn circle_intersects_another_circle() {
        let circle1 = make_circle(5.0, 5.0, 5.0);
        let circle2 = make_circle(10.0, 5.0, 5.0);
        let circle3 = make_circle(20.0, 5.0, 5.0);

        assert!(
            circle1.intersects(&circle2),
            "Circle1 should intersect Circle2"
        );
        assert!(
            !circle1.intersects(&circle3),
            "Circle1 should not intersect Circle3"
        );
    }

    #[test]
    fn circle_contains_rect() {
        let circle = make_circle(5.0, 5.0, 5.0);
        let rect = make_rect(4.0, 4.0, 6.0, 6.0);
        assert!(circle.contains_rect(&rect), "Circle should contain rect");

        let circle = make_circle(5.0, 5.0, 5.0);
        let rect = make_rect(5.0, 5.0, 10.0, 10.0);
        assert!(
            !circle.contains_rect(&rect),
            "Circle should not contain overlapping rect"
        );
    }
}
