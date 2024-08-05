use nalgebra::{self as na, vector, Point2};

/// Trait for getting a 2d point position of data stored in the [`QuadTree`]
pub trait Point {
    /// Get 2d point position
    fn point(&self) -> Point2<f64>;
}

impl Point for Point2<f64> {
    fn point(&self) -> Point2<f64> {
        *self
    }
}

/// Represents an axis-aligned rectangle defined by two points: the start and the end.
/// It is used to define boundaries for QuadTree nodes and provides utility functions
/// for geometric calculations.
#[derive(Clone, Copy, Debug)]
pub struct Rect {
    start: Point2<f64>,
    center: Point2<f64>,
    end: Point2<f64>,
}

impl Rect {
    /// Create a new rect with a start and end point
    pub fn new(start: Point2<f64>, end: Point2<f64>) -> Self {
        Self {
            start,
            center: na::center(&start, &end),
            end,
        }
    }

    /// Get the start point of the rect
    pub fn start(&self) -> Point2<f64> {
        self.start
    }

    /// Get the end point of the rect
    pub fn end(&self) -> Point2<f64> {
        self.end
    }

    /// Get the center point of the rect
    pub fn center(&self) -> Point2<f64> {
        self.center
    }

    /// Set the start point of the rect
    pub fn set_start(&mut self, start: Point2<f64>) {
        self.start = start;
        self.center = na::center(&self.start, &self.end);
    }

    /// Set the end point of the rect
    pub fn set_end(&mut self, end: Point2<f64>) {
        self.end = end;
        self.center = na::center(&self.start, &self.end);
    }

    /// Check if a point exists within the rect
    pub fn contains(&self, point: &Point2<f64>) -> bool {
        *point >= self.start && *point <= self.end
    }

    /// Check if the rect shares any space with another rect
    pub fn intersects(&self, other: &Rect) -> bool {
        self.contains(&other.start)
            || self.contains(&other.end)
            || other.contains(&self.start)
            || other.contains(&self.end)
    }
}

/// Represents a circle defined by a center point and radius. Provides utility functions
/// for geometric calculations, particularly for interactions with QuadTree.
#[derive(Clone, Copy, Debug)]
pub struct Circle {
    center: Point2<f64>,
    radius: f64,
    start: Point2<f64>,
    end: Point2<f64>,
}

impl Circle {
    /// Create a new circle with a center point and radius
    pub fn new(center: Point2<f64>, radius: f64) -> Self {
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

    /// Get the center point of the circle
    pub fn center(&self) -> Point2<f64> {
        self.center
    }

    /// Get the radius of the circle
    pub fn radius(&self) -> f64 {
        self.radius
    }

    fn update_bounds(&mut self) {
        let v = vector![self.radius, self.radius];
        self.start = self.center - v;
        self.end = self.center + v;
    }

    /// Set the center point of the circle
    pub fn set_center(&mut self, center: Point2<f64>) {
        self.center = center;
        self.update_bounds();
    }

    /// Set the radius of the circle
    pub fn set_radius(&mut self, radius: f64) {
        self.radius = radius;
        self.update_bounds();
    }

    /// Check if a point exists within the circle
    pub fn contains(&self, point: &Point2<f64>) -> bool {
        na::distance(&self.center, point) <= self.radius
    }

    /// Inscribe the circle in a square and return the square
    pub fn inscribe(&self) -> Rect {
        Rect::new(self.start, self.end)
    }
}

/// A generic QuadTree implementation for spatial indexing of 2D points.
/// It supports insertion and query operations within rectangular and circular bounds.
///
/// ## Variants
/// - `Internal`: Contains children nodes and represents a subdivided area.
/// - `External`: Contains a single data item and represents a leaf node.
/// - `Empty`: Represents an empty area without any data.
#[derive(Debug)]
pub enum QuadTree<T> {
    Internal {
        boundary: Rect,
        children: [Box<Self>; 4],
    },
    External {
        boundary: Rect,
        item: T,
    },
    Empty {
        boundary: Rect,
    },
}

impl<T: Point + Clone> QuadTree<T> {
    /// Create a new empty quadtree with a boundary rect
    pub fn new(boundary: Rect) -> Self {
        Self::Empty { boundary }
    }

    /// Insert a point into the quadtree
    ///
    /// **Returns** a boolean value indicating if the item was inserted successfully
    pub fn insert(&mut self, item: &T) -> bool {
        let point = item.point();
        match self {
            &mut Self::Empty { boundary } => {
                if !boundary.contains(&point) {
                    return false;
                }

                *self = Self::External {
                    boundary,
                    item: item.clone(),
                };
                true
            }
            &mut Self::External {
                boundary,
                item: ref existing_item,
            } => {
                if !boundary.contains(&point) {
                    return false;
                }

                let mut children = self.subdivide();
                let inserted_existing = children
                    .iter_mut()
                    .map(|c| c.insert(&existing_item))
                    .any(|i| i);
                let inserted_new = children.iter_mut().any(|c| c.insert(item));
                if inserted_existing && inserted_new {
                    *self = Self::Internal { boundary, children };
                    true
                } else {
                    false
                }
            }
            Self::Internal { boundary, children } => {
                if !boundary.contains(&point) {
                    return false;
                }

                children.iter_mut().any(|c| c.insert(item))
            }
        }
    }

    /// Queries the QuadTree for items within a specified rectangular area.
    /// This method populates a passed mutable vector with all found items.
    pub fn query_rectangular(&self, rect: &Rect, results: &mut Vec<T>) {
        match self {
            Self::External { item, .. } => {
                if rect.contains(&item.point()) {
                    results.push(item.clone());
                }
            }
            Self::Internal { boundary, children } => {
                if boundary.intersects(rect) {
                    for c in children {
                        c.query_rectangular(rect, results)
                    }
                }
            }
            _ => (),
        }
    }

    /// Queries the QuadTree for items within a specified circular area.
    /// This method populates a passed mutable vector with all found items.
    pub fn query_circular(&self, circle: &Circle, results: &mut Vec<T>) {
        match self {
            Self::External { item, .. } => {
                if circle.contains(&item.point()) {
                    results.push(item.clone());
                }
            }
            Self::Internal { boundary, children } => {
                if boundary.intersects(&circle.inscribe()) {
                    for c in children {
                        c.query_circular(circle, results)
                    }
                }
            }
            _ => (),
        }
    }

    /// Return the point at the center of the boundary
    pub fn center(&self) -> &Point2<f64> {
        &self.boundary().center
    }

    /// Get the boundary rect out of the enum
    pub fn boundary(&self) -> &Rect {
        match self {
            Self::Empty { boundary } => boundary,
            Self::External { boundary, .. } => boundary,
            Self::Internal { boundary, .. } => boundary,
        }
    }

    /// Chop the node into four quarters and return the new subnodes
    fn subdivide(&self) -> [Box<Self>; 4] {
        let &Rect { start, center, end } = self.boundary();
        let diff = center - start;
        let diff_x = na::vector![diff.x, 0.];
        let diff_y = na::vector![0., diff.y];

        [
            Box::new(Self::Empty {
                boundary: Rect::new(start, center),
            }),
            Box::new(Self::Empty {
                boundary: Rect::new(start + diff_x, center + diff_x),
            }),
            Box::new(Self::Empty {
                boundary: Rect::new(start + diff_y, center + diff_y),
            }),
            Box::new(Self::Empty {
                boundary: Rect::new(center, end),
            }),
        ]
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::point;

    use super::*;

    #[derive(Clone, Debug)]
    struct TestPoint {
        pos: Point2<f64>,
    }

    impl TestPoint {
        fn new(x: f64, y: f64) -> Self {
            Self { pos: point![x, y] }
        }
    }

    impl Point for TestPoint {
        fn point(&self) -> Point2<f64> {
            self.pos
        }
    }

    fn make_rect(x1: f64, y1: f64, x2: f64, y2: f64) -> Rect {
        Rect::new(point![x1, y1], point![x2, y2])
    }

    fn make_circle(x: f64, y: f64, r: f64) -> Circle {
        Circle::new(point![x, y], r)
    }

    #[test]
    fn rect_contains_point() {
        let rect = make_rect(0.0, 0.0, 10.0, 10.0);
        assert!(
            rect.contains(&Point2::new(5.0, 5.0)),
            "Point should be within the rect"
        );
        assert!(
            !rect.contains(&Point2::new(-1.0, 5.0)),
            "Point should be outside the rect"
        );
        assert!(
            !rect.contains(&Point2::new(5.0, 11.0)),
            "Point should be outside the rect"
        );
        assert!(
            rect.contains(&Point2::new(0.0, 0.0)),
            "Point on the boundary should be within the rect"
        );
        assert!(
            rect.contains(&Point2::new(10.0, 10.0)),
            "Point on the boundary should be within the rect"
        );
    }

    #[test]
    fn rect_intersects_with_another_rect() {
        let rect1 = make_rect(0.0, 0.0, 10.0, 10.0);
        let rect2 = make_rect(5.0, 5.0, 15.0, 15.0);
        let rect3 = make_rect(10.0, 10.0, 20.0, 20.0);
        let rect4 = make_rect(-5.0, -5.0, -1.0, -1.0);

        assert!(rect1.intersects(&rect2), "Rects should intersect");
        assert!(rect1.intersects(&rect3), "Rects should touch at the edge");
        assert!(!rect1.intersects(&rect4), "Rects should not intersect");

        let rect5 = make_rect(3.0, 3.0, 7.0, 7.0);
        assert!(
            rect1.intersects(&rect5),
            "One rect completely inside another should count as intersecting"
        );
    }

    #[test]
    fn insert_single_item() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item = TestPoint::new(25.0, 25.0);
        assert!(qt.insert(&item), "Should insert item successfully");
    }

    #[test]
    fn insert_item_out_of_bounds() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item = TestPoint::new(150.0, 150.0);
        assert!(!qt.insert(&item), "Should not insert item outside bounds");
    }

    #[test]
    fn insert_multiple_items_subdivision() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item1 = TestPoint::new(25.0, 25.0);
        let item2 = TestPoint::new(75.0, 75.0);

        qt.insert(&item1);
        assert!(qt.insert(&item2), "Should insert second item successfully");

        match qt {
            QuadTree::Internal { children, .. } => {
                assert_eq!(
                    children.len(),
                    4,
                    "Should have four children after subdivision"
                );
            }
            _ => panic!("QuadTree should have subdivided into an internal node"),
        }
    }

    #[test]
    fn query_rectangular_empty_quadtree() {
        let qt = QuadTree::<TestPoint>::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let mut results = Vec::new();
        qt.query_rectangular(&make_rect(10.0, 10.0, 50.0, 50.0), &mut results);
        assert!(results.is_empty(), "Should be empty for an empty tree");
    }

    #[test]
    fn query_rectangular_external_node_contains_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item = TestPoint::new(25.0, 25.0);
        qt.insert(&item);
        let mut results = Vec::new();
        qt.query_rectangular(&make_rect(20.0, 20.0, 30.0, 30.0), &mut results);
        assert_eq!(results.len(), 1, "Should find one item in the range");
        assert_eq!(
            results[0].point(),
            item.point(),
            "The point should match the inserted item"
        );
    }

    #[test]
    fn query_rectangular_external_node_does_not_contain_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item = TestPoint::new(75.0, 75.0);
        qt.insert(&item);
        let mut results = Vec::new();
        qt.query_rectangular(&make_rect(20.0, 20.0, 30.0, 30.0), &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the range"
        );
    }

    #[test]
    fn query_rectangular_internal_nodes_multiple_items() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item1 = TestPoint::new(25.0, 25.0);
        let item2 = TestPoint::new(75.0, 75.0);
        qt.insert(&item1);
        qt.insert(&item2);

        let mut results = Vec::new();
        qt.query_rectangular(&make_rect(20.0, 20.0, 80.0, 80.0), &mut results);
        assert_eq!(results.len(), 2, "Should find both items in the range");

        let mut results = Vec::new();
        qt.query_rectangular(&make_rect(70.0, 70.0, 80.0, 80.0), &mut results);
        assert_eq!(results.len(), 1, "Should find one item in the range");
        assert_eq!(
            results[0].point(),
            item2.point(),
            "The point should match the second inserted item"
        );

        let mut results = Vec::new();
        qt.query_rectangular(&make_rect(200.0, 200.0, 300.0, 300.0), &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the range"
        );
    }

    #[test]
    fn query_circular_empty_tree() {
        let qt = QuadTree::<TestPoint>::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let circle = make_circle(50.0, 50.0, 10.0);
        let mut results = Vec::new();
        qt.query_circular(&circle, &mut results);
        assert!(results.is_empty(), "Should be empty for an empty tree");
    }

    #[test]
    fn query_circular_contains_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item = TestPoint::new(25.0, 25.0);
        qt.insert(&item);
        let circle = make_circle(20.0, 20.0, 10.0);
        let mut results = Vec::new();
        qt.query_circular(&circle, &mut results);
        assert_eq!(results.len(), 1, "Should find one item within the circle");
        assert_eq!(
            results[0].point(),
            item.point(),
            "The point should match the inserted item"
        );
    }

    #[test]
    fn query_circular_does_not_contain_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item = TestPoint::new(75.0, 75.0);
        qt.insert(&item);
        let circle = make_circle(50.0, 50.0, 30.0);
        let mut results = Vec::new();
        qt.query_circular(&circle, &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the circle"
        );
    }

    #[test]
    fn query_circular_internal_nodes_multiple_items() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0));
        let item1 = TestPoint::new(35.0, 35.0);
        let item2 = TestPoint::new(65.0, 65.0);
        qt.insert(&item1);
        qt.insert(&item2);

        let mut results = Vec::new();
        let circle = make_circle(50.0, 50.0, 30.0);
        qt.query_circular(&circle, &mut results);
        assert_eq!(results.len(), 2, "Should find both items within the circle");

        let mut results = Vec::new();
        let circle = make_circle(70.0, 70.0, 10.0);
        qt.query_circular(&circle, &mut results);
        assert_eq!(results.len(), 1, "Should find one item within the circle");
        assert_eq!(
            results[0].point(),
            item2.point(),
            "The point should match the second inserted item"
        );

        let mut results = Vec::new();
        let circle = make_circle(200.0, 200.0, 50.0);
        qt.query_circular(&circle, &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the circle"
        );
    }
}
