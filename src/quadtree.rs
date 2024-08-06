use crate::{
    shapes::{Rect, Shape},
    util::{determine_overlap_quadrants, determine_quadrant},
    Point, P2,
};

/// A generic QuadTree implementation for spatial indexing of 2D points.
#[derive(Debug)]
pub struct QuadTree<T> {
    root: Node<T>,
    node_capacity: usize,
}

impl<T: Point + Clone> QuadTree<T> {
    /// Create a new empty quadtree
    ///
    /// ## Arguments
    /// - `boundary`: The boundary of the quadtree
    /// - `node_capacity`: The maximum number of items a node can hold before subdividing
    pub fn new(boundary: Rect, node_capacity: usize) -> Self {
        Self {
            root: Node::Empty { boundary },
            node_capacity,
        }
    }

    /// Insert a point into the quadtree
    ///
    /// **Returns** a boolean value indicating if the item was inserted successfully
    pub fn insert(&mut self, item: &T) -> bool {
        self.root.insert(item, self.node_capacity)
    }

    /// Queries the QuadTree for items within a specified shape area.
    /// This method populates a passed mutable vector with all found items.
    pub fn query<S: Shape>(&self, shape: &S, results: &mut Vec<T>) {
        self.root.query(shape, results)
    }

    pub fn get(&self, point: &P2) -> Option<T> {
        self.root.get(point)
    }

    pub fn delete<S: Shape>(&mut self, shape: &S) {
        self.root.delete(shape);
    }

    /// Return the point at the center of the boundary
    pub fn center(&self) -> P2 {
        self.root.center()
    }

    /// Get the boundary rect of the quadtree
    pub fn boundary(&self) -> &Rect {
        self.root.boundary()
    }
}

/// QuadTree node enum
///
/// ## Variants
/// - `Internal`: Contains children nodes and represents a subdivided area.
/// - `External`: Contains data and represents a leaf node.
/// - `Empty`: Represents an empty area without any data.
#[derive(Debug)]
enum Node<T> {
    Internal {
        boundary: Rect,
        children: [Box<Self>; 4],
    },
    External {
        boundary: Rect,
        data: Vec<T>,
    },
    Empty {
        boundary: Rect,
    },
}

impl<T: Point + Clone> Node<T> {
    fn insert(&mut self, item: &T, capacity: usize) -> bool {
        let point = item.point();

        if !self.boundary().contains(&point) {
            return false;
        }

        match self {
            &mut Self::Empty { boundary } => {
                let mut data = Vec::with_capacity(capacity);
                data.push(item.clone());
                *self = Self::External { boundary, data };
                true
            }
            &mut Self::External {
                boundary,
                ref mut data,
            } => {
                if data.len() < capacity {
                    data.push(item.clone());
                    return true;
                }

                let data = std::mem::take(data);
                let children = self.subdivide();
                *self = Self::Internal { boundary, children };

                for existing_item in &data {
                    self.insert(existing_item, capacity);
                }

                self.insert(item, capacity)
            }
            Self::Internal { boundary, children } => match determine_quadrant(boundary, &point) {
                Some(q) => children[q].insert(item, capacity),
                None => false,
            },
        }
    }

    fn query<S: Shape>(&self, shape: &S, results: &mut Vec<T>) {
        match self {
            Self::External { boundary, data } => {
                if shape.contains_rect(boundary) {
                    results.extend(data.clone());
                } else {
                    for item in data {
                        if shape.contains(&item.point()) {
                            results.push(item.clone());
                        }
                    }
                }
            }
            Self::Internal { boundary, children } => {
                if boundary.intersects(&shape.rect()) {
                    for q in determine_overlap_quadrants(boundary, &shape.rect()) {
                        children[q].query(shape, results);
                    }
                }
            }
            Self::Empty { .. } => (),
        }
    }

    fn get(&self, point: &P2) -> Option<T> {
        match self {
            Self::External { data, .. } => {
                for item in data {
                    if item.point() == *point {
                        return Some(item.clone());
                    }
                }
                None
            }
            Self::Internal { boundary, children } => match determine_quadrant(boundary, point) {
                Some(q) => children[q].get(point),
                None => None,
            },
            Self::Empty { .. } => None,
        }
    }

    /// Delete from quadtree
    ///
    /// Returns true if the node is empty after deletion
    fn delete<S: Shape>(&mut self, shape: &S) -> bool {
        match self {
            &mut Self::External {
                boundary,
                ref mut data,
            } => {
                if !boundary.intersects(&shape.rect()) {
                    return false;
                }

                if shape.contains_rect(&boundary) {
                    *self = Self::Empty { boundary };
                    return true;
                }

                *data = data
                    .drain(..)
                    .filter(|item| !shape.contains(&item.point()))
                    .collect();

                if data.is_empty() {
                    *self = Self::Empty { boundary };
                    return true;
                }
                false
            }
            &mut Self::Internal {
                boundary,
                ref mut children,
            } => {
                if boundary.intersects(&shape.rect()) {
                    let mut is_all_empty = true;
                    for c in children {
                        let is_empty = c.delete(shape);
                        if !is_empty {
                            is_all_empty = false;
                        }
                    }
                    if is_all_empty {
                        *self = Self::Empty { boundary };
                        return true;
                    }
                }

                false
            }
            Self::Empty { .. } => true,
        }
    }

    fn center(&self) -> P2 {
        self.boundary().center()
    }

    fn boundary(&self) -> &Rect {
        match self {
            Self::Empty { boundary } => boundary,
            Self::External { boundary, .. } => boundary,
            Self::Internal { boundary, .. } => boundary,
        }
    }

    fn subdivide(&self) -> [Box<Self>; 4] {
        let rects = self.boundary().quarter();
        rects.map(|r| Box::new(Self::Empty { boundary: r }))
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::point;

    use crate::{
        util::tests::{make_circle, make_rect},
        Point,
    };

    use super::*;

    #[test]
    fn insert_single_item() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![25.0, 25.0];
        assert!(qt.insert(&item), "Should insert item successfully");
    }

    #[test]
    fn insert_item_out_of_bounds() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![150.0, 150.0];
        assert!(!qt.insert(&item), "Should not insert item outside bounds");
    }

    #[test]
    fn insert_multiple_items_subdivision() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 2);
        let item1 = point![20.0, 20.0];
        let item2 = point![40.0, 40.0];
        let item3 = point![60.0, 60.0];

        qt.insert(&item1);
        qt.insert(&item2);
        qt.insert(&item3);

        match qt.root {
            Node::Internal { children, .. } => {
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
        let qt = QuadTree::<P2>::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let mut results = Vec::new();
        qt.query(&make_rect(10.0, 10.0, 50.0, 50.0), &mut results);
        assert!(results.is_empty(), "Should be empty for an empty tree");
    }

    #[test]
    fn query_rectangular_external_node_contains_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![25.0, 25.0];
        qt.insert(&item);
        let mut results = Vec::new();
        qt.query(&make_rect(20.0, 20.0, 30.0, 30.0), &mut results);
        assert_eq!(results.len(), 1, "Should find one item in the range");
        assert_eq!(
            results[0].point(),
            item.point(),
            "The point should match the inserted item"
        );
    }

    #[test]
    fn query_rectangular_external_node_does_not_contain_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![75.0, 75.0];
        qt.insert(&item);
        let mut results = Vec::new();
        qt.query(&make_rect(20.0, 20.0, 30.0, 30.0), &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the range"
        );
    }

    #[test]
    fn query_rectangular_internal_nodes_multiple_items() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item1 = point![25.0, 25.0];
        let item2 = point![75.0, 75.0];
        qt.insert(&item1);
        qt.insert(&item2);

        let mut results = Vec::new();
        qt.query(&make_rect(20.0, 20.0, 80.0, 80.0), &mut results);
        assert_eq!(results.len(), 2, "Should find both items in the range");

        let mut results = Vec::new();
        qt.query(&make_rect(70.0, 70.0, 80.0, 80.0), &mut results);
        assert_eq!(results.len(), 1, "Should find one item in the range");
        assert_eq!(
            results[0].point(),
            item2.point(),
            "The point should match the second inserted item"
        );

        let mut results = Vec::new();
        qt.query(&make_rect(200.0, 200.0, 300.0, 300.0), &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the range"
        );
    }

    #[test]
    fn query_rectangular_boundary_edge_overlap() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 4);
        let edge_point = point![100.0, 50.0]; // Exactly on the boundary edge
        qt.insert(&edge_point);
        let query_shape = make_rect(95.0, 45.0, 105.0, 55.0);
        let mut results = Vec::new();
        qt.query(&query_shape, &mut results);
        assert_eq!(results.len(), 1, "Should find the edge point.");
    }

    #[test]
    fn query_circular_empty_tree() {
        let qt = QuadTree::<P2>::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let circle = make_circle(50.0, 50.0, 10.0);
        let mut results = Vec::new();
        qt.query(&circle, &mut results);
        assert!(results.is_empty(), "Should be empty for an empty tree");
    }

    #[test]
    fn query_circular_contains_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![25.0, 25.0];
        qt.insert(&item);
        let circle = make_circle(20.0, 20.0, 10.0);
        let mut results = Vec::new();
        qt.query(&circle, &mut results);
        assert_eq!(results.len(), 1, "Should find one item within the circle");
        assert_eq!(
            results[0].point(),
            item.point(),
            "The point should match the inserted item"
        );
    }

    #[test]
    fn query_circular_does_not_contain_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![75.0, 75.0];
        qt.insert(&item);
        let circle = make_circle(50.0, 50.0, 30.0);
        let mut results = Vec::new();
        qt.query(&circle, &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the circle"
        );
    }

    #[test]
    fn query_circular_internal_nodes_multiple_items() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item1 = point![35.0, 35.0];
        let item2 = point![65.0, 65.0];
        qt.insert(&item1);
        qt.insert(&item2);

        let mut results = Vec::new();
        let circle = make_circle(50.0, 50.0, 30.0);
        qt.query(&circle, &mut results);
        assert_eq!(results.len(), 2, "Should find both items within the circle");

        let mut results = Vec::new();
        let circle = make_circle(70.0, 70.0, 10.0);
        qt.query(&circle, &mut results);
        assert_eq!(results.len(), 1, "Should find one item within the circle");
        assert_eq!(
            results[0].point(),
            item2.point(),
            "The point should match the second inserted item"
        );

        let mut results = Vec::new();
        let circle = make_circle(200.0, 200.0, 50.0);
        qt.query(&circle, &mut results);
        assert!(
            results.is_empty(),
            "Should not find any items outside the circle"
        );
    }

    #[test]
    fn delete_rect() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let points = vec![point![10.0, 10.0], point![30.0, 10.0], point![10.0, 30.0]];
        for point in &points {
            qt.insert(point);
        }

        let deletion_shape = make_rect(5.0, 5.0, 35.0, 15.0);
        qt.delete(&deletion_shape);
        assert!(
            qt.get(&points[0]).is_none(),
            "Point at (10.0, 10.0) should have been deleted"
        );
        assert!(
            qt.get(&points[1]).is_none(),
            "Point at (30.0, 10.0) should have been deleted"
        );
        assert!(
            qt.get(&points[2]).is_some(),
            "Point at (10.0, 30.0) should still exist"
        );
    }

    #[test]
    fn delete_rect_bounds() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let points = vec![point![10.0, 10.0], point![20.0, 20.0], point![30.0, 30.0]];
        for point in &points {
            qt.insert(point);
        }

        let deletion_shape = make_rect(15.0, 15.0, 25.0, 25.0);
        qt.delete(&deletion_shape);
        assert!(
            qt.get(&points[0]).is_some(),
            "Point at (10.0, 10.0) should still exist"
        );
        assert!(
            qt.get(&points[1]).is_none(),
            "Point at (20.0, 20.0) should have been deleted"
        );
        assert!(
            qt.get(&points[2]).is_some(),
            "Point at (30.0, 30.0) should still exist"
        );
    }

    #[test]
    fn delete_rect_multi_level() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 2);
        // Points to cause subdivisions
        let points = [
            point![10.0, 10.0],
            point![90.0, 90.0],
            point![10.0, 90.0],
            point![90.0, 10.0],
            point![50.0, 50.0],
            point![30.0, 30.0],
        ];
        for p in &points {
            qt.insert(p);
        }
        let deletion_shape = make_rect(25.0, 25.0, 75.0, 75.0);
        qt.delete(&deletion_shape);
        let mut results = Vec::new();
        qt.query(&make_rect(0.0, 0.0, 100.0, 100.0), &mut results);
        assert_eq!(
            results.len(),
            4,
            "Should have 4 points left after partial deletion."
        );
    }

    #[test]
    fn precise_floating_point_handling() {
        let mut qt = QuadTree::new(make_rect(0.00001, 0.00001, 99.99999, 99.99999), 2);
        let point = point![0.0001, 0.0001];
        assert!(
            qt.insert(&point),
            "Point near the boundary should be inserted."
        );
    }
}
