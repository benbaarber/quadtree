use crate::{
    shapes::{Rect, Shape},
    util::{determine_overlap_quadrants, determine_quadrant},
    Point, P2,
};

/// A generic QuadTree implementation for spatial indexing of 2D points
#[derive(Debug)]
pub struct QuadTree<T> {
    root: Node<T>,
    node_capacity: usize,
    count: usize,
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
            count: 0,
        }
    }

    /// Get current number of items stored
    pub fn count(&self) -> usize {
        self.count
    }

    /// Insert an item into the QuadTree
    ///
    /// **Returns** a boolean value indicating if the item was inserted successfully
    pub fn insert(&mut self, item: &T) -> bool {
        let success = self.root.insert(item, self.node_capacity);
        if success {
            self.count += 1;
        }
        success
    }

    /// Insert multiple items into the QuadTree
    ///
    /// **Returns** a vector of items that failed to insert, if any
    pub fn insert_many(&mut self, items: &[T]) -> Vec<T> {
        let mut failed = Vec::with_capacity(items.len());
        for item in items {
            let success = self.insert(item);
            if !success {
                failed.push(item.clone());
            }
        }
        failed
    }

    /// Get an item by its exact position
    ///
    /// **Returns** an `Option` containing the item if it exists
    pub fn get(&self, point: &P2) -> Option<T> {
        self.root.get(point)
    }

    /// Query for items within a specified shape area
    ///
    /// **Returns** a vector of items that were found within the shape
    pub fn query<S: Shape>(&self, shape: &S) -> Vec<T> {
        let mut results = vec![];
        self.root.query(shape, &mut results);
        results
    }

    /// Delete items that are within a specified shape area
    ///
    /// **Returns** the number of items that were deleted
    pub fn delete<S: Shape>(&mut self, shape: &S) -> usize {
        let mut deleted = 0;
        self.root.delete(shape, &mut deleted);
        self.count -= deleted;
        deleted
    }

    /// Pop items that are within a specified shape area
    ///
    /// **Returns** a vector of items that were found within the shape and removed
    pub fn pop<S: Shape>(&mut self, shape: &S) -> Vec<T> {
        let mut results = vec![];
        self.root.pop(shape, &mut results);
        self.count -= results.len();
        results
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
/// - `Internal`: Contains children nodes and represents a subdivided area
/// - `External`: Contains data and represents a leaf node
/// - `Empty`: Represents an empty area without any data
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

    // Returns true if the node is empty after deletion
    fn delete<S: Shape>(&mut self, shape: &S, deleted: &mut usize) -> bool {
        match self {
            &mut Self::External {
                boundary,
                ref mut data,
            } => {
                if !boundary.intersects(&shape.rect()) {
                    return false;
                }

                if shape.contains_rect(&boundary) {
                    *deleted += data.len();
                    *self = Self::Empty { boundary };
                    return true;
                }

                let original_data_len = data.len();
                data.retain(|item| !shape.contains(&item.point()));
                *deleted += original_data_len - data.len();

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
                        let is_empty = c.delete(shape, deleted);
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

    // Returns true if the node is empty after deletion
    fn pop<S: Shape>(&mut self, shape: &S, results: &mut Vec<T>) -> bool {
        match self {
            &mut Self::External {
                boundary,
                ref mut data,
            } => {
                if !boundary.intersects(&shape.rect()) {
                    return false;
                }

                if shape.contains_rect(&boundary) {
                    results.extend(data.drain(..));
                    *self = Self::Empty { boundary };
                    return true;
                }

                let mut left_data = Vec::with_capacity(data.capacity());
                for item in data.drain(..) {
                    if shape.contains(&item.point()) {
                        results.push(item);
                    } else {
                        left_data.push(item);
                    }
                }

                if left_data.is_empty() {
                    *self = Self::Empty { boundary };
                    true
                } else {
                    *data = left_data;
                    false
                }
            }
            &mut Self::Internal {
                boundary,
                ref mut children,
            } => {
                if boundary.intersects(&shape.rect()) {
                    let mut is_all_empty = true;
                    for c in children {
                        let is_empty = c.pop(shape, results);
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
    fn insert_multiple_items() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let points = vec![
            point![10.0, 10.0],
            point![150.0, 150.0], // This should fail (out of bounds)
            point![20.0, 20.0],
            point![120.0, 120.0], // This should fail (out of bounds)
        ];

        let failed_inserts = qt.insert_many(&points);

        assert_eq!(failed_inserts.len(), 2, "Should return 2 failed inserts");
        assert!(
            failed_inserts.contains(&points[1]),
            "Should include point (150, 150) as failed"
        );
        assert!(
            failed_inserts.contains(&points[3]),
            "Should include point (120, 120) as failed"
        );

        // Ensure that successful points are indeed in the QuadTree
        assert!(
            qt.get(&points[0].point()).is_some(),
            "Point (10, 10) should be successfully inserted"
        );
        assert!(
            qt.get(&points[2].point()).is_some(),
            "Point (20, 20) should be successfully inserted"
        );

        // Check count correctness
        assert_eq!(
            qt.count(),
            2,
            "Count should be 2 for successfully inserted items"
        );
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
        let points = vec![point![20.0, 20.0], point![40.0, 40.0], point![60.0, 60.0]];

        qt.insert_many(&points);

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
    fn get_item() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let point = point![20.0, 20.0];
        qt.insert(&point);

        assert_eq!(
            qt.get(&point),
            Some(point),
            "Should find the inserted point"
        );
        assert!(
            qt.get(&point![30.0, 30.0]).is_none(),
            "Should not find a point that was not inserted"
        );
    }

    #[test]
    fn query_rectangular_empty_quadtree() {
        let qt = QuadTree::<P2>::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let results = qt.query(&make_rect(10.0, 10.0, 50.0, 50.0));
        assert!(results.is_empty(), "Should be empty for an empty tree");
    }

    #[test]
    fn query_rectangular_external_node_contains_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![25.0, 25.0];
        qt.insert(&item);
        let results = qt.query(&make_rect(20.0, 20.0, 30.0, 30.0));
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
        let results = qt.query(&make_rect(20.0, 20.0, 30.0, 30.0));
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

        let results = qt.query(&make_rect(20.0, 20.0, 80.0, 80.0));
        assert_eq!(results.len(), 2, "Should find both items in the range");

        let results = qt.query(&make_rect(70.0, 70.0, 80.0, 80.0));
        assert_eq!(results.len(), 1, "Should find one item in the range");
        assert_eq!(
            results[0].point(),
            item2.point(),
            "The point should match the second inserted item"
        );

        let results = qt.query(&make_rect(200.0, 200.0, 300.0, 300.0));
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
        let results = qt.query(&query_shape);
        assert_eq!(results.len(), 1, "Should find the edge point.");
    }

    #[test]
    fn query_circular_empty_tree() {
        let qt = QuadTree::<P2>::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let circle = make_circle(50.0, 50.0, 10.0);
        let results = qt.query(&circle);
        assert!(results.is_empty(), "Should be empty for an empty tree");
    }

    #[test]
    fn query_circular_contains_point() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let item = point![25.0, 25.0];
        qt.insert(&item);
        let circle = make_circle(20.0, 20.0, 10.0);
        let results = qt.query(&circle);
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
        let results = qt.query(&circle);
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

        let circle = make_circle(50.0, 50.0, 30.0);
        let results = qt.query(&circle);
        assert_eq!(results.len(), 2, "Should find both items within the circle");

        let circle = make_circle(70.0, 70.0, 10.0);
        let results = qt.query(&circle);
        assert_eq!(results.len(), 1, "Should find one item within the circle");
        assert_eq!(
            results[0].point(),
            item2.point(),
            "The point should match the second inserted item"
        );

        let circle = make_circle(200.0, 200.0, 50.0);
        let results = qt.query(&circle);
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
        let deleted = qt.delete(&deletion_shape);
        assert_eq!(deleted, 2, "Two items were deleted");
        assert_eq!(qt.count(), 1, "One item remains in tree");
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
        assert_eq!(qt.count(), 3, "Three items should be in the tree");

        let deletion_shape = make_rect(15.0, 15.0, 25.0, 25.0);
        let deleted = qt.delete(&deletion_shape);
        assert_eq!(deleted, 1, "One item was deleted");
        assert_eq!(qt.count(), 2, "Two items remain in tree");
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
        let results = qt.query(&make_rect(0.0, 0.0, 100.0, 100.0));
        assert_eq!(
            results.len(),
            4,
            "Should have 4 points left after partial deletion."
        );
    }

    #[test]
    fn test_pop_function() {
        let mut qt = QuadTree::new(make_rect(0.0, 0.0, 100.0, 100.0), 1);
        let points = vec![
            point![10.0, 10.0],
            point![20.0, 20.0],
            point![30.0, 30.0],
            point![40.0, 40.0],
        ];

        qt.insert_many(&points);

        assert_eq!(qt.count(), 4, "All four points were inserted");

        let results = qt.pop(&make_rect(15.0, 15.0, 35.0, 35.0));
        assert_eq!(
            results.len(),
            2,
            "Should pop two points that intersect with the shape"
        );
        assert!(
            results.contains(&points[1]),
            "Should contain point (20, 20)"
        );
        assert!(
            results.contains(&points[2]),
            "Should contain point (30, 30)"
        );

        let remaining_points = qt.query(&make_rect(0.0, 0.0, 100.0, 100.0));
        assert_eq!(qt.count(), 2, "Two points should remain in the quadtree");
        assert!(
            remaining_points.contains(&points[0]),
            "Should still contain point (10, 10)"
        );
        assert!(
            remaining_points.contains(&points[3]),
            "Should still contain point (40, 40)"
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
