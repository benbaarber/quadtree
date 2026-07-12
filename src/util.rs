use glam::vec2;

extern crate alloc;
use alloc::vec::Vec;

use crate::{
    Point,
    shapes::{Rect, Shape},
};

/// Get bounding rect for a list of items
pub(crate) fn bound_items<T: Point>(items: &[T]) -> Rect {
    let mut min_x = f32::MAX;
    let mut min_y = f32::MAX;
    let mut max_x = f32::MIN;
    let mut max_y = f32::MIN;

    for item in items {
        let p = item.point();
        min_x = min_x.min(p.x);
        min_y = min_y.min(p.y);
        max_x = max_x.max(p.x);
        max_y = max_y.max(p.y);
    }

    Rect::new(vec2(min_x, min_y), vec2(max_x, max_y))
}

// https://github.com/DeadlockCode/barnes-hut/blob/improved/src/partition.rs
pub(crate) trait Partition<T> {
    fn partition<F: Fn(&T) -> bool>(&mut self, predicate: F) -> usize;
}

impl<T> Partition<T> for [T] {
    fn partition<F: Fn(&T) -> bool>(&mut self, predicate: F) -> usize {
        if self.is_empty() {
            return 0;
        }

        let mut l = 0;
        let mut r = self.len() - 1;

        loop {
            while l <= r && predicate(&self[l]) {
                l += 1;
            }
            while l < r && !predicate(&self[r]) {
                r -= 1;
            }
            if l >= r {
                return l;
            }

            self.swap(l, r);
            l += 1;
            r -= 1;
        }
    }
}

pub(crate) fn group_by_quadrant<T: Point>(rect: Rect, items: Vec<T>) -> [Vec<T>; 5] {
    let mut groups: [Vec<T>; 5] = core::array::from_fn(|_| Vec::with_capacity(items.len()));
    for item in items {
        match rect.quadrant(item.point()) {
            Some(q) => groups[q].push(item),
            None => groups[4].push(item),
        }
    }
    groups
}

#[allow(unused)]
pub(crate) fn group_by_quadrant_slice<'a, T: Point>(rect: Rect, items: &'a [T]) -> [Vec<&'a T>; 5] {
    let mut groups: [Vec<&T>; 5] = core::array::from_fn(|_| Vec::with_capacity(items.len()));
    for item in items {
        match rect.quadrant(item.point()) {
            Some(q) => groups[q].push(item),
            None => groups[4].push(item),
        }
    }
    groups
}

pub(crate) fn determine_overlap_quadrants(outer: &Rect, inner: &Rect) -> Vec<usize> {
    let mut quadrants = Vec::with_capacity(4);
    for (i, rect) in outer.quarter().iter().enumerate() {
        if rect.intersects(inner) {
            quadrants.push(i);
        }
    }
    quadrants
}

#[cfg(test)]
pub(crate) mod tests {
    use glam::vec2;

    use crate::shapes::*;

    use super::*;

    pub(crate) fn make_rect(x1: f32, y1: f32, x2: f32, y2: f32) -> Rect {
        Rect::new(vec2(x1, y1), vec2(x2, y2))
    }

    pub(crate) fn make_circle(x: f32, y: f32, r: f32) -> Circle {
        Circle::new(vec2(x, y), r)
    }

    #[test]
    fn test_group_by_quadrant() {
        let rect = make_rect(0.0, 0.0, 10.0, 10.0);
        let points = [
            vec2(2.5, 2.5),   // Should be in the first quadrant (index 0)
            vec2(7.5, 2.5),   // Should be in the second quadrant (index 1)
            vec2(2.5, 7.5),   // Should be in the third quadrant (index 2)
            vec2(7.5, 7.5),   // Should be in the fourth quadrant (index 3)
            vec2(10.5, 10.5), // Should be outside all quadrants (index 4)
        ];

        let expected_groups = [
            vec![vec2(2.5, 2.5)],
            vec![vec2(7.5, 2.5)],
            vec![vec2(2.5, 7.5)],
            vec![vec2(7.5, 7.5)],
            vec![vec2(10.5, 10.5)],
        ];

        let results = group_by_quadrant(rect, points.to_vec());

        for (expected, result) in expected_groups.iter().zip(results.iter()) {
            assert_eq!(
                result, expected,
                "Each group should match its expected value"
            );
        }
    }

    #[test]
    fn test_determine_overlap_quadrants() {
        let outer = make_rect(0.0, 0.0, 100.0, 100.0);

        // Test with an inner rectangle that intersects multiple quadrants
        let inner_multiple_overlap = make_rect(25.0, 25.0, 75.0, 75.0);
        assert_eq!(
            determine_overlap_quadrants(&outer, &inner_multiple_overlap),
            &[0, 1, 2, 3],
            "Inner rectangle overlaps all quadrants."
        );

        // Test with an inner rectangle that overlaps only one quadrant
        let inner_single_overlap = make_rect(10.0, 10.0, 30.0, 30.0);
        assert_eq!(
            determine_overlap_quadrants(&outer, &inner_single_overlap),
            &[0],
            "Inner rectangle overlaps only the first quadrant."
        );

        // Test with an inner rectangle that does not overlap any quadrant
        let inner_no_overlap = make_rect(101.0, 101.0, 150.0, 150.0);
        assert!(
            determine_overlap_quadrants(&outer, &inner_no_overlap).is_empty(),
            "Inner rectangle does not overlap any quadrant."
        );

        // Test with an inner rectangle that overlaps on the boundary between two quadrants
        let inner_boundary_overlap = make_rect(50.0, 50.0, 70.0, 70.0);
        assert_eq!(
            determine_overlap_quadrants(&outer, &inner_boundary_overlap),
            &[0, 1, 2, 3],
            "Inner rectangle overlaps the boundary between all quadrants."
        );
    }

    #[test]
    fn test_partition_basic() {
        let mut arr = [1, 4, 2, 5, 3];
        // move all < 4 to front
        let pivot = arr.partition(|&x| x < 4);
        // elements before pivot are <4, after >=4
        assert_eq!(pivot, 3);
        assert!(arr[..pivot].iter().all(|&x| x < 4));
        assert!(arr[pivot..].iter().all(|&x| x >= 4));
    }

    #[test]
    fn test_bound_items_rect() {
        let pts = vec![vec2(0.0, 0.0), vec2(2.0, 4.0)];
        let bound = bound_items(&pts);
        // center should be midpoint
        assert_eq!(bound.center(), vec2(1.0, 2.0));
        // perimeter = 2*(width+height) = 2*(2+4) = 12
        assert_eq!(bound.perimeter(), 12.0);
        // quarter() should produce four sub-rectangles of equal size
        let quads = bound.quarter();
        for qr in &quads {
            // each quadrant width = 1, height = 2
            assert_eq!(qr.perimeter(), 2.0 * (1.0 + 2.0));
        }
    }
}
