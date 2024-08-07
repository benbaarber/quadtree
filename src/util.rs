use crate::{
    shapes::{Rect, Shape},
    P2,
};

pub(crate) fn determine_quadrant(rect: &Rect, point: &P2) -> Option<usize> {
    for (i, rect) in rect.quarter().iter().enumerate() {
        if rect.contains(point) {
            return Some(i);
        }
    }
    None
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
    use nalgebra::point;

    use crate::shapes::*;

    use super::*;

    pub(crate) fn make_rect(x1: f64, y1: f64, x2: f64, y2: f64) -> Rect {
        Rect::new(point![x1, y1], point![x2, y2])
    }

    pub(crate) fn make_circle(x: f64, y: f64, r: f64) -> Circle {
        Circle::new(point![x, y], r)
    }

    #[test]
    fn test_determine_quadrant() {
        let rect = make_rect(0.0, 0.0, 10.0, 10.0);
        let points = [
            point![2.5, 2.5],   // Should be in the first quadrant (index 0)
            point![7.5, 2.5],   // Should be in the second quadrant (index 1)
            point![2.5, 7.5],   // Should be in the third quadrant (index 2)
            point![7.5, 7.5],   // Should be in the fourth quadrant (index 3)
            point![10.5, 10.5], // Should be outside all quadrants (None)
        ];

        let expected_quadrants = [Some(0), Some(1), Some(2), Some(3), None];
        let results = points
            .iter()
            .map(|point| determine_quadrant(&rect, point))
            .collect::<Vec<_>>();

        assert_eq!(
            results, expected_quadrants,
            "Each point should match its expected quadrant"
        );
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
        assert_eq!(
            determine_overlap_quadrants(&outer, &inner_no_overlap),
            &[],
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
}
