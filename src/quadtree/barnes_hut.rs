use crate::{
    Point,
    shapes::Rect,
    util::{Partition, bound_items},
};
use glam::Vec2;
use std::ops::Range;

/// A point with mass
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct WeightedPoint {
    pub pos: Vec2,
    pub mass: f32,
}

impl WeightedPoint {
    pub fn new(pos: Vec2, mass: f32) -> Self {
        Self { pos, mass }
    }
}

impl Point for WeightedPoint {
    fn point(&self) -> Vec2 {
        self.pos
    }
}

#[derive(Debug)]
struct Node {
    bound: Rect,
    children: usize,
    next: usize,
    cm: WeightedPoint,
    items: Range<usize>,
}

impl Node {
    fn new(bound: Rect, items: Range<usize>, next: usize) -> Self {
        Self {
            bound,
            items,
            next,
            children: 0,
            cm: WeightedPoint::default(),
        }
    }
}

/// A Quadtree specially optimized for the Barnes-Hut algorithm
///
/// An interactive explanation of the algorithm can be found
/// [here](https://jheer.github.io/barnes-hut/)
///
/// This quadtree is immutable and flat, not recursive. It is optimized to be rebuilt frequently
/// and supports efficient accumulation of approximated force. Each step, convert your items into
/// [`WeightedPoint`] and call the build method to clear and reconstruct the tree. Then for each
/// item you need to accumulate force on, call the accumulate method, passing a custom force
/// function.
///
/// This implementation is heavily inspired by [DeadlockCode's Barnes-Hut
/// implementation](https://github.com/DeadlockCode/barnes-hut/tree/improved)
#[derive(Debug)]
pub struct BHQuadtree {
    nodes: Vec<Node>,
    internal_nodes: Vec<usize>,
    items: Vec<WeightedPoint>,
    node_capacity: usize,
    max_depth: usize,
    theta2: f32,
}

impl BHQuadtree {
    /// Create a new empty BHQuadtree
    ///
    /// ## Arguments
    /// - `node_capacity`: The maximum number of items a node can hold before subdividing
    /// - `max_depth`: The maximum depth of the tree at which nodes will ignore capacity
    /// - `theta`: Theta parameter of Barnes-Hut algorithm
    pub fn new(node_capacity: usize, max_depth: usize, theta: f32) -> Self {
        Self {
            nodes: Vec::new(),
            internal_nodes: Vec::new(),
            items: Vec::new(),
            node_capacity,
            max_depth,
            theta2: theta * theta,
        }
    }

    /// Clear all internal data and reconstruct the tree from a sequence of weighted points
    pub fn build(&mut self, items: Vec<WeightedPoint>) {
        self.nodes.clear();
        self.internal_nodes.clear();
        self.items = items;

        let bound = bound_items(&self.items);
        self.nodes.push(Node::new(bound, 0..self.items.len(), 0));

        let mut node_depths = vec![0];

        let mut n = 0;
        while n < self.nodes.len() {
            let range = self.nodes[n].items.clone();
            let depth = node_depths[n];
            if range.len() > self.node_capacity && depth < self.max_depth {
                self.subdivide(n, range);
                node_depths.extend([depth + 1; 4]);
            } else {
                for i in range {
                    self.nodes[n].cm.pos += self.items[i].pos * self.items[i].mass;
                    self.nodes[n].cm.mass += self.items[i].mass;
                }
            }
            n += 1;
        }

        for &n in self.internal_nodes.iter().rev() {
            let c = self.nodes[n].children;
            for i in 0..4 {
                let cm = self.nodes[c + i].cm;
                self.nodes[n].cm.pos += cm.pos;
                self.nodes[n].cm.mass += cm.mass;
            }
        }

        for node in &mut self.nodes {
            node.cm.pos /= node.cm.mass.max(f32::MIN_POSITIVE);
        }
    }

    /// Accumulate a force vector to act on a target position with an arbitrary force function,
    /// approximating weighted points based on the theta parameter.
    pub fn accumulate<F: Fn(WeightedPoint) -> Vec2>(&self, target: Vec2, force_fn: F) -> Vec2 {
        let mut acc = Vec2::ZERO;

        let mut n = 0;
        loop {
            let node = &self.nodes[n];
            let cm = node.cm;
            let d2 = Vec2::distance_squared(target, cm.pos);
            let s = (node.bound.bb - node.bound.aa).max_element();
            if (s * s) < self.theta2 * d2 {
                acc += force_fn(cm);
                n = node.next;
            } else if node.children == 0 {
                for i in node.items.clone() {
                    acc += force_fn(self.items[i]);
                }
                n = node.next;
            } else {
                n = node.children;
            }

            if n == 0 {
                break;
            }
        }

        acc
    }

    fn subdivide(&mut self, n: usize, range: Range<usize>) {
        let c = self.nodes.len();
        self.nodes[n].children = c;
        self.internal_nodes.push(n);

        let center = self.nodes[n].bound.center();

        let mut split = [range.start, 0, 0, 0, range.end];

        let predicate = |item: &WeightedPoint| item.pos.y < center.y;
        split[2] = split[0] + self.items[split[0]..split[4]].partition(predicate);

        let predicate = |item: &WeightedPoint| item.pos.x < center.x;
        split[1] = split[0] + self.items[split[0]..split[2]].partition(predicate);
        split[3] = split[2] + self.items[split[2]..split[4]].partition(predicate);

        let bounds = self.nodes[n].bound.quarter();
        let nexts = [c + 1, c + 2, c + 3, self.nodes[n].next];
        for i in 0..4 {
            let items = split[i]..split[i + 1];
            self.nodes.push(Node::new(bounds[i], items, nexts[i]));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::Rect;
    use glam::vec2;

    #[test]
    fn test_weighted_point_traits() {
        let p = WeightedPoint::new(vec2(3.0, 4.0), 2.5);
        // Default
        let def = WeightedPoint::default();
        assert_eq!(def.pos, vec2(0.0, 0.0));
        assert_eq!(def.mass, 0.0);
        // Point trait
        assert_eq!(p.pos, vec2(3.0, 4.0));
    }

    #[test]
    fn test_node_new_initializer() {
        let rect = Rect::new(vec2(0.0, 1.0), vec2(1.0, 2.0));
        let range = 0..5;
        let node = Node::new(rect, range.clone(), 7);
        assert_eq!(node.children, 0);
        assert_eq!(node.next, 7);
        assert_eq!(node.items, range);
        assert_eq!(node.bound, rect);
        assert_eq!(node.cm, WeightedPoint::default());
    }

    #[test]
    fn test_build_and_accumulate_no_subdivide() {
        // two points, no subdivide (capacity >= 2)
        let pts = vec![
            WeightedPoint::new(vec2(0.0, 0.0), 1.0),
            WeightedPoint::new(vec2(1.0, 0.0), 1.0),
        ];
        let mut qt = BHQuadtree::new(2, 8, 0.0); // theta=0 forces full traversal
        qt.build(pts.clone());
        // one node
        assert_eq!(qt.nodes.len(), 1);
        // cm should be average of positions
        let cm = qt.nodes[0].cm;
        assert_eq!(cm.pos, vec2(0.5, 0.0));
        assert_eq!(cm.mass, 2.0);
        // accumulate with identity on items
        let sum: glam::Vec2 = qt.accumulate(vec2(0.0, 0.0), |wp| wp.pos);
        // since theta=0, it will sum items directly: (0,0)+(1,0)
        assert_eq!(sum, vec2(1.0, 0.0));
        // accumulate with theta large to use cm
        let mut qt2 = BHQuadtree::new(2, 8, 1000.0);
        qt2.build(pts.clone());
        let avg: glam::Vec2 = qt2.accumulate(vec2(0.0, 0.0), |wp| wp.pos);
        // should return cm.pos only
        assert_eq!(avg, vec2(0.5, 0.0));
    }

    #[test]
    fn test_subdivide_and_accumulate() {
        // two points, capacity=1 so it subdivides
        let pts = vec![
            WeightedPoint::new(vec2(0.0, 0.0), 1.0),
            WeightedPoint::new(vec2(2.0, 0.0), 1.0),
        ];
        let mut qt = BHQuadtree::new(1, 8, 0.0);
        qt.build(pts);
        // root + 4 children
        assert_eq!(qt.nodes.len(), 1 + 4);
        // internal_nodes contains root index 0
        assert_eq!(qt.internal_nodes, vec![0]);
        // each leaf has its own single point
        for child_idx in qt.nodes[0].children..qt.nodes[0].children + 4 {
            let leaf = &qt.nodes[child_idx];
            assert!(leaf.items.len() <= 1);
        }
        // accumulate with theta=0 sums two points
        let sum = qt.accumulate(vec2(1.0, 0.0), |wp| wp.pos);
        assert_eq!(sum, vec2(2.0, 0.0));
    }

    #[test]
    fn many_identical_points() {
        let mut qt = BHQuadtree::new(4, 4, 1.0);
        let mut pts = vec![
            WeightedPoint::new(vec2(0.0, 0.0), 1.0),
            WeightedPoint::new(vec2(2.0, 2.0), 1.0),
        ];
        pts.extend([WeightedPoint::new(vec2(0.2, 0.2), 1.0); 10]);
        // Should not infinite loop
        qt.build(pts);
    }
}
