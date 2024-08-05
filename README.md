# QuadTree

[![Current Crates.io Version](https://img.shields.io/crates/v/quadtree.svg)](https://crates.io/crates/quadtree)
[![Documentation](https://img.shields.io/badge/Docs-latest-blue)](https://docs.rs/quadtree/0.1.0/quadtree/)

This QuadTree library provides efficient spatial querying capabilities for 2D points. It supports various operations such as insertion, and rectangular and circular querying, making it suitable for applications in areas such as gaming, geographical information systems, and real-time simulations.

## Features

- **Generic Implementation**: Works with any data type that implements the custom `Point` trait.
- **Spatial Queries**: Supports querying within rectangular and circular areas.
- **Dynamic Insertion**: Efficiently insert items and maintain spatial indexing without full rebuilds.

## Planned

- **Dynamic Deletion**: Delete items individually and by query without full rebuilds.
