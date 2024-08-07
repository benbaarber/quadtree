# QuadTree

[![Current Crates.io Version](https://img.shields.io/crates/v/quadtree.svg)](https://crates.io/crates/quadtree)
[![Documentation](https://img.shields.io/badge/Docs-latest-blue)](https://docs.rs/quadtree/latest/quadtree/)

This QuadTree library provides efficient spatial querying capabilities for 2D points. It supports various operations such as insertion, and rectangular and circular querying, making it suitable for applications in areas such as gaming, geographical information systems, and real-time simulations.

## Features

- **Generic Implementation**: Works with any data type that implements the `Point` trait.
- **Spatial Queries**: Supports querying within spatial regions that implement the `Shape` trait (`Rect` and `Circle` are provided).
- **Dynamic Operations**: Efficiently perform mutating operations without full rebuilds.
    - insert
    - insert_many
    - delete
    - pop