// MIT License
//
// Copyright (c) 2024 Erik Holum
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

use ordered_float::OrderedFloat;
use rand::Rng;
use rustplanning::planning::rrt::rrt;
use rustplanning::tree::Distance;
use std::fmt;

/// Basic 2D point class for representing hashable points in the plane
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
struct Point2D(OrderedFloat<f64>, OrderedFloat<f64>);

impl Point2D {
    pub fn new(x: f64, y: f64) -> Self {
        Point2D(OrderedFloat(x), OrderedFloat(y))
    }

    pub fn x(&self) -> f64 {
        self.0.into_inner()
    }

    pub fn y(&self) -> f64 {
        self.1.into_inner()
    }
}

// Norm distance function for 2D points
impl Distance for Point2D {
    fn distance(&self, other: &Point2D) -> f64 {
        let (dx, dy) = (self.x() - other.x(), self.y() - other.y());
        (dx * dx + dy * dy).sqrt()
    }
}

// Handy for debugging
impl fmt::Display for Point2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Customize the format here. The following is a simple example.
        write!(f, "({:.2}, {:.2})", self.x(), self.y())
    }
}

/// Function for randomly sampling the 2-D plane between (0, 0) and (10.0, 10.0)
fn sample() -> Point2D {
    let mut rng = rand::thread_rng();
    Point2D::new(rng.gen_range(0.0..=10.0), rng.gen_range(0.0..=10.0))
}

// Returns a point that is 0.1 along the line between the specified start and end pose
fn extend(start: &Point2D, end: &Point2D) -> Point2D {
    let direction = (end.0 - start.0, end.1 - start.1);
    let length = (direction.0.powi(2) + direction.1.powi(2)).sqrt();
    let norm_direction = (direction.0 / length, direction.1 / length);
    let step_size = 0.1;
    Point2D(
        start.0 + norm_direction.0 * step_size,
        start.1 + norm_direction.1 * step_size,
    )
}

#[test]
fn test_rrt() {
    let start = Point2D::new(0.0, 0.0);
    let goal = Point2D::new(9.0, 9.0);

    // Success is within this tolerance of the goal pose.
    // This is a test of a random algorithm so just making this real big so that it *always
    // succeeds.
    let success_distance = 0.5;

    // All points are valid for now
    let is_valid = |_: &Point2D| true;

    // Are we within 0.5 of the goal?
    let success = |p: &Point2D| p.distance(&goal) < success_distance;

    let result = rrt(
        &start, sample, extend, is_valid, success, 10000,
    );

    assert!(result.is_ok(), "Expected Ok result, got Err");

    let path = result.unwrap();
    assert!(!path.is_empty(), "Path should not be empty");
    assert_eq!(path[0], start, "Path should start at the start point");

    // Verify it ends at the goal
    let end = path.last().unwrap();
    assert!(end.distance(&goal) < success_distance, "Path should end near the goal");
    print!("Path:\n");
    for p in path {
        print!("  {p}\n");
    }
}
