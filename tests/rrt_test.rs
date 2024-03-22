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
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rustplanning::planning::rrt::rrt;
use rustplanning::tree::Distance;
use std::f64::EPSILON;
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

/// Function for randomly sampling the 2-D plane
fn sample_2d(rng: &mut StdRng, max_x: f64, max_y: f64) -> Point2D {
    Point2D::new(rng.gen_range(0.0..=max_x), rng.gen_range(0.0..=max_y))
}

// Returns a point that is step_size along the line between the specified start and end pose
fn extend_2d(start: &Point2D, end: &Point2D, step_size: f64) -> Point2D {
    let direction = (end.0 - start.0, end.1 - start.1);
    let length = (direction.0.powi(2) + direction.1.powi(2)).sqrt();
    let norm_direction = (direction.0 / length, direction.1 / length);
    Point2D(
        start.0 + norm_direction.0 * step_size,
        start.1 + norm_direction.1 * step_size,
    )
}

fn run_rrt(use_rrtstar: bool, start: &Point2D, goal: &Point2D, grid_size: f64) {
    // Seed the generator for consistency
    let mut rng = StdRng::seed_from_u64(1);
    let step_size = 1.0;

    // Define closures
    let obstacle = Point2D::new(grid_size / 2.0, grid_size / 2.0); // All points except for ball in the center are valid
    let extend_fn = |start: &Point2D, end: &Point2D| extend_2d(start, end, step_size);
    let mut sample_fn = || sample_2d(&mut rng, grid_size, grid_size);
    let connectable_fn = |start: &Point2D, end: &Point2D| {
        end.distance(&obstacle) > 3.0 &&
        start.distance(end) < step_size
    };

    let result = rrt(
        start,
        goal,
        &mut sample_fn,
        &extend_fn,
        &connectable_fn,
        use_rrtstar,
        2.0,
        100000,
    );

    assert!(result.is_ok(), "Expected Ok result, got Err");

    let (path, _) = result.unwrap();
    assert!(!path.is_empty(), "Path should not be empty");
    assert_eq!(path[0], *start, "Path should start at the start point");

    // Verify it ends at the goal
    let end = path.last().unwrap();
    assert!(
        end.distance(&goal) < EPSILON,
        "Path should end at the goal"
    );
}

#[test]
fn test_rrt() {
    let start = Point2D::new(1.0, 1.0);
    let end = Point2D::new(10.0, 10.0);
    let grid_size = 10.0;
    run_rrt(false, &start, &end, grid_size);
}

#[test]
fn test_rrtstar() {
    let start = Point2D::new(1.0, 1.0);
    let end = Point2D::new(10.0, 10.0);
    let grid_size = 10.0;
    run_rrt(true, &start, &end, grid_size);
}
