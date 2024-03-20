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

use std::env;

use geo::{polygon, EuclideanDistance, Point, Polygon};
use ordered_float::OrderedFloat;
use rand::Rng;

use rustplanning::planning::rrt::rrtstar;
use rustplanning::tree::Distance;

// Define a new wrapper type around `geo::Point<f64>` for robot poses, and
// to satisfy additional required traits.
#[derive(Debug, Hash, Clone, Copy)]
struct RobotPose(Point<OrderedFloat<f64>>);

// Implement methods to easily create and interact with `MyPoint`
impl RobotPose {
    fn new(x: f64, y: f64) -> Self {
        RobotPose(Point::new(OrderedFloat(x), OrderedFloat(y)))
    }

    fn inner(&self) -> &Point<OrderedFloat<f64>> {
        &self.0
    }

    fn to_point(&self) -> Point<f64> {
        Point::new(self.inner().x().into_inner(), self.inner().y().into_inner())
    }

    fn extend(&self, end: &Self, step_size: f64) -> Self {
        let direction = (
            (end.0.x() - self.0.x()).into_inner(),
            (end.0.y() - self.0.y()).into_inner(),
        );
        let length = (direction.0.powi(2) + direction.1.powi(2)).sqrt();
        let norm_direction = (direction.0 / length, direction.1 / length);
        RobotPose::new(
            self.0.x().into_inner() + norm_direction.0 * step_size,
            self.0.y().into_inner() + norm_direction.1 * step_size,
        )
    }
}

impl PartialEq for RobotPose {
    fn eq(&self, other: &Self) -> bool {
        self.0.x() == other.0.x() && self.0.y() == other.0.y()
    }
}

impl Eq for RobotPose {}

// Required inherited trait
impl Distance for RobotPose {
    fn distance(&self, other: &Self) -> f64 {
        let (dx, dy) = (self.0.x() - other.0.x(), self.0.y() - other.0.y());
        (dx * dx + dy * dy).sqrt()
    }
}

/// Simple representation of a 2-D rectangular world.
///
/// Limits are from 0 to x_max and y_max.
/// Obstacles are represented by Polygons.
struct World {
    /// x_max and y_max for the world, must be >0.0
    bounds: (f64, f64),

    // Closed polygons with inaccessible interiors
    obstacles: Vec<Polygon>,
}

impl World {
    /// Constructs a new world object with the specified shapes
    pub fn new(x_max: f64, y_max: f64, obstacles: Vec<Polygon>) -> Self {
        World {
            bounds: (x_max, y_max),
            obstacles: obstacles,
        }
    }

    pub fn sample(&self) -> RobotPose {
        let mut generator = rand::thread_rng();
        let x = generator.gen_range(0.0..=self.bounds.0);
        let y = generator.gen_range(0.0..=self.bounds.1);
        RobotPose::new(x, y)
    }

    pub fn is_valid(&self, p: &RobotPose) -> bool {
        !self
            .obstacles
            .iter()
            .any(|obstacle| p.to_point().euclidean_distance(obstacle) <= 0.0)
    }
}

pub fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 5 {
        eprintln!("Usage: program start_x start_y end_x end_y");
        return;
    }

    let start_x: f64 = args[1].parse().expect("Invalid start_x");
    let start_y: f64 = args[2].parse().expect("Invalid start_y");
    let end_x: f64 = args[3].parse().expect("Invalid end_x");
    let end_y: f64 = args[4].parse().expect("Invalid end_y");

    let start = RobotPose::new(start_x, start_y);
    let end = RobotPose::new(end_x, end_y);

    println!("Start pose: ({}, {})", start_x, start_y);
    println!("End pose: ({}, {})", end_x, end_y);

    // Add a few square obstacles to the world
    let obstacles = vec![
        polygon![(x: 20.0, y: 20.0), (x: 30.0, y: 20.0), (x: 30.0, y: 30.0), (x: 20.0, y: 30.0), (x: 20.0, y: 20.0)],
        polygon![(x: 50.0, y: 50.0), (x: 60.0, y: 50.0), (x: 60.0, y: 60.0), (x: 50.0, y: 60.0), (x: 50.0, y: 50.0)],
        polygon![(x: 70.0, y: 20.0), (x: 80.0, y: 20.0), (x: 80.0, y: 30.0), (x: 70.0, y: 30.0), (x: 70.0, y: 20.0)],
    ];

    let world = World::new(100.0, 100.0, obstacles);

    // Define closures for rrtstar
    let sample_fn = || world.sample();
    let extend_fn = |from: &RobotPose, to: &RobotPose| from.extend(to, 1.0);
    let is_valid_fn = |pose: &RobotPose| world.is_valid(pose);
    let success_fn = |pose: &RobotPose| pose.distance(&end) <= 3.0;

    // Call rrtstar
    match rrtstar(
        &start,
        sample_fn,
        extend_fn,
        is_valid_fn,
        success_fn,
        2.5,
        100000,
    ) {
        Ok((path, _tree)) => {
            println!("Path found:");
            for pose in path {
                println!(
                    "({}, {})",
                    pose.inner().x().into_inner(),
                    pose.inner().y().into_inner()
                );
            }
        }
        Err(e) => {
            println!("RRT* failed: {}", e);
        }
    }
}
