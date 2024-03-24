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

use geo::{coord, polygon, Coord, EuclideanDistance, Line, Point, Polygon};
use ordered_float::OrderedFloat;
use plotly::common::{Fill, Line as PlotlyLine, Mode};
use plotly::{Layout, Plot, Scatter};
use rand::Rng;
use rustplanning::planning::rrt::rrt;
use rustplanning::tree::{Distance, HashTree};
use std::env;

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

    fn to_coord(&self) -> Coord<f64> {
        coord! {x: self.inner().x().into_inner(), y: self.inner().y().into_inner()}
    }

    fn extend(&self, end: &Self, step_size: f64) -> Self {
        let direction = (
            (end.0.x() - self.0.x()).into_inner(),
            (end.0.y() - self.0.y()).into_inner(),
        );
        let length = self.distance(end);
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
    pub bounds: (f64, f64),

    // Closed polygons with inaccessible interiors
    pub obstacles: Vec<Polygon>,
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

    /// Returns whether or not a line between the two provided poses intersects with
    /// any obstacles and if the distance is within the maximum connectable step size.
    pub fn connectable(
        &self,
        from: &RobotPose,
        to: &RobotPose,
        buffer: f64,
        step_size: f64,
    ) -> bool {
        let line = Line::new(from.to_coord(), to.to_coord());
        let intersects = self
            .obstacles
            .iter()
            .any(|obstacle| line.euclidean_distance(obstacle) < buffer);
        let reachable = from.distance(to) < step_size;
        !intersects && reachable
    }
}

/// Visualize a successful path
fn visualize_rrt(
    world: &World,
    path: &Vec<RobotPose>,
    tree: &HashTree<RobotPose>,
) {
    let mut plot = Plot::new();

    // Plot obstacles
    for obstacle in &world.obstacles {
        let (x, y): (Vec<_>, Vec<_>) = obstacle.exterior().points().map(|p| (p.x(), p.y())).unzip();
        let trace = Scatter::new(x, y)
            .fill(Fill::ToSelf)
            .fill_color("black")
            .line(PlotlyLine::new().color("black"))
            .opacity(1.0);
        plot.add_trace(trace);
    }

    // Plot tree
    for pose in tree.iter_depth_first() {
        if let Some(parent_pose) = tree.get_parent(pose) {
            let p = pose.to_point();
            let parent = parent_pose.to_point();
            let trace = Scatter::new(vec![p.x(), parent.x()], vec![p.y(), parent.y()])
                .mode(Mode::Lines)
                .line(PlotlyLine::new().color("blue").width(1.0));
            plot.add_trace(trace);
        }
    }

    // Plot path
    let path_x: Vec<_> = path
        .iter()
        .map(|pose| pose.inner().x().into_inner())
        .collect();
    let path_y: Vec<_> = path
        .iter()
        .map(|pose| pose.inner().y().into_inner())
        .collect();
    let path_trace = Scatter::new(path_x, path_y)
        .mode(Mode::Lines)
        .line(PlotlyLine::new().color("red").width(4.0));
    plot.add_trace(path_trace);

    // Plot start and end
    let start = path.first().unwrap();
    let end = path.last().unwrap();
    let start_trace = Scatter::new(
        vec![start.inner().x().into_inner()],
        vec![start.inner().y().into_inner()],
    )
    .mode(Mode::Markers)
    .marker(plotly::common::Marker::new().color("green").size(16));
    let end_trace = Scatter::new(
        vec![end.inner().x().into_inner()],
        vec![end.inner().y().into_inner()],
    )
    .mode(Mode::Markers)
    .marker(plotly::common::Marker::new().color("yellow").size(16));
    plot.add_trace(start_trace);
    plot.add_trace(end_trace);

    let layout = Layout::new()
        .title(format!("RRT Path Finding Result").as_str().into())
        .show_legend(false)
        .width(750)
        .height(750)
        .x_axis(plotly::layout::Axis::new().title("X".into()))
        .y_axis(plotly::layout::Axis::new().title("Y".into()));

    plot.set_layout(layout);
    plot.show();
}

pub fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 7 && args.len() != 8 {
        eprintln!("Usage: program start_x start_y end_x end_y use_rrtstar use_rrtconnect [timeout]");
        return;
    }

    let start_x: f64 = args[1].parse().expect("Invalid start_x");
    let start_y: f64 = args[2].parse().expect("Invalid start_y");
    let end_x: f64 = args[3].parse().expect("Invalid end_x");
    let end_y: f64 = args[4].parse().expect("Invalid end_y");
    let use_rrtstar: bool = args[5]
        .parse()
        .expect("Invalid use_rrtstar argument; should be true or false");
    let use_rrtconnect: bool = args[6]
        .parse()
        .expect("Invalid use_rrtconnect argument; should be true or false");
    let mut fast_return = true;
    let mut timeout = 1000.0;
    if args.len() == 8 {
        fast_return = false;
        timeout = args[7].parse().expect("Invalid timeout");
    }

    let start = RobotPose::new(start_x, start_y);
    let goal = RobotPose::new(end_x, end_y);

    println!("Starting pathfinding with parameters:");
    println!("  start pose: ({}, {})", start_x, start_y);
    println!("  end pose: ({}, {})", end_x, end_y);
    println!("  use_rrtstar: {}", use_rrtstar);
    println!("  use_rrtconnect: {}", use_rrtconnect);
    println!("  fast_return: {}", fast_return);
    println!("  timeout: {}", timeout);

    // Add a few rectangular obstacles to the world
    let obstacles = vec![
        polygon![(x: 10.0, y: 10.0), (x: 30.0, y: 10.0), (x: 30.0, y: 30.0), (x: 10.0, y: 30.0), (x: 10.0, y: 10.0)],
        polygon![(x: 50.0, y: 50.0), (x: 80.0, y: 50.0), (x: 80.0, y: 80.0), (x: 50.0, y: 80.0), (x: 50.0, y: 50.0)],
        polygon![(x: 70.0, y: 20.0), (x: 90.0, y: 20.0), (x: 90.0, y: 40.0), (x: 70.0, y: 40.0), (x: 70.0, y: 20.0)],
        polygon![(x: 35.0, y: 30.0), (x: 45.0, y: 30.0), (x: 45.0, y: 90.0), (x: 35.0, y: 90.0), (x: 35.0, y: 30.0)],
    ];

    let world = World::new(100.0, 100.0, obstacles);

    // Constants for this particular run
    let buffer = 1.0; // All samples must be > 1.0 away from obstacles.
    let step_size = 1.0; // Distance between existing nodes and samples.
    let rewire_radius = 5.0; // Radius for rewiring tree if using RRT*.

    // Define closures
    let sample_fn = || world.sample();
    let extend_fn = |from: &RobotPose, to: &RobotPose| from.extend(to, step_size);
    let connectable_fn =
        |from: &RobotPose, to: &RobotPose| world.connectable(from, to, buffer, rewire_radius);

    let result = rrt(
        &start,
        &goal,
        sample_fn,
        extend_fn,
        connectable_fn,
        use_rrtstar,
        rewire_radius,
        use_rrtconnect,
        1000000,
        timeout,
        fast_return,
    );
    match result {
        Ok((path, tree)) => {
            println!("Path found!");
            visualize_rrt(&world, &path, &tree);
        }
        Err(e) => {
            println!("RRT failed: {}", e);
        }
    }
}
