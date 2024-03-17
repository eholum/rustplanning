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

use crate::tree::Distance;
use crate::tree::Tree;
use std::hash::Hash;

/// Attempts to randomly extend the tree in an arbitrary direction.
/// Return the new point and the nearest neighbor, if available.
/// Otherwise return None.
fn extend_tree<T, FS, FE, FV>(
    tree: &Tree<T>,
    sample: &mut FS,
    extend: &mut FE,
    is_valid: &mut FV,
) -> Option<(T, T)>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FV: FnMut(&T) -> bool,
{
    // Sample the grab the nearest point, and extend in that direction
    let s = sample();
    let nearest = tree.nearest_neighbor(&s);
    let new_point = extend(&nearest, &s);

    // If it is an invalid point try again
    if !is_valid(&new_point) {
        return None;
    }

    Some((new_point, nearest.clone()))
}

/// Basic RRT implementation.
///
/// Will attempt to compute a path using the RRT algorithm given the specified start pose
/// and user-defined coverage functions.
///
/// # Parameters
///
/// - `start`: The reference to the starting pose of type `T`
/// - `sample`: Generate a random sample pose over the search space
/// - `extend`: Function to extend a pose towards a randomly sampled pose
/// - `is_valid`: Determines whether or not a pose is valid
/// - `success`: Determines whether or not we have reached the goal
/// - `max_iterations`: Maximum number of random samples to attempt before the search fails
///
/// # Returns
/// Returns a `Result` containing either:
/// - `Ok(Vec<T>)`: A vector of points of type `T` representing the path from the start to a point
///                 satisfying the `success` condition, if such a path is found within the given number
///                 of iterations.
/// - `Err(String)`: An error message in a string if the algorithm fails to find a satisfactory path
///                  within the `max_iterations`.
///
/// # Example
///
/// See the integration tests for an example.
///
pub fn rrt<T, FS, FE, FV, FD>(
    start: &T,
    mut sample: FS,
    mut extend: FE,
    mut is_valid: FV,
    mut success: FD,
    max_iterations: usize,
) -> Result<Vec<T>, String>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FV: FnMut(&T) -> bool,
    FD: FnMut(&T) -> bool,
{
    let mut tree = Tree::new(start.clone());

    for _ in 0..max_iterations {
        let (new_point, nearest) = match extend_tree(&tree, &mut sample, &mut extend, &mut is_valid)
        {
            Some((new_point, nearest)) => (new_point, nearest),
            None => continue,
        };

        if tree.add_child(&nearest, new_point).is_err() {
            // Then the child wasn't added for some reason so just try again
            continue;
        }

        // Are we there yet? If so return the path.
        if success(&new_point) {
            match tree.path(&new_point) {
                Ok(path) => return Ok(path),
                Err(e) => return Err(e),
            }
        }
    }

    // Otherwise we've hit max_iter with finding success
    Err("Failed to find a path".to_string())
}

/// Basic implementation for RRTStar.
/// Method signature is nearly identical to rrt, though includes a radius for
/// rewiring neighbors based on distances.
///
/// WIP!
///
pub fn rrtstar<T, FS, FE, FV, FD>(
    start: &T,
    mut sample: FS,
    mut extend: FE,
    mut is_valid: FV,
    mut success: FD,
    sample_radius: f64,
    max_iterations: usize,
) -> Result<Vec<T>, String>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FV: FnMut(&T) -> bool,
    FD: FnMut(&T) -> bool,
{
    let mut tree = Tree::new(start.clone());

    for _ in 0..max_iterations {
        // Sample the grab the nearest point, and extend in that direction
        let (new_point, nearest) = match extend_tree(&tree, &mut sample, &mut extend, &mut is_valid)
        {
            Some((new_point, nearest)) => (new_point, nearest),
            None => continue,
        };

        // Compute the cost to reach the new node from the nearest node
        let new_cost = new_point.distance(&nearest) + tree.cost(&nearest).unwrap();

        // Get a list of all nodes that are within the sample radius
        let neighbors_maybe = tree.nearest_neighbors(&new_point, sample_radius);
        if neighbors_maybe.is_err() {
            continue;
        }
        let (_, neighbors) = neighbors_maybe.unwrap();

        // Rewire the tree
        for (neighbor, cost) in neighbors.iter() {
            if new_cost + neighbor.distance(&new_point) < *cost {
                tree.set_parent(&new_point, neighbor)?;
            }
        }

        if tree.add_child(&nearest, new_point).is_err() {
            continue;
        }

        // Are we there yet? If so return the path.
        if success(&new_point) {
            match tree.path(&new_point) {
                Ok(path) => return Ok(path),
                Err(e) => return Err(e),
            }
        }
    }

    // Otherwise we've hit max_iter with finding success
    Err("Failed to find a path".to_string())
}
