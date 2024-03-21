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
use crate::tree::HashTree;
use std::hash::Hash;

/// Attempts to randomly extend the tree in an arbitrary direction.
/// Return the new point and the nearest neighbor, if available.
/// Otherwise return None.
fn extend_tree<T, FS, FE, FV>(
    tree: &HashTree<T>,
    sample: &mut FS,
    extend: &mut FE,
    is_valid: &mut FV,
) -> Option<(T, T)>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FV: FnMut(&T, &T) -> bool,
{
    // Sample the grab the nearest point, and extend in that direction
    let s = sample();
    let nearest = tree.nearest_neighbor(&s);
    let new_point = extend(&nearest, &s);

    // If it is an invalid point try again
    if !is_valid(nearest, &new_point) {
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
/// - `sample`: Function to randomly sample the configuration space
/// - `extend`: Given two nodes, function to return an intermediate value between them
/// - `is_valid`: Function to determine whether or not a link can be added between two nodes
/// - `success`:  Returns whether or not a node has reached the goal
/// - `max_iterations`: Maximum number of random samples to attempt before the search fails
///
/// # Returns
/// Returns a `Result` containing either:
/// - `Ok((Vec<T>, Tree<T>))`: A tuple of a vector of points of type `T` representing the path from the
///                 start to a poin satisfying the `success` condition, if such a path is found within
///                 the given number of iterations. Along with the Tree itself.
/// - `Err(String)`: An error message in a string if the algorithm fails to find a satisfactory path
///                  within the `max_iterations`.
///
/// # Example
///
/// Refer to the world example or integration tests.
///
pub fn rrt<T, FS, FE, FV, FD>(
    start: &T,
    mut sample: FS,
    mut extend: FE,
    mut is_valid: FV,
    mut success: FD,
    max_iterations: u64,
) -> Result<(Vec<T>, HashTree<T>), String>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FV: FnMut(&T, &T) -> bool,
    FD: FnMut(&T) -> bool,
{
    let mut tree = HashTree::new(start.clone());

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
                Ok(path) => return Ok((path, tree)),
                Err(e) => return Err(e),
            }
        }
    }

    // Otherwise we've hit max_iter with finding success
    Err("Failed to find a path".to_string())
}

/// Basic implementation for RRTStar.
///
/// Method signature is nearly identical to [`rrt`], though includes a radius for
/// rewiring neighbors of sampled nodes.
///
/// # Parameters
///
/// Are the same as `rrt` excepting for,
///
/// - `rewire_radius`: The max distance to identify and rewire neighbors of newly added nodes
///
pub fn rrtstar<T, FS, FE, FV, FD>(
    start: &T,
    mut sample: FS,
    mut extend: FE,
    mut is_valid: FV,
    mut success: FD,
    rewire_radius: f64,
    max_iterations: u64,
) -> Result<(Vec<T>, HashTree<T>), String>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FV: FnMut(&T, &T) -> bool,
    FD: FnMut(&T) -> bool,
{
    let mut tree = HashTree::new(start.clone());

    for _ in 0..max_iterations {
        // Sample the grab the nearest point, and extend in that direction
        let (new_point, nearest) = match extend_tree(&tree, &mut sample, &mut extend, &mut is_valid)
        {
            Some((new_point, nearest)) => (new_point, nearest),
            None => continue,
        };

        // If it's valid add it to the tree
        tree.add_child(&nearest, new_point).unwrap();
        let new_cost = tree.cost(&new_point).unwrap();

        // Get a list of all nodes that are within the sample radius, and rewire if necessary
        let neighbors = tree.nearest_neighbors(&new_point, rewire_radius);
        for (neighbor, distance) in neighbors.iter() {
            if neighbor == &new_point {
                continue;
            }
            if !is_valid(&new_point, neighbor) {
                continue;
            }
            // If it's cheaper to get to the neighbor from the new node reparent it
            if distance + new_cost < tree.cost(neighbor).unwrap() {
                tree.set_parent(neighbor, &new_point)?;
            }
        }

        // Are we there yet? If so return the path.
        if success(&new_point) {
            match tree.path(&new_point) {
                Ok(path) => return Ok((path, tree)),
                Err(e) => return Err(e),
            }
        }
    }

    // Otherwise we've hit max_iter with finding success
    Err("Failed to find a path".to_string())
}
