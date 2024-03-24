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
use std::time::{Duration, Instant};

/// Attempts to randomly extend the tree in an arbitrary direction.
/// Return the new point and the nearest neighbor, if available.
/// Otherwise return None.
///
/// If `use_connect`, continue extending until the sample is reached or we can't
/// connect.
fn extend_tree<T, FE, FC>(
    tree: &HashTree<T>,
    sample: T,
    extend: &mut FE,
    connectable: &mut FC,
    use_connect: bool,
) -> (Vec<T>, T)
where
    T: Eq + Copy + Hash + Distance,
    FE: FnMut(&T, &T) -> T,
    FC: FnMut(&T, &T) -> bool,
{
    // Sample the grab the nearest point, and extend in that direction
    let nearest = tree.nearest_neighbor(&sample);
    let mut path = Vec::new();

    if connectable(&nearest, &sample) {
        path.push(sample);
    }
    // If using connect, extend until we can extend no further or we begin
    // moving further away from the sample.
    else if use_connect {
        let mut current_point = nearest;
        let mut distance_to_sample = current_point.distance(&sample);
        while !connectable(&current_point, &sample) {
            let new_point = extend(&current_point, &sample);
            let new_distance_to_sample = new_point.distance(&sample);
            if new_distance_to_sample >= distance_to_sample
                || !connectable(&current_point, &new_point)
            {
                break;
            }

            path.push(new_point);
            distance_to_sample = new_distance_to_sample;
            current_point = path.last().unwrap();
        }
        if connectable(&current_point, &sample) {
            path.push(sample);
        }
    } else {
        let new_point = extend(&nearest, &sample);
        if connectable(&nearest, &new_point) {
            path.push(new_point);
        }
    }

    (path, nearest.clone())
}

fn rewire_tree<T, FC>(tree: &mut HashTree<T>, connectable: &mut FC, point: &T, rewire_radius: f64)
where
    T: Eq + Copy + Hash + Distance,
    FC: FnMut(&T, &T) -> bool,
{
    // Get a list of all nodes that are within the sample radius, and rewire if necessary
    let neighbors = tree.nearest_neighbors(point, rewire_radius);
    let point_cost = tree.cost(point).unwrap();
    for (neighbor, distance) in neighbors.iter() {
        if neighbor == point {
            continue;
        }
        // If it's cheaper and valid to get to the neighbor from the new node reparent it
        let old_cost = tree.cost(neighbor).unwrap();
        let new_cost = distance + point_cost;
        if new_cost < old_cost {
            if connectable(point, neighbor) {
                let _ = tree.set_parent(neighbor, point);
            }
        }
    }
}

/// Implementation of RRT planning algorithms.
///
/// Will attempt to compute a path using the specified version of RRT given the start pose
/// and user-defined coverage functions.
///
/// # Parameters
///
/// - `start`: The reference to the starting pose of type `T`
/// - `sample_fn`: Function to randomly sample the configuration space
/// - `extend_fn`: Given two nodes, function to return an intermediate value between them
/// - `connectable_fn`: Function to determine whether or not a link can be added between two nodes. If a sampled node is
///                     connectable to the goal, we return success.
/// - `use_rrtstar`: Whether or not to use RRT*
/// - `rewire_radius`: If using RRT*, the max distance to identify and rewire neighbors of newly added nodes
/// - `use_rrtconnect`: Whether or not to use RRT connect
/// - `max_iterations`: Maximum number of random samples to attempt before the search fails
/// - `max_duration`: Maximum amount of time in seconds to find a solution
/// - `fast_return`: Return as soon as a solution is found, or iterate until max_iterations or max_duration is reached
///
/// # Returns
/// Returns a `Result` containing either:
/// - `Ok((Vec<T>, Tree<T>))`: A tuple of a vector of points of type `T` representing the path from the
///                 start to a poin satisfying the `success` condition, if such a path is found within
///                 the given number of iterations. Along with the Tree itself.
/// - `Err(String)`: An error message in a string if the algorithm fails to find a satisfactory path.
///
/// # Example
///
/// Refer to the world example or integration tests.
///
pub fn rrt<T, FS, FE, FC>(
    start: &T,
    goal: &T,
    mut sample_fn: FS,
    mut extend_fn: FE,
    mut connectable_fn: FC,
    use_rrtstar: bool,
    rewire_radius: f64,
    use_rrtconnect: bool,
    max_iterations: u64,
    max_duration: f64,
    fast_return: bool,
) -> Result<(Vec<T>, HashTree<T>), String>
where
    T: Eq + Copy + Hash + Distance,
    FS: FnMut() -> T,
    FE: FnMut(&T, &T) -> T,
    FC: FnMut(&T, &T) -> bool,
{
    let mut tree = HashTree::new(start.clone());
    let start_time = Instant::now();
    let duration_limit = Duration::from_secs_f64(max_duration);

    for _ in 0..max_iterations {
        // Have we timed out?
        if start_time.elapsed() > duration_limit {
            break;
        }

        // Sample the nearest point, and extend in that direction.
        // If we end up with no connectable nodes just try again.
        let sample = sample_fn();
        let (new_points, nearest) = extend_tree(
            &tree,
            sample,
            &mut extend_fn,
            &mut connectable_fn,
            use_rrtconnect,
        );
        if new_points.is_empty() {
            continue;
        }

        // Add all valid nodes to the tree
        let mut parent = &nearest;
        for node in &new_points {
            let _ = tree.add_child(parent, *node);
            parent = &node;
        }

        // Rewire the tree if using RRT*
        if use_rrtstar {
            for node in &new_points {
                rewire_tree(&mut tree, &mut connectable_fn, &node, rewire_radius);
            }
        }

        // If we have reached the goal ensure the link is added to the tree.
        if connectable_fn(goal, new_points.last().unwrap()) {
            let _ = tree.add_child(new_points.last().unwrap(), *goal);

            // Then we're done.
            if fast_return {
                break;
            }
        }
    }

    match tree.path(goal) {
        Ok(path) => return Ok((path, tree)),
        Err(_) => return Err("Failed to find path between poses".into()),
    }
}

//
// Unit tests
//

#[cfg(test)]
mod tests {

    use crate::{planning::rrt::rewire_tree, tree::HashTree};

    use super::extend_tree;

    #[test]
    fn test_rewire_tree() {
        // Tree is: 2 -> 4 -> 1
        let mut tree: HashTree<i32> = HashTree::new(2);
        assert!(tree.add_child(&2, 4).is_ok());
        assert!(tree.add_child(&4, 1).is_ok());
        let mut is_valid_fn = |_: &i32, _: &i32| -> bool { true };

        assert_eq!(tree.get_parent(&4).unwrap(), &2);
        assert_eq!(tree.get_parent(&1).unwrap(), &4);
        assert_eq!(tree.cost(&1).unwrap(), 5.0);

        // When we rewire at 2, 1 should be reparented
        // 2 -> 1
        //   -> 4
        rewire_tree(&mut tree, &mut is_valid_fn, &2, 5.0);
        assert_eq!(tree.get_parent(&4).unwrap(), &2);
        assert_eq!(tree.get_parent(&1).unwrap(), &2);
        assert_eq!(tree.cost(&1).unwrap(), 1.0);
    }

    #[test]
    fn test_extend_tree() {
        let tree: HashTree<i32> = HashTree::new(1);
        let mut extend_fn = |from: &i32, _: &i32| from + 1;
        let mut connectable_fn = |from: &i32, to: &i32| (to - from).abs() == 1;

        // The sample is right next to the nearest node, so it should connect directly
        let (new_points, nearest) = extend_tree(
            &tree,
            2,
            &mut extend_fn,
            &mut connectable_fn,
            false,
        );
        let nearest_path = vec![2];
        assert_eq!(nearest, 1);
        assert_eq!(new_points, nearest_path);

        // Extend the path by exactly 1
        let (new_points, nearest) = extend_tree(
            &tree,
            3,
            &mut extend_fn,
            &mut connectable_fn,
            false,
        );
        let nearest_path = vec![2];
        assert_eq!(nearest, 1);
        assert_eq!(new_points, nearest_path);

        // Connect all the way to the sample
        let (new_points, nearest) = extend_tree(
            &tree,
            5,
            &mut extend_fn,
            &mut connectable_fn,
            true,
        );
        let nearest_path = vec![2, 3, 4, 5];
        assert_eq!(nearest, 1);
        assert_eq!(new_points, nearest_path);
    }
}
