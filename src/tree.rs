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

//! Basic tree structure to store vertices with arbitrary data types.
//! Types must implement a distance trait to enable determination of nearest neighbors.
use std::collections::{HashMap, HashSet};
use std::hash::Hash;

/// Basic node element for the tree.
///
/// Must be used with [Tree] since children are referenced by index in the [Tree]'s node vector.
#[derive(Debug)]
struct Node<T> {
    // The value of this node.
    value: T,

    // Location of the nodes parent, if available
    parent: Option<usize>,

    // Maintains a list of pointers to the children's location in the parent's vector.
    // Using a vector to maintain order for tree traversals.
    children: Vec<usize>,

    // Set of nearest neighbors.
    neighbors: HashSet<usize>,

    // Optional cost to reach this node.
    cost: Option<f64>,
}

impl<T> Node<T> {
    fn new(value: T, parent: Option<usize>) -> Self {
        Node {
            value: value,
            parent: parent,
            children: Vec::new(),
            neighbors: HashSet::new(),
            cost: None,
        }
    }
}

/// Define a distance trait for tree node values.
pub trait Distance {
    fn distance(&self, other: &Self) -> f64;
}

/// DFS Iterator for a [Tree]
pub struct DepthFirstIterator<'a, T>
where
    T: 'a + Eq + Clone + Distance + Hash,
{
    tree: &'a Tree<T>,
    stack: Vec<usize>,
}

impl<'a, T> DepthFirstIterator<'a, T>
where
    T: Eq + Clone + Distance + Hash,
{
    fn new(tree: &'a Tree<T>) -> Self {
        let mut stack = Vec::new();
        if !tree.nodes.is_empty() {
            // Root is always idx 0
            stack.push(0);
        }
        DepthFirstIterator { tree, stack }
    }
}

impl<'a, T> Iterator for DepthFirstIterator<'a, T>
where
    T: Eq + Clone + Distance + Hash,
{
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        self.stack.pop().map(|index| {
            // Children should be pushed onto the stack in reverse order to ensure left-most
            // are processed first
            for &child_index in self.tree.nodes[index].children.iter().rev() {
                self.stack.push(child_index);
            }
            &self.tree.nodes[index].value
        })
    }
}

/// Basic tree for use in search algorithms.
///
/// Provides functions for creating, growing, finding the nearest neighbors to `T`,
/// and rewiring the based on cost are provided.
/// Node values must be unique.
///
/// TODO: Make this a KD Tree?
/// TODO: Is a hashmap dumb?
#[derive(Debug)]
pub struct Tree<T>
where
    T: Eq + Clone + Distance + Hash,
{
    // Detailed node data for the tree.
    nodes: Vec<Node<T>>,

    // Support constant time lookup of nodes data with a value - node index map.
    nodes_map: HashMap<T, usize>,
}

impl<T: Eq + Clone + Distance + Hash> Tree<T> {
    /// Construct a new tree with the specified value as the root node.
    ///
    /// The node will take ownership of the provided value.
    pub fn new(val: T) -> Self {
        let mut nodes = Vec::new();
        let mut nodes_map = HashMap::new();

        // Construct root node and add it to storage
        let root_node = Node::new(val.clone(), None);
        nodes.push(root_node);
        nodes_map.insert(val, 0);

        Tree { nodes, nodes_map }
    }

    /// Adds the value to the specified node's children
    ///
    /// # Errors
    ///
    /// If the parent is not found in the tree.
    /// If the child is already in the tree.
    pub fn add_child(&mut self, parent: &T, child: T) -> Result<(), String> {
        // Cannot duplicate children
        if self.nodes_map.contains_key(&child) {
            return Err("The child is already in the tree".to_string());
        }

        if let Some(&parent_idx) = self.nodes_map.get(&parent) {
            // Append the child node to the nodes vector and note the location in the map.
            let child_idx = self.nodes.len();
            self.nodes.push(Node::new(child.clone(), Some(parent_idx)));
            self.nodes_map.insert(child, child_idx);
            self.nodes[parent_idx].children.push(child_idx);
        } else {
            return Err("The parent cannot be found in the tree".to_string());
        }

        Ok(())
    }

    // Return the size of the tree
    pub fn size(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the closest element to the specified value
    pub fn nearest_neighbor(&self, val: &T) -> &T {
        &self
            .nodes
            .iter()
            .min_by(|a, b| {
                let da = val.distance(&a.value);
                let db = val.distance(&b.value);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap()
            .value
    }

    /// Finds all nodes that are within the specified radius and sets the
    /// neighbors list accordingly. The node must already be in the tree.
    ///
    /// Returns the closet element to the specified value.
    ///
    /// # Errors
    ///
    /// If the provided `T` value is not in the tree.
    pub fn nearest_neighbors(&mut self, val: &T, radius: f64) -> Result<&T, String> {
        let node_idx: usize = *self
            .nodes_map
            .get(val)
            .ok_or("Specified value is not present in the tree".to_string())?;

        // First iterate over all nodes to identify all neighbors
        let mut min_distance = std::f64::MAX;
        let mut nearest_idx = node_idx;
        let mut neighbors = Vec::new();
        for (i, check) in self.nodes.iter().enumerate() {
            // Skip the current node
            if i == node_idx {
                continue;
            }

            // Compute and check distances
            let distance = val.distance(&check.value);
            if distance <= radius {
                neighbors.push(i);
            }
            if distance < min_distance {
                min_distance = distance;
                nearest_idx = i;
            }
        }

        // Then update neighbors sets and identify the nearest.
        for i in neighbors {
            print!("ERH TEMP: Node {node_idx} neighbor {i}\n");
            self.nodes[node_idx].neighbors.insert(i);
            self.nodes[i].neighbors.insert(node_idx);
        }

        Ok(&self.nodes[nearest_idx].value)
    }

    /// Returns a [DepthFirstIterator] for the tree
    pub fn iter_depth_first(&self) -> DepthFirstIterator<T> {
        DepthFirstIterator::new(self)
    }

    /// Returns a path to the root given the specified end point
    ///
    /// # Errors
    ///
    /// If the specified node is not found in the Tree
    pub fn path(&self, end: &T) -> Result<Vec<T>, String> {
        // Must be a valid node
        if !self.nodes_map.contains_key(&end) {
            return Err("Node is not present in tree".to_string());
        }

        // Build the path from end to beginning
        let mut path = Vec::new();

        // Loop until you get to the root
        let mut cur_idx = Some(self.nodes_map[&end]);
        while let Some(idx) = cur_idx {
            path.push(self.nodes[idx].value.clone());
            cur_idx = self.nodes[idx].parent;
        }

        // Reverse it to get the path in order
        path.reverse();
        Ok(path)
    }

    /// Returns the node with the specified value
    ///
    /// Returns None if the specified value is not in the tree.
    /// Only used for testing.
    #[allow(dead_code)]
    fn get_node(&self, val: &T) -> Option<&Node<T>> {
        self.nodes_map
            .get(val)
            .and_then(|&index| self.nodes.get(index))
    }
}

//
// Unit tests
//

// Needed for distancing points on a line
impl Distance for i32 {
    fn distance(&self, other: &Self) -> f64 {
        (self - other).abs().into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tree_children() {
        // Construct tree with a single node
        let mut tree: Tree<i32> = Tree::new(1);
        assert_eq!(tree.size(), 1);
        assert_eq!(tree.nodes[0].value, 1);

        // Add a child and make sure everything is ok
        assert!(tree.add_child(&1, 2).is_ok());
        assert_eq!(tree.size(), 2);

        // Make the tree bigger
        assert!(tree.add_child(&1, 3).is_ok());
        assert!(tree.add_child(&2, 4).is_ok());
        assert_eq!(tree.size(), 4);

        // Add an existing child and everything is not ok
        assert!(tree.add_child(&1, 2).is_err());

        // Add to a nonexistent parent and everything is not ok
        assert!(tree.add_child(&3, 2).is_err());
    }

    #[test]
    fn test_tree_get_nearest() {
        // Construct tree with many nodes
        let mut tree: Tree<i32> = Tree::new(1);

        assert!(tree.add_child(&1, 2).is_ok());
        assert!(tree.add_child(&1, 3).is_ok());
        assert!(tree.add_child(&2, 4).is_ok());
        assert!(tree.add_child(&2, 5).is_ok());
        assert!(tree.add_child(&2, 6).is_ok());

        // Make assertions
        assert_eq!(tree.nearest_neighbor(&7), &6);
        assert_eq!(tree.nearest_neighbor(&-1), &1);
        assert_eq!(tree.nearest_neighbor(&3), &3);
    }

    #[test]
    fn test_tree_dfs() {
        // Construct tree with many nodes
        let mut tree: Tree<i32> = Tree::new(1);

        assert!(tree.add_child(&1, 2).is_ok());
        assert!(tree.add_child(&1, 3).is_ok());
        assert!(tree.add_child(&2, 4).is_ok());
        assert!(tree.add_child(&2, 5).is_ok());
        assert!(tree.add_child(&3, 6).is_ok());

        // Expected order
        let expected_dfs_order = vec![1, 2, 4, 5, 3, 6];
        let dfs_order: Vec<i32> = tree.iter_depth_first().cloned().collect();

        // Compare
        assert_eq!(dfs_order, expected_dfs_order);
    }

    #[test]
    fn test_tree_compute_back_path() {
        // Construct tree with many nodes
        let mut tree: Tree<i32> = Tree::new(1);

        assert!(tree.add_child(&1, 2).is_ok());
        assert!(tree.add_child(&1, 3).is_ok());
        assert!(tree.add_child(&2, 4).is_ok());
        assert!(tree.add_child(&2, 5).is_ok());
        assert!(tree.add_child(&3, 7).is_ok());
        assert!(tree.add_child(&5, 6).is_ok());

        // Verify expected paths to different nodes
        let ep1 = vec![1, 2, 5, 6];
        let cp1 = tree.path(&6).unwrap();
        assert_eq!(cp1, ep1);

        let ep2 = vec![1, 3, 7];
        let cp2 = tree.path(&7).unwrap();
        assert_eq!(cp2, ep2);

        // Invalid node
        assert!(tree.path(&8).is_err());
    }

    #[test]
    fn test_tree_nearest_neighbors() {
        let mut tree: Tree<i32> = Tree::new(1);

        assert!(tree.add_child(&1, 2).is_ok());
        assert!(tree.add_child(&1, 4).is_ok());
        assert!(tree.add_child(&2, 5).is_ok());
        assert!(tree.add_child(&4, 7).is_ok());

        // 5 is the closest to 4.
        let nearest = *tree.nearest_neighbors(&4, 2.0).unwrap();
        assert_eq!(nearest, 5);

        // All neighbors should be updated
        let node_2_neighbors = &tree.get_node(&2).unwrap().neighbors;
        let node_4_neighbors = &tree.get_node(&4).unwrap().neighbors;
        let node_5_neighbors = &tree.get_node(&5).unwrap().neighbors;

        assert_eq!(node_2_neighbors.len(), 1);
        assert_eq!(node_4_neighbors.len(), 2);
        assert_eq!(node_5_neighbors.len(), 1);

        // We're operating on indexes, not values, this is a dumb way to validate since
        // we're not testing interfaces :/
        assert!(node_2_neighbors.contains(tree.nodes_map.get(&4).unwrap()));
        assert!(node_4_neighbors.contains(tree.nodes_map.get(&2).unwrap()));
        assert!(node_4_neighbors.contains(tree.nodes_map.get(&5).unwrap()));
        assert!(node_5_neighbors.contains(tree.nodes_map.get(&4).unwrap()));
    }
}
