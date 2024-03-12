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
use std::collections::HashMap;
use std::hash::Hash;

/// Basic node element for the tree.
///
/// Stores the value of the node and all children in a mut hash map to support constant
/// time lookups by value.
#[derive(Debug)]
struct TreeNode<T> {
    value: T,

    // Maintains a list of pointers to the children's location in the parent's vector
    children: Vec<usize>,
}

impl<T> TreeNode<T> {
    fn new(val: T) -> Self {
        TreeNode {
            value: val,
            children: Vec::new(),
        }
    }

    // Add a child to this node's children. Uniqueness must be enforced by the caller.
    fn add_child(&mut self, child: usize) {
        self.children.push(child);
    }
}

/// Define a distance trait for tree node values.
pub trait Distance {
    fn distance(&self, other: &Self) -> f64;
}

/// Basic tree for use in search algorithms.
///
/// Provides functions for creating, growing, and finding the nearest neighbor to `T`.
/// Node values must be unique.
///
/// TODO: Make this a KD Tree?
/// TODO: Is a hashmap dumb?
#[derive(Debug)]
pub struct Tree<T>
where
    T: Eq + Copy + Distance + Hash,
{
    // Store nodes in a vector to support easy iteration and growth of the tree.
    // The root will always be at idx 0.
    nodes: Vec<TreeNode<T>>,

    // Support constant time lookup of nodes by value
    nodes_map: HashMap<T, usize>,
}

impl<T: Eq + Copy + Distance + Hash> Tree<T> {
    pub fn new(val: T) -> Self {
        let root_idx: usize = 0;
        let root_node = TreeNode::new(val);
        let mut nodes = Vec::new();
        let mut nodes_map = HashMap::new();

        nodes.push(root_node);
        nodes_map.insert(val, root_idx);

        Tree {
            nodes: nodes,
            nodes_map: nodes_map,
        }
    }

    /// Adds the value to the specified node's children
    ///
    /// # Errors
    ///
    /// If the parent is not found in the tree.
    /// If the child is already in the tree.
    pub fn add_child(&mut self, parent: T, child: T) -> Result<(), String> {
        // Cannot ad
        if self.nodes_map.contains_key(&child) {
            return Err("The child is already in the tree".to_string());
        }

        if let Some(&parent_idx) = self.nodes_map.get(&parent) {
            // Append the child node to the nodes vector and note the location in the map.
            let child_idx = self.nodes.len();
            self.nodes.push(TreeNode::new(child));
            self.nodes_map.insert(child, child_idx);
            self.nodes[parent_idx].add_child(child_idx);
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
    pub fn nearest(&self, val: T) -> T {
        self.nodes
            .iter()
            .min_by(|a, b| {
                let da = val.distance(&a.value);
                let db = val.distance(&b.value);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap()
            .value
    }
}

//
// Unit tests
//

// Needed for distancing points on a plane
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
        assert!(tree.add_child(1, 2).is_ok());
        assert_eq!(tree.size(), 2);
        assert_eq!(tree.nodes[1].value, 2);
        assert_eq!(tree.nodes[tree.nodes_map[&1]].children, vec![1]);

        // Make the tree bigger
        assert!(tree.add_child(1, 3).is_ok());
        assert!(tree.add_child(2, 4).is_ok());
        assert_eq!(tree.size(), 4);

        // Add an existing child and everything is not ok
        assert!(tree.add_child(1, 2).is_err());

        // Add to a nonexistent parent and everything is not ok
        assert!(tree.add_child(3, 2).is_err());
    }

    #[test]
    fn test_tree_get_nearest() {
        // Construct tree with many nodes
        let mut tree: Tree<i32> = Tree::new(1);

        assert!(tree.add_child(1, 2).is_ok());
        assert!(tree.add_child(1, 3).is_ok());
        assert!(tree.add_child(2, 4).is_ok());
        assert!(tree.add_child(2, 5).is_ok());
        assert!(tree.add_child(2, 6).is_ok());

        // Make assertions
        assert_eq!(tree.nearest(7), 6);
        assert_eq!(tree.nearest(-1), 1);
        assert_eq!(tree.nearest(3), 3);
    }
}
