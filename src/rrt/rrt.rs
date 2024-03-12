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

// Qgoal //region that identifies success
// Counter = 0 //keeps track of iterations
// lim = n //number of iterations algorithm should run for
// G(V,E) //Graph containing edges and vertices, initialized as empty
// While counter < lim:
//     Xnew  = RandomPosition()
//     if IsInObstacle(Xnew) == True:
//         continue
//     Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
//     Link = Chain(Xnew,Xnearest)
//     G.append(Link)
//     if Xnew in Qgoal:
//         Return G
// Return G

use crate::tree::Distance;
use crate::tree::Tree;
use std::hash::Hash;

/// Basic RRT implementation.
pub fn rrt<T, FR, FV, FN, FS>(
    start: &T,
    mut sample: FR,
    mut is_valid: FV,
    mut success: FS,
) -> Option<Vec<T>>
where
    T: Eq + Clone + Hash + Distance,
    FR: FnMut() -> T,
    FV: FnMut(&T) -> bool,
    FS: FnMut(&T) -> bool,
{
    Some(Vec::new())
}
