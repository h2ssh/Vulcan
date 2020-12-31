/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// #include <iostream>
// #include <vector>
// #include "math/geometry/convex_hull.h"
//
//
// using namespace vulcan;
//
//
// int main(int argc, char** argv)
// {
//     std::vector<Point<float>> points(5);
//
//     points[0] = Point<float>(0, 0);
//     points[1] = Point<float>(1, 0);
//     points[2] = Point<float>(1, 1);
//     points[3] = Point<float>(0.5, 0.5);
//     points[4] = Point<float>(0, 1);
//
//     math::Polygon<float> hull = math::convex_hull(points.begin(), points.end());
//
//     std::cout<<"Hull:";
//     for(auto pointIt = hull.vertices.begin(), endIt = hull.vertices.end(); pointIt != endIt; ++pointIt)
//     {
//         std::cout<<*pointIt<<(pointIt != endIt-1 ? "->" : "\n");
//     }
//
//     points.resize(6);
//
//     points[0] = Point<float>(0, 0);
//     points[1] = Point<float>(1, 0);
//     points[2] = Point<float>(1, 1);
//     points[3] = Point<float>(0.5, 0.5);
//     points[4] = Point<float>(0, 1);
//     points[5] = Point<float>(0.5, 0);
//
//     hull = math::convex_hull(points.begin(), points.end());
//
//     std::cout<<"Hull:";
//     for(auto pointIt = hull.vertices.begin(), endIt = hull.vertices.end(); pointIt != endIt; ++pointIt)
//     {
//         std::cout<<*pointIt<<(pointIt != endIt-1 ? "->" : "\n");
//     }
//
//     return 0;
// }
