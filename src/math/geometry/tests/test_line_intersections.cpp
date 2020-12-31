/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// #include <iostream>
// #include <vector>
// #include "core/point.h"
// #include "core/line.h"
// #include "math/geometry/line_intersections.h"
// #include "core/float_comparison.h"
//
//
// using namespace vulcan;
// using namespace vulcan::math;
//
//
// bool test_diagonal_parallel_lines(void);
// bool test_horizonal_parallel_lines(void);
// bool test_vertical_parallel_lines(void);
// bool test_x(void);
// bool test_plus(void);
// bool test_overlapping(void);
// bool test_same_start_point(void);
// bool test_same_end_point(void);
// bool test_complicated(void);
//
// void print_intersections(const std::vector<intersection_point_t>& intersections);
//
//
// /**
// * line_intersections_test runs a number of tests for checking whether the
// * find_line_intersections() code is functioning properly. Simple cases, like
// * parallel lines and vertical lines are checked, along with more complicated
// * cases of overlapping lines, and lines that meet at their endpoints.
// */
// int main(int argc, char** argv)
// {
//     int testCount = 0;
//     int passCount = 0;
//
//     if(test_diagonal_parallel_lines())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_horizonal_parallel_lines())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_vertical_parallel_lines())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_x())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_plus())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_overlapping())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_same_start_point())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_same_end_point())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     if(test_complicated())
//     {
//         ++passCount;
//     }
//     ++testCount;
//
//     std::cout<<"Results: "<<passCount<<'/'<<testCount<<" tests passed.\n";
//
//     return 0;
// };
//
//
// bool test_diagonal_parallel_lines(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(0, 0);
//     lines[0].b = Point<double>(0.5, 0.5);
//
//     lines[1].a = Point<double>(0, -0.5);
//     lines[1].b = Point<double>(0.5, 0);
//
//     std::cout<<"STARTING: diagonal_parallel_lines\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(!intersections.empty())
//     {
//         std::cout<<"FAILED: diagonal_parallel_lines: Intersection between parallel diagonal lines\n";
//         print_intersections(intersections);
//         return false;
//     }
//
//     std::cout<<"PASSED: diagonal_parallel_lines\n";
//
//     return true;
// }
//
//
// bool test_horizonal_parallel_lines(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(1, 1);
//     lines[0].b = Point<double>(2, 1);
//
//     lines[1].a = Point<double>(1, 1.01);
//     lines[1].b = Point<double>(2, 1.01);
//
//     std::cout<<"STARTING: horizontal_parallel_lines\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(!intersections.empty())
//     {
//         std::cout<<"FAILED: horizontal_parallel_lines: Intersection between parallel horizontal lines\n";
//         print_intersections(intersections);
//         return false;
//     }
//
//     std::cout<<"PASSED: horizontal_parallel_lines\n";
//
//     return true;
// }
//
//
// bool test_vertical_parallel_lines(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(1, 1);
//     lines[0].b = Point<double>(1, 2);
//
//     lines[1].a = Point<double>(0.99, 0);
//     lines[1].b = Point<double>(0.99, 3);
//
//     std::cout<<"STARTING: vertical_parallel_lines\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(!intersections.empty())
//     {
//         std::cout<<"FAILED: vertical_parallel_lines: Intersection between parallel vertical lines\n";
//         print_intersections(intersections);
//         return false;
//     }
//
//     std::cout<<"PASSED: vertical_parallel_lines\n";
//
//     return true;
// }
//
//
// bool test_x(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(-0.5, -0.5);
//     lines[0].b = Point<double>(0.5, 0.5);
//
//     lines[1].a = Point<double>(-0.5, 0.5);
//     lines[1].b = Point<double>(0.5, -0.5);
//
//     std::cout<<"STARTING: x\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(intersections.empty())
//     {
//         std::cout<<"FAILED: x: No intersections found\n";
//         return false;
//     }
//     else if(intersections.size() > 1)
//     {
//         std::cout<<"FAILED: x: Too many intersections. Found: "<<intersections.size()<<" Expected: 1\n";
//         print_intersections(intersections);
//         return false;
//     }
//     else if(!absolute_fuzzy_equal(intersections[0].intersection.x, 0.0f) ||
//             !absolute_fuzzy_equal(intersections[0].intersection.y, 0.0f))
//     {
//         std::cout<<"FAILED: x: Wrong intersection point. Found: "<<intersections[0].intersection<<" Expected: (0, 0)\n";
//         return false;
//     }
//
//     std::cout<<"PASSED: x\n";
//
//     return true;
// }
//
//
// bool test_plus(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(0,  1);
//     lines[0].b = Point<double>(0, -1);
//
//     lines[1].a = Point<double>(-1, 0);
//     lines[1].b = Point<double>(1,  0);
//
//     std::cout<<"STARTING: plus\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(intersections.empty())
//     {
//         std::cout<<"FAILED: plus: No intersections found\n";
//         return false;
//     }
//     else if(intersections.size() > 1)
//     {
//         std::cout<<"FAILED: plus: Too many intersections. Found: "<<intersections.size()<<" Expected: 1\n";
//         print_intersections(intersections);
//         return false;
//     }
//     else if(!absolute_fuzzy_equal(intersections[0].intersection.x, 0.0f) ||
//             !absolute_fuzzy_equal(intersections[0].intersection.y, 0.0f))
//     {
//         std::cout<<"FAILED: plus: Wrong intersection point. Found: "<<intersections[0].intersection<<" Expected: (0, 0)\n";
//         return false;
//     }
//
//     std::cout<<"PASSED: plus\n";
//
//     return true;
// }
//
//
// bool test_overlapping(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(-0.5, -0.5);
//     lines[0].b = Point<double>(0.5, 0.5);
//
//     lines[1].a = Point<double>(-0.5, 0.5);
//     lines[1].b = Point<double>(0.5, -0.5);
//
//     std::cout<<"STARTING: overlapping\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(intersections.empty())
//     {
//         std::cout<<"FAILED: same_start_point: No intersections found\n";
//         return false;
//     }
//     else if(intersections.size() > 1)
//     {
//         std::cout<<"FAILED: same_start_point: Too many intersections. Found: "<<intersections.size()<<" Expected: 1\n";
//         print_intersections(intersections);
//         return false;
//     }
//
//     // TODO: Determine method for easily checking the correct points, rather than just the number of points
//
//     std::cout<<"PASSED: overlapping\n";
//     print_intersections(intersections);
//
//     return true;
// }
//
//
// bool test_same_start_point(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(1.5, 1.5);
//     lines[0].b = Point<double>(0.5, 0.5);
//
//     lines[1].a = Point<double>(1.5, 1.5);
//     lines[1].b = Point<double>(0.5, -0.5);
//
//     std::cout<<"STARTING: same_start_point\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(intersections.empty())
//     {
//         std::cout<<"FAILED: same_start_point: No intersections found\n";
//         return false;
//     }
//     else if(intersections.size() > 1)
//     {
//         std::cout<<"FAILED: same_start_point: Too many intersections. Found: "<<intersections.size()<<" Expected: 1\n";
//         print_intersections(intersections);
//         return false;
//     }
//     else if(!absolute_fuzzy_equal(intersections[0].intersection.x, 1.5f) ||
//             !absolute_fuzzy_equal(intersections[0].intersection.y, 1.5f))
//     {
//         std::cout<<"FAILED: same_start_point: Wrong intersection point. Found: "<<intersections[0].intersection<<" Expected: (1.5, 1.5)\n";
//         return false;
//     }
//
//     std::cout<<"PASSED: same_start_point\n";
//
//     return true;
// }
//
//
// bool test_same_end_point(void)
// {
//     std::vector<Line<double>> lines(2);
//     lines[0].a = Point<double>(-1.5, -1.5);
//     lines[0].b = Point<double>(0.5, 0.5);
//
//     lines[1].a = Point<double>(-1.5, -1.5);
//     lines[1].b = Point<double>(0.5, -0.5);
//
//     std::cout<<"STARTING: same_end_point\n";
//
//     std::vector<intersection_point_t> intersections = find_line_intersections(lines);
//
//     if(intersections.empty())
//     {
//         std::cout<<"FAILED: same_end_point: No intersections found\n";
//         return false;
//     }
//     else if(intersections.size() > 1)
//     {
//         std::cout<<"FAILED: same_end_point: Too many intersections. Found: "<<intersections.size()<<" Expected: 1\n";
//         print_intersections(intersections);
//         return false;
//     }
//     else if(!absolute_fuzzy_equal(intersections[0].intersection.x, -1.5f) ||
//             !absolute_fuzzy_equal(intersections[0].intersection.y, -1.5f))
//     {
//         std::cout<<"FAILED: same_end_point: Wrong intersection point. Found: "<<intersections[0].intersection<<" Expected: (-1.5, -1.5)\n";
//         return false;
//     }
//
//     std::cout<<"PASSED: same_end_point\n";
//
//     return true;
// }
//
//
// bool test_complicated(void)
// {
//     std::cout<<"WARNING: test_complicated unimplemented\n";
//
//     return true;
// }
//
//
// void print_intersections(const std::vector<intersection_point_t>& intersections)
// {
//     std::cout<<"INFO: Intersections:\n";
//     for(auto intersectionIt = intersections.begin(), intersectionEnd = intersections.end(); intersectionIt != intersectionEnd; ++intersectionIt)
//     {
//         std::cout<<intersectionIt->intersection<<'\n';
//     }
// }
