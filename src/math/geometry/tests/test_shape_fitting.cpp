/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// #include <iostream>
// #include <math/geometry/shape_fitting.h>
//
//
// using namespace vulcan::math;
//
//
// bool test_basic_square        (void);
// bool test_basic_rectangle     (void);
// bool test_rotated_rectangle   (void);
// bool test_right_triangle      (void);
// bool test_equilateral_triangle(void);
// bool test_pentagon            (void);
//
// bool has_vertex_at(const Rectangle<float>& boundary, const Point<float>& vertex);
//
// void print_rectangle(const Rectangle<float>& boundary);
//
//
// /**
// * bounding_rectangle_test tests the minimum_area_bounding_rectangle function to see that
// * it produces reasonable results.
// */
// int main(int argc, char** argv)
// {
//     std::cout<<"bounding_rectangle_test: Beginning tests...\n";
//
//     int totalTests  = 0;
//     int passedTests = 0;
//
//     ++totalTests;
//     std::cout<<"test_basic_square:\n";
//     if(test_basic_square())
//     {
//         std::cout<<"Passed\n";
//         ++passedTests;
//     }
//     else
//     {
//         std::cout<<"Failed\n";
//     }
//
//     ++totalTests;
//     std::cout<<"test_basic_rectangle:\n";
//     if(test_basic_rectangle())
//     {
//         std::cout<<"Passed\n";
//         ++passedTests;
//     }
//     else
//     {
//         std::cout<<"Failed\n";
//     }
//
//     ++totalTests;
//     std::cout<<"test_rotated_rectangle:\n";
//     if(test_rotated_rectangle())
//     {
//         std::cout<<"Passed\n";
//         ++passedTests;
//     }
//     else
//     {
//         std::cout<<"Failed\n";
//     }
//
//     ++totalTests;
//     std::cout<<"test_right_triangle:\n";
//     if(test_right_triangle())
//     {
//         std::cout<<"Passed\n";
//         ++passedTests;
//     }
//     else
//     {
//         std::cout<<"Failed\n";
//     }
//
//     ++totalTests;
//     std::cout<<"test_equilateral_triangle:\n";
//     if(test_equilateral_triangle())
//     {
//         std::cout<<"Passed\n";
//         ++passedTests;
//     }
//     else
//     {
//         std::cout<<"Failed\n";
//     }
//
//     ++totalTests;
//     std::cout<<"test_pentagon:\n";
//     if(test_pentagon())
//     {
//         std::cout<<"Passed\n";
//         ++passedTests;
//     }
//     else
//     {
//         std::cout<<"Failed\n";
//     }
//
//     std::cout<<"Finished testing: "<<passedTests<<'\\'<<totalTests<<'\n';
//
//     return 0;
// }
//
//
// bool test_basic_square(void)
// {
//     std::vector<Point<float>> vertices;
//     vertices.push_back(Point<float>(0, 0));
//     vertices.push_back(Point<float>(-1, 0));
//     vertices.push_back(Point<float>(-1, 1));
//     vertices.push_back(Point<float>(0, 1));
//
//     Rectangle<float> boundary = minimum_area_bounding_rectangle(vertices);
//
//     if(!has_vertex_at(boundary, Point<float>(0, 0))  ||
//        !has_vertex_at(boundary, Point<float>(-1, 0)) ||
//        !has_vertex_at(boundary, Point<float>(-1, 1)) ||
//        !has_vertex_at(boundary, Point<float>(0, 1)))
//     {
//         std::cout<<"ERROR: basic_square:";
//         print_rectangle(boundary);
//         std::cout<<'\n';
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// }
//
//
// bool test_basic_rectangle(void)
// {
//     std::vector<Point<float>> vertices;
//     vertices.push_back(Point<float>(0, 0));
//     vertices.push_back(Point<float>(-2, 0));
//     vertices.push_back(Point<float>(-2, 1));
//     vertices.push_back(Point<float>(0, 1));
//
//     Rectangle<float> boundary = minimum_area_bounding_rectangle(vertices);
//
//     if(!has_vertex_at(boundary, Point<float>(0, 0))  ||
//         !has_vertex_at(boundary, Point<float>(-2, 0)) ||
//         !has_vertex_at(boundary, Point<float>(-2, 1)) ||
//         !has_vertex_at(boundary, Point<float>(0, 1)))
//     {
//         std::cout<<"ERROR: basic_rectangle:";
//         print_rectangle(boundary);
//         std::cout<<'\n';
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// }
//
//
// bool test_rotated_rectangle(void)
// {
//     std::vector<Point<float>> vertices;
//     vertices.push_back(Point<float>(0, 0));
//     vertices.push_back(Point<float>(-2, 0));
//     vertices.push_back(Point<float>(0, 1));
//     vertices.push_back(Point<float>(0, 2));
//
//     Rectangle<float> boundary = minimum_area_bounding_rectangle(vertices);
//
//     if(!has_vertex_at(boundary, Point<float>(0, 0))  ||
//         !has_vertex_at(boundary, Point<float>(-2, 0)) ||
//         !has_vertex_at(boundary, Point<float>(-2, 1)) ||
//         !has_vertex_at(boundary, Point<float>(0, 1)))
//     {
//         std::cout<<"ERROR: rotated_rectangle:";
//         print_rectangle(boundary);
//         std::cout<<'\n';
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// }
//
//
// bool test_right_triangle(void)
// {
//     return true;
// }
//
//
// bool test_equilateral_triangle(void)
// {
//     return true;
// }
//
//
// bool test_pentagon(void)
// {
//     return true;
// }
//
//
// bool has_vertex_at(const Rectangle<float>& boundary, const Point<float>& vertex)
// {
//     return (boundary.topRight    == vertex) ||
//            (boundary.topLeft     == vertex) ||
//            (boundary.bottomRight == vertex) ||
//            (boundary.bottomLeft  == vertex);
// }
//
//
// void print_rectangle(const Rectangle<float>& boundary)
// {
//     std::cout<<boundary.topRight<<','<<boundary.bottomRight<<','<<boundary.bottomLeft<<','<<boundary.topLeft;
// }
