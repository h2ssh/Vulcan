/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     config_file_utils.h
* \author   Collin Johnson
*
* A collection of helpful parsing tools for understanding the contents of a
* ConfigFile at a higher level of abstraction than the basic string or numeric
* types.
*/

#ifndef UTILS_CONFIG_FILE_UTILS_H
#define UTILS_CONFIG_FILE_UTILS_H

#include <string>
#include <vector>

namespace vulcan
{

struct pose_t;
struct pose_6dof_t;

template <typename T>
class Point;

namespace math
{
template <typename T>
class Rectangle;
}

namespace utils
{

/**
* create_point_vector_from_string creates a vector of points described
* by a string.
*
* Format: (point1.x,point1.y),(point2.x,point2.y),...
*/
std::vector<Point<float>> create_point_vector_from_string(const std::string& pointVectorString);

/**
* create_rectangle_from_string creates a rectangle from a collection of points described
* by a string.
*
* Format: (bottomLeft.x,bottomLeft.y),(topRight.x,topRight.y)
*
* The created rectangle can only be axis-aligned.
*/
math::Rectangle<float> create_rectangle_from_string(const std::string& rectangleString);

/**
* create_point_from_string creates a point from a string description
*
* Format: (x,y)
*/
Point<float> create_point_from_string(const std::string& pointString);

/**
* create_pose_from_string creates a pose_t from a string description.
*
* Format:  (x,y,theta)
*/
pose_t create_pose_from_string(const std::string& poseString);

/**
* create_pose_6dof_from_string creates a pose_6dof_t from a string description.
*
* Format: (x,y,z,phi,rho,theta)
*/
pose_6dof_t create_pose_6dof_from_string(const std::string& poseString);

/**
* split_into_strings splits a string into many smaller strings based on a split character.
*
* \param    str         String to split
* \param    split       Character to split on
* \return   Vector of strings separated by split.
*/
std::vector<std::string> split_into_strings(const std::string& str, char split);

}
}

#endif // UTILS_CONFIG_FILE_UTILS_H
