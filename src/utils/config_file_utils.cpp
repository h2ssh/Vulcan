/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     config_file_utils.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of helpful parsing functions for various types used in config files.
*/

#include <core/point.h>
#include <math/geometry/rectangle.h>
#include <core/pose.h>
#include <utils/config_file_utils.h>

namespace vulcan
{
namespace utils
{

std::vector<float> parse_number_list(const std::string& numberList);


std::vector<Point<float>> create_point_vector_from_string(const std::string& pointVectorString)
{
    std::vector<Point<float>> pointVector;

    size_t pointStart = 0;
    size_t pointEnd   = 0;
    
    size_t stringEnd = pointVectorString.rfind(')');

    while(pointEnd != stringEnd)
    {
        pointStart = pointVectorString.find('(', pointEnd);
        pointEnd   = pointVectorString.find(')', pointStart);

        pointVector.push_back(create_point_from_string(pointVectorString.substr(pointStart+1, pointEnd-pointStart-1)));
    }

    return pointVector;
}


math::Rectangle<float> create_rectangle_from_string(const std::string& rectangleString)
{
    // Order is: (bottom,left), (top,right)

    Point<float> points[2];

    size_t pointStart = 0;
    size_t pointEnd   = 0;

    for(int i = 0; i < 2; ++i)
    {
        pointStart = rectangleString.find('(', pointEnd);
        pointEnd   = rectangleString.find(')', pointStart);

        points[i] = create_point_from_string(rectangleString.substr(pointStart+1, pointEnd-pointStart-1));
    }

    return math::Rectangle<float>(points[0], points[1]);
}


Point<float> create_point_from_string(const std::string& pointString)
{
    Point<float> point;

    std::vector<float> numbers = parse_number_list(pointString);

    point.x = numbers[0];
    point.y = numbers[1];

    return point;
}


pose_t create_pose_from_string(const std::string& poseString)
{
    std::vector<float> numbers = parse_number_list(poseString);

    pose_t pose;

    pose.x     = numbers[0];
    pose.y     = numbers[1];
    pose.theta = numbers[2];

    return pose;
}


pose_6dof_t create_pose_6dof_from_string(const std::string& poseString)
{
    std::vector<float> numbers = parse_number_list(poseString);

    pose_6dof_t pose;

    pose.x     = numbers[0];
    pose.y     = numbers[1];
    pose.z     = numbers[2];
    pose.phi   = numbers[3];
    pose.rho   = numbers[4];
    pose.theta = numbers[5];

    return pose;
}


std::vector<std::string> split_into_strings(const std::string& str, char split)
{
    std::vector<std::string> strings;

    std::string::size_type startPos = 0;
    std::string::size_type splitPos = 0;

    do
    {
        startPos = splitPos;
        splitPos = str.find(split, startPos);

        if(splitPos != std::string::npos)
        {
            if(startPos != splitPos)
            {
                strings.push_back(str.substr(startPos, splitPos - startPos));
            }
            ++splitPos;
        }
        else
        {
            if(str[startPos] != split)
            {
                strings.push_back(str.substr(startPos));
            }
        }
    } while(splitPos != std::string::npos);

    return strings;
}


std::vector<float> parse_number_list(const std::string& numberList)
{
    const std::string NUMBERS("0123456789.-");

    std::vector<float> numbers;

    size_t numberStart = numberList.find_first_of(NUMBERS);

    while(numberStart != std::string::npos)
    {
        size_t numberEnd = numberList.find_first_not_of(NUMBERS, numberStart);

        if(numberEnd != std::string::npos)
        {
            numbers.push_back(atof(numberList.substr(numberStart, numberEnd-numberStart).c_str()));
        }
        else
        {
            numbers.push_back(atof(numberList.substr(numberStart).c_str()));
        }

        numberStart = numberList.find_first_of(NUMBERS, numberEnd);
    }

    return numbers;
}

}
}
