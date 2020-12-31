/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     conversions.cpp
 * \author   Collin Johnson
 *
 * Definition of various pose conversion functions.
 */

#include "core/conversions.h"
#include "core/pose.h"

namespace vulcan
{

pose_t vector_to_pose(const Vector& v)
{
    pose_t pose;

    pose.x = v(0);
    pose.y = v(1);
    pose.theta = v(2);

    return pose;
}


Vector pose_to_vector(const pose_t& p)
{
    Vector poseVector(3);

    poseVector(0) = p.x;
    poseVector(1) = p.y;
    poseVector(2) = p.theta;

    return poseVector;
}

}   // namespace vulcan
