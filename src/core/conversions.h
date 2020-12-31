/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     conversions.h
* \author   Collin Johnson
*
* Declaration of helper functions for converting poses to/from other types.
*/

#ifndef CORE_CONVERSIONS_H
#define CORE_CONVERSIONS_H

#include "core/point.h"
#include "core/vector.h"

namespace vulcan
{

struct pose_t;

/**
* vector_to_pose converts a Vector to a pose_t. The format of the vector is assumed to be
* v(0) = x;
* v(1) = y;
* v(2) = theta;
*/
pose_t vector_to_pose(const Vector& v);

/**
* pose_to_vector converts the pose to a vector with the following format:
*
* v(0) = x;
* v(1) = y;
* v(2) = theta;
*/
Vector pose_to_vector(const pose_t& p);

}

#endif // CORE_CONVERSIONS_H
