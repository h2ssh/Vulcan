/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     icp.h
* \author   Collin Johnson
* 
* Declaration of functions for performing Iterative Closest Point matching
* between two collections of points:
* 
*   - icp_2d : ICP for 2D range scan
*/

#ifndef UTILS_ICP_H
#define UTILS_ICP_H

#include <core/pose.h>
#include <vector>

namespace vulcan
{
namespace math  { template <typename T> class Point; }
namespace utils
{
    
/**
* icp_2d implements a version of the iterative closest point algorithm to match two 2d range scans.
* The algorithm finds the best transform from the 'from' points to the 'to points. An initial guess
* of the transform can be provided to help speed convergence and increase the likelihood of convergence.
* 
* The transform to be applied is: position + rotation*point
* 
* \param    from            Initial set of points
* \param    to              Next set of points
* \param    initial         Initial guess at the transform
* \return   Best transform found for matching 'from' points to 'to' points.
*/
pose_t icp_2d(const std::vector<Point<float>>& from,
                     const std::vector<Point<float>>& to,
                     const pose_t&                   initial = pose_t(0.0f, 0.0f, 0.0f));
    
}
}

#endif // UTILS_ICP_H
