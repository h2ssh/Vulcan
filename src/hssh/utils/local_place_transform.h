/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_place_transform.h
* \author   Collin Johnson
*
* Declaration of function for finding the transform from one local place to another.
*/

#ifndef HSSH_UTILS_LOCAL_PLACE_TRANSFORM_H
#define HSSH_UTILS_LOCAL_PLACE_TRANSFORM_H

#include "core/pose.h"

namespace vulcan
{
namespace hssh
{

class  LocalPlace;
struct local_path_fragment_t;

/**
* calculate_transform_to_reference_place takes a currently measured LocalPlace and finds the best
* transform to match a reference place. The reference is the local maximum of a simple gradient
* ascent scan matching with a scan generated from the current place into the reference place.
*
* The process for finding the transform is:
*
*   0) Generate a scan in the current place starting from the center.
*   1) Create an initial guess for the transform by comparing the vectors pointing to the
*      matching local_path_fragment_t. What is the transform for the current fragment
*      to match the reference fragment?
*   2) Do a gradient ascent search within the LPM of the reference place.
*
* Both of the fragments are assumed to be in a coordinate system relative to their respective place
* centers and thus the center of the fragment gateway can be considered as a vector pointing from
* the center of the place to the gateway.
*
* \param    currentPlace                Current place to be matched
* \param    referencePlace              Reference place to transform into
* \param    currentFragment             Fragment in current place that is matched with referenceFragment
* \param    referenceFragment           Fragment in the reference place
* \return   Best transform for a coordinate in the current place into the reference place.
*/
pose_t calculate_transform_to_reference_place(const LocalPlace&            currentPlace,
                                                     const LocalPlace&            referencePlace,
                                                     const local_path_fragment_t& currentFragment,
                                                     const local_path_fragment_t& referenceFragment);

}
}

#endif // HSSH_UTILS_LOCAL_PLACE_TRANSFORM_H
