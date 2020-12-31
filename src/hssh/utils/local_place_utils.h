/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_place_utils.h
 * \author   Collin Johnson
 *
 * Declaration of utility functions for operating on LocalPlaces:
 *
 *   LocalPlace rotate_local_place(LocalPlace, theta)
 */

#ifndef HSSH_UTILS_LOCAL_PLACE_UTILS_H
#define HSSH_UTILS_LOCAL_PLACE_UTILS_H

namespace vulcan
{
struct pose_t;

namespace hssh
{

class LocalPlace;

/**
 * rotate_local_place takes a LocalPlace and rotates all objects within it by the specified
 * angle. The LPM and gateways for the SmallScaleStar are rotated accordingly.
 *
 * The place will be rotated by transform.theta. The center of the place will be set to
 * (transform.x, transform.y).
 *
 * \param    place           Original place to be transform
 * \param    newId           Id for the new place
 * \param    transform       Transform to apply to the place
 * \return   New LocalPlace with all contents rotated by theta.
 */
LocalPlace transform_local_place(const LocalPlace& place, uint32_t newId, const pose_t& transform);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_LOCAL_PLACE_UTILS_H
