/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_utils.h
* \author   Collin Johnson
*
* Declaration of the following functions:
*
*   - change_lpm_reference_frame
*/

#ifndef HSSH_UTILS_LPM_UTILS_H
#define HSSH_UTILS_LPM_UTILS_H

namespace vulcan
{
struct pose_t;

namespace hssh
{

class LocalPerceptualMap;

/**
* change_lpm_reference_frame updates the provided LPM to the new reference frame. The transform
* between the current frame and the new reference frame is provided. To change the reference
* frame two steps are required:
*
*   1) Rotate the map to the new orientation (-reference.theta)
*   2) Update the location of the bottom left corner of the map in the new reference frame.
*
* \param[in]    lpm                 Map to be transformed
* \param[in]    newReference        Location of the new reference frame in the current LPM
* \return   A new LPM representing in the new reference frame.
*/
LocalPerceptualMap change_lpm_reference_frame(const LocalPerceptualMap& lpm, const pose_t& newReference);

}
}

#endif // HSSH_UTILS_LPM_UTILS_H
