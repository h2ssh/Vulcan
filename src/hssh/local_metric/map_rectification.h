/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     map_rectification.h
* \author   Collin Johnson
*
* Declaration of functions for map rectification:
*
*   - calculate_map_orientation
*   - rotate_lpm
*/

#ifndef HSSH_LOCAL_METRIC_MAP_RECTIFICATION_H
#define HSSH_LOCAL_METRIC_MAP_RECTIFICATION_H

namespace vulcan
{
namespace hssh
{

class LocalPerceptualMap;

/**
* calculate_map_orientation determines the orientation of an LPM by calculating the first and second
* moments of the free space in the map. The formula for determining the orientation is taken from
* Horn's "Robot Vision" somewhere in Chapter 3.
*
* \param    lpm         LPM to be evaluated
* \return   Orientation of the x-axis in the LPM.
*/
float calculate_map_orientation(const LocalPerceptualMap& lpm);

/**
* rotate_lpm rotates every cell in the map by the specified angle. The LPM won't be resized, so some
* information will be lost. A new LPM is calculated from the old LPM.
*
* \param    lpm         LPM to be rotated
* \param    angle       Angle by which to rotate
* \return   A new LPM with each cell rotated by the specified angle.
*/
LocalPerceptualMap rotate_lpm(const LocalPerceptualMap& lpm, float angle);

}
}

#endif // HSSH_LOCAL_METRIC_MAP_RECTIFICATION_H
