/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     place_extent.h
* \author   Collin Johnson
*
* Declaration of place_extent_t.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_PLACE_EXTENT_H
#define HSSH_LOCAL_TOPOLOGICAL_PLACE_EXTENT_H

#include "math/geometry/polygon.h"
#include "math/geometry/rectangle.h"

namespace vulcan
{
namespace hssh
{

/**
* place_extent_t defines the properties of the extent of a place. The extent of a place is the portion
* of the LPM the place occupies. The extent itself is a rectangular bounding box. The properties of the
* extent give the proportion of the boundary that is a gateway, occupied cell, or frontier cells. These values
* provide the explored status of the place.
*/
struct place_extent_t
{
    math::Rectangle<float> boundary;                ///< Rectangle drawn around the polygon boundary -- corresponds to section of LPM containing the place
    math::Polygon<float>   polygonBoundary;         ///< Polygon formed by rays traced from the center of the place until hitting frontier, gateway, or occupied cell

    // These percentages define the makeup of the boundary of the place as % of rays hitting the various types of cells
    float gatewayPercent;
    float occupiedPercent;
    float frontierPercent;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_PLACE_EXTENT_H
