/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map_utils.cpp
* \author   Collin Johnson
* 
* Definition of utility functions for interacting with the GlassMap:
* 
*   - is_cell_in_region
*   - active_region_in_flat_map
*/

#include "hssh/metrical/mapping/glass_map_utils.h"
#include <boost/algorithm/clamp.hpp>

namespace vulcan
{
namespace hssh
{

bool is_cell_in_region(Point<int> cell, const math::Rectangle<int>& region)
{
    return (cell.x >= region.bottomLeft.x)
        && (cell.x <  region.topRight.x)
        && (cell.y >= region.bottomLeft.y)
        && (cell.y <  region.topRight.y);
}


math::Rectangle<int> active_region_in_flat_map(const GlassMap& map)
{
    using namespace boost::algorithm;
    
    auto activeRegion = map.activeRegionInCells();
    auto offset = map.glassToFlatMapOffset();
    math::Rectangle<int> flatActiveRegion(activeRegion.bottomLeft + offset,
                                          activeRegion.topRight + offset);
    flatActiveRegion.bottomLeft.x = clamp(flatActiveRegion.bottomLeft.x, 0, static_cast<int>(map.flattenedMap().getWidthInCells()));
    flatActiveRegion.bottomLeft.y = clamp(flatActiveRegion.bottomLeft.y, 0, static_cast<int>(map.flattenedMap().getHeightInCells()));
    flatActiveRegion.topRight.x = clamp(flatActiveRegion.topRight.x, 
                                        flatActiveRegion.bottomLeft.x, 
                                        static_cast<int>(map.flattenedMap().getWidthInCells()));
    flatActiveRegion.topRight.y = clamp(flatActiveRegion.topRight.y, 
                                        flatActiveRegion.bottomLeft.y, 
                                        static_cast<int>(map.flattenedMap().getHeightInCells()));
    
    return flatActiveRegion;
}


math::angle_range_t angle_bins_to_angle_range(int x, int y, const GlassMap& map)
{
    if(!map.isCellInActiveRegion(x, y))
    {
        return math::angle_range_t();
    }

    double binAngleWidth = 2 * M_PI / map.numAngleBins();
    auto bin = map.beginBin(x, y);

    math::angle_range_t range;
    bool haveHit = false;

    for(int n = 0; n < map.numAngleBins(); ++n, ++bin)
    {
        if(*bin > 0)
        {
            double angle = map.binToAngle(n);

            // If the last bin wasn't a hit, then this is the start of a new angle range
            if(!haveHit)
            {
                range = math::angle_range_t(angle);
                range.expand(angle + binAngleWidth);
            }
            // Otherwise, we are continuing to expand an existing range
            else
            {
                // Add the whole bin
                range.expand(angle + binAngleWidth);
            }

            haveHit = true;
        }
    }

    return range;
}


double glass_cell_normal(int x, int y, const GlassMap& map)
{
    // Make sure the cell is active in order to process the normal
    if(!map.isCellInActiveRegion(x, y))
    {
        return 0.0;
    }

    return weighted_angle_sum(map.beginBin(x, y), map.endBin(x, y));
}

} // namespace hssh
} // namespace vulcan
