/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map_utils.h
* \author   Collin Johnson
* 
* Declaration of utility functions for interacting with the GlassMap:
* 
*   - is_cell_in_region : check if a cell falls in the specified active region
*   - active_region_in_flat_map : get the active region cells in the flat map that correspond to the active region in
*       the glass map 
*/

#ifndef HSSH_METRICAL_MAPPING_GLASS_MAP_UTILS_H
#define HSSH_METRICAL_MAPPING_GLASS_MAP_UTILS_H

#include <hssh/metrical/glass_map.h>
#include <math/angle_range.h>
#include <math/geometry/rectangle.h>
#include <cmath>

namespace vulcan
{
namespace hssh 
{

/**
* is_cell_in_region checks if a cell is contained in the defined region.
*/
bool is_cell_in_region(Point<int> cell, const math::Rectangle<int>& region);

/**
* active_region_in_flat_map determines the active region cells in the flat map that correspond to the active region in
* the glass map. The active region in the glass map is the action glass region with the glassToFlatOffset applied.
* 
* \param    map         GlassMap in which to get the active region
* \return   Active region in grid coordinates of the active region of the flat map.
*/
math::Rectangle<int> active_region_in_flat_map(const GlassMap& map);

/**
* angle_bins_to_angle_range converts angle bins in the GlassMap to an angle range. The requested cell must be in the
* current active region of the map to successfully create a new range. Otherwise, an empty range is returned.
* 
* The angle range is the complete range of angles in which a hit was registered. It isn't a contiguous hit range though, 
* meaning there might be some free or unknown bins between the hits. Given the nature of the grid, where not all angles 
* are viewed for a given cell, this approach gives a better approximation of the range from which a particular cell
* is seen.
*
* \param    x           x-coordinate of the cell
* \param    y           y-coordinate of the cell
* \return   Largest viewable angle range for the given cell. If there is no viewable region, the returned value will
*   have an extent of 0.
*/
math::angle_range_t angle_bins_to_angle_range(int x, int y, const GlassMap& map);

/**
* glass_cell_normal estimates the normal for the given cell (if it is in the active region), by finding the weighted
* mean of all hits in the cell's angle bins.
*
* \param    x           x-coordinate of the cell
* \param    y           y-coordinate of the cell
* \return   Normal of the largest viewable region of the cell.
*/
double glass_cell_normal(int x, int y, const GlassMap& map);

/**
* weighted_angle_sum computes the weighted sum of angle bins. It is assumed the bins span 360 degrees and are evenly
* distributed around the circle, that is the resolution of each bin is constant. The bins can be any iterator where
* weights are computed.
* 
* \param    beginBin            Start of the bins
* \param    endBin              End of the bins
*/
template <class BinIter>
double weighted_angle_sum(BinIter beginBin, BinIter endBin)
{
    if(beginBin >= endBin)
    {
        return 0.0;
    }
    
    double kBinResolution = 2.0 * M_PI / std::distance(beginBin, endBin);
    double sumCos = 0;
    double sumSin = 0;
    double sumWeight = 0.0;
    double binAngle = 0.0;

    for(auto binIt = beginBin; binIt != endBin; ++binIt, binAngle +=kBinResolution)
    {
        if(*binIt > 0)
        {
            sumCos += *binIt * std::cos(binAngle);
            sumSin += *binIt * std::sin(binAngle);
            sumWeight += *binIt;
        }
    }

    if(sumWeight > 0.0)
    {
        double avgNormal = std::atan2(sumSin / sumWeight, sumCos / sumWeight);
        return avgNormal;
    }
    else
    {
        return 0.0;
    }
}

}
}

#endif // HSSH_METRICAL_MAPPING_GLASS_MAP_UTILS_H
