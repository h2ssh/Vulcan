/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_isovist_field.cpp
* \author   Collin Johnson
*
* Definition of VoronoiIsovistField.
*/

#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <math/statistics.h>
#include <cassert>
#include <unordered_map>
#include <functional>


namespace vulcan
{
namespace hssh
{

std::vector<cell_t> extract_isovist_positions(const VoronoiSkeletonGrid& grid, IsovistLocation location);


VoronoiIsovistField::VoronoiIsovistField(const VoronoiSkeletonGrid& grid, IsovistLocation location, utils::isovist_options_t options)
: VoronoiIsovistField(grid, extract_isovist_positions(grid, location), options)
{
}


VoronoiIsovistField::VoronoiIsovistField(const VoronoiSkeletonGrid& grid, const std::vector<cell_t>& positions, utils::isovist_options_t options)
: field_(positions, grid, VoronoiSkeletonTerminationFunc(), options)
{
    int nextIndex = 0;
    for(auto& cell : positions)
    {
        cellToIsovist_[cell] = nextIndex++;
    }
}


VoronoiIsovistField::~VoronoiIsovistField(void)
{
    // Nothing to do here
}


std::vector<cell_t> extract_isovist_positions(const VoronoiSkeletonGrid& grid, IsovistLocation location)
{
    switch(location)
    {
    case IsovistLocation::FREE_SPACE:
        return extract_cells_with_mask(grid, SKELETON_CELL_FREE, 2);  // only use every other cell when processing free space -- too slow otherwise!

    case IsovistLocation::SKELETON:
        return extract_cells_with_mask(grid, SKELETON_CELL_SKELETON, 1);

    case IsovistLocation::REDUCED_SKELETON:
        return extract_cells_with_mask(grid, SKELETON_CELL_REDUCED_SKELETON, 1);

    default:
        std::cerr<<"ERROR::VoronoiIsovistField: Unknown isovist location\n.";
    }

    return std::vector<cell_t>();
}

} // namespace hssh 
} // namespace vulcan
