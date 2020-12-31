/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_isovist_field.h
* \author   Collin Johnson
*
* Declaration of VoronoiIsovistField.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_ISOVIST_FIELD_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_ISOVIST_FIELD_H

#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/isovist.h"
#include <memory>

namespace vulcan
{
namespace hssh
{

/**
* IsovistLocation defines the possible locations for the isovists to be calculated when not using the
* constructor that accepts a set of points.
*/
enum class IsovistLocation
{
    FREE_SPACE,
    SKELETON,
    REDUCED_SKELETON
};

/**
* VoronoiSkeletonTerminationFunc is a termination function for the ray tracing that is performed when the IsovistField
* is constructed.
*/
struct VoronoiSkeletonTerminationFunc
{
public:

    VoronoiSkeletonTerminationFunc(uint8_t mask = SKELETON_CELL_OCCUPIED | SKELETON_CELL_UNKNOWN)
    : mask_(mask)
    {
    }

    bool operator()(const VoronoiSkeletonGrid& grid, Point<int> cell) const
    {
        return grid.getClassificationNoCheck(cell.x, cell.y) & mask_;
    }

private:

    uint8_t mask_;
};

/**
* VoronoiIsovistField is a wrapper around the generic IsovistField that crafts isovists using the representation of space
* in the VoronoiSkeletonGrid. The isovist field can be generated using known points in the grid -- free space or junctions --
* or some otherwise defined positions, perhaps locations within a LocalArea, like the center or frontiers.
*/
class VoronoiIsovistField
{
public:

    using Iter = utils::IsovistField::Iter;

    /**
     * Constructor for VoronoiIsovistField.
    *
    * \param    grid        Grid to use for constructing the field
    * \param    location    Locations where the isovists should be generated
    */
    VoronoiIsovistField(const VoronoiSkeletonGrid& grid, IsovistLocation location, utils::isovist_options_t options = utils::isovist_options_t());

    /**
    * Constructor for VoronoiIsovistField.
    *
    * \param    grid        Grid to use for constructing the field
    * \param    positions   Positions at which to generate the isovists
    */
    VoronoiIsovistField(const VoronoiSkeletonGrid& grid, const std::vector<cell_t>& positions, utils::isovist_options_t options = utils::isovist_options_t());

    /**
    * Destructor for VoronoiIsovistField.
    */
    ~VoronoiIsovistField(void);

    // Iterators
    Iter        begin(void) const { return field_.begin(); }
    Iter        end(void)   const { return field_.end(); }
    std::size_t size(void)  const { return field_.size(); }
    const utils::Isovist& at(std::size_t n) const { return operator[](n); }
    const utils::Isovist& at(cell_t cell) const { return operator[](cell); }
    
    const utils::Isovist& operator[](std::size_t n) const { return *(field_.begin() + n); }
    const utils::Isovist& operator[](cell_t cell) const { return field_[cellToIsovist_.at(cell)]; }
    
    bool contains(cell_t cell) const { return cellToIsovist_.find(cell) != cellToIsovist_.end(); }

private:

    utils::IsovistField field_;
    CellToIntMap        cellToIsovist_;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_ISOVIST_FIELD_H
