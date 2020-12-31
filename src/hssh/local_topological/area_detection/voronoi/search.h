/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     search.h
 * \author   Collin Johnson
 *
 * Declaration of VoronoiPath and find_path_along_skeleton function. The path finding located the closest skeleton cell
 * and then uses an A* search along the skeleton to find the path.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_VORONOI_SEARCH_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_VORONOI_SEARCH_H

#include "hssh/types.h"

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;

enum class VoronoiPathResult
{
    success,
    invalid_start,
    invalid_goal,
    no_path_found,
};

struct voronoi_path_t
{
    CellVector cells;
    double length = 0.0;
    VoronoiPathResult result;
};


/**
 * path_to_skeleton finds the path leading from a cell in free space to a skeleton cell defined by the provided mask.
 *
 * This function can be used to check that a valid path through the skeleton can be found for a given path.
 *
 * \param    freeCell        Starting cell
 * \param    mask            Mask defining which skeleton cells are valid
 * \param    skeleton        Skeleton grid in which to search
 * \return   Path from the free cell to a skeleton cell with the mask -- if one exists. Otherwise, the path result will
 *   indicate what error occurred.
 */
voronoi_path_t path_to_skeleton(cell_t freeCell, uint8_t mask, const VoronoiSkeletonGrid& skeleton);


/**
 * find_path_along_skeleton finds the path along the skeleton from a starting point to an end point. The type of
 * skeleton to follow is determined by the provided mask.
 *
 * The search first finds a path from the start to the skeleton and skeleton to goal. It then searches for a path
 * between these two points to create the full path.
 *
 * \param    start       Start cell for the search
 * \param    goal        Goal cell for the search
 * \param    mask        Type of skeleton cells along which to search
 * \param    skeleton    Skeleton grid of the environment
 * \return   A voronoi_path_t containing the cells from the start to the goal. If no path is found, there will be no
 * cells.
 */
voronoi_path_t find_path_along_skeleton(cell_t start, cell_t goal, uint8_t mask, const VoronoiSkeletonGrid& skeleton);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_VORONOI_SEARCH_H
