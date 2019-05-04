/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     search.cpp
* \author   Collin Johnson
*
* Definition of find_path_along_skeleton function.
*/

#include <hssh/local_topological/area_detection/voronoi/search.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <utils/algorithm_ext.h>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>

namespace vulcan
{
namespace hssh
{

using PredMap = CellToTypeMap<cell_t>;     // maintain predecessors for extracting the path

std::ostream& operator<<(std::ostream& out, const voronoi_path_t& path)
{
    std::copy(path.cells.begin(), path.cells.end(), std::ostream_iterator<cell_t>(out, " "));
    return out;
}

/*
* astar_in_skeleton_grid finds a path from start until the goal condition is satisfied. The search will propagate to all
* neighbors satisfying the mask. The GoalCondition is a functor with the signature:
*
*   cond(cell, skeleton)
*
* and should return true or false depending on if the condition has been satisfied.
*/
template <class GoalCondition>
voronoi_path_t astar_in_skeleton_grid(cell_t start,
                                      uint8_t mask,
                                      const VoronoiSkeletonGrid& skeleton,
                                      GoalCondition goal);
voronoi_path_t extract_path_from_predecessors(cell_t goal,
                                              PredMap& predecessors,
                                              const VoronoiSkeletonGrid& skeleton);


voronoi_path_t find_path_along_skeleton(cell_t start, cell_t goal, uint8_t mask, const VoronoiSkeletonGrid& skeleton)
{
    assert(mask & (SKELETON_CELL_SKELETON | SKELETON_CELL_REDUCED_SKELETON));

    voronoi_path_t startToSkeleton = path_to_skeleton(start, mask, skeleton);
    voronoi_path_t goalToSkeleton = path_to_skeleton(goal, mask, skeleton);
    std::reverse(goalToSkeleton.cells.begin(), goalToSkeleton.cells.end());   // reverse path to get skeleton to goal

    // If no path to the skeleton is found, then exit
    if(startToSkeleton.cells.empty())
    {
//         std::cout << "ERROR: find_path_along_skeleton: Failed to find path from start cell to skeleton. Start:"
//             << start << '\n';
        voronoi_path_t failed;
        failed.result = VoronoiPathResult::invalid_start;
        return failed;
    }

    if(goalToSkeleton.cells.empty())
    {
//         std::cout << "ERROR: find_path_along_skeleton: Failed to find path from goal cell to skeleton. Goal:"
//             << goal << '\n';
        voronoi_path_t failed;
        failed.result = VoronoiPathResult::invalid_goal;
        return failed;
    }

    // Run an A* search
    voronoi_path_t pathAlongSkeleton = astar_in_skeleton_grid(
        startToSkeleton.cells.back(),
        mask,
        skeleton,
        [&goalToSkeleton](cell_t cell, const VoronoiSkeletonGrid& skeleton) {
            return cell == goalToSkeleton.cells.front();    // find the cell along the skeleton leading to the goal
    });

    if(pathAlongSkeleton.cells.empty())
    {
//         std::cout << "ERROR: find_path_along_skeleton: Failed to find path along skeleton from "
//             << startToSkeleton.cells.back() << " to " << goalToSkeleton.cells.front() << '\n';
//
//         std::cout << "Start to skeleton:" << startToSkeleton << "\nSkeleton to goal:" << goalToSkeleton << '\n';
        voronoi_path_t failed;
        failed.result = VoronoiPathResult::no_path_found;
        return failed;
    }

    // Concatenate the paths into one
    voronoi_path_t finalPath = startToSkeleton;
    boost::push_back(finalPath.cells, boost::as_array(pathAlongSkeleton.cells));
    boost::push_back(finalPath.cells, boost::as_array(goalToSkeleton.cells));
    utils::erase_unique(finalPath.cells);
    finalPath.length += pathAlongSkeleton.length + goalToSkeleton.length;
    finalPath.result = VoronoiPathResult::success;

    return finalPath;
}


voronoi_path_t path_to_skeleton(cell_t freeCell, uint8_t mask, const VoronoiSkeletonGrid& skeleton)
{
    return astar_in_skeleton_grid(freeCell,
                                  SKELETON_CELL_FREE,       // search through all free space for the path
                                  skeleton,
                                  [mask](cell_t cell, const VoronoiSkeletonGrid& skeleton) {
        return skeleton.getClassification(cell.x, cell.y) & mask;
    });
}


template <class GoalCondition>
voronoi_path_t astar_in_skeleton_grid(cell_t start,
                                      uint8_t mask,
                                      const VoronoiSkeletonGrid& skeleton,
                                      GoalCondition goalCond)
{
    PredMap predecessors;
    predecessors[start] = start;        // the start points to itself

    // If the start satisfies the goal, then we don't need to perform the full search
    if(goalCond(start, skeleton))
    {
        return extract_path_from_predecessors(start, predecessors, skeleton);
    }

    NeighborArray neighbors;
    std::deque<cell_t> queue;
    queue.push_back(start);

    while(!queue.empty())
    {
        cell_t active = queue.front();
        queue.pop_front();

        int numNeighbors = neighbor_cells_with_classification(active, mask, skeleton, FOUR_THEN_EIGHT_WAY, neighbors);
        for(int n = 0; n < numNeighbors; ++n)
        {
            cell_t next = neighbors[n];

            // Ignore neighbor that already have predecessors because they have been reached already
            if(predecessors.find(next) != predecessors.end())
            {
                continue;
            }

            predecessors[next] = active;
            queue.push_back(next);

            // As soon as the goal is found, then we're done
            if(goalCond(next, skeleton))
            {
                return extract_path_from_predecessors(next, predecessors, skeleton);
            }
        }
    }

    // If we get to here, then the goal condition was never satisfied, so no path exists.
    voronoi_path_t failed;
    failed.result = VoronoiPathResult::no_path_found;
    return failed;
}


voronoi_path_t extract_path_from_predecessors(cell_t goal,
                                              PredMap& predecessors,
                                              const VoronoiSkeletonGrid& skeleton)
{
    voronoi_path_t path;
    path.cells.push_back(goal);
    path.length = 0.0;
    path.result = VoronoiPathResult::success;

    while(predecessors[goal] != goal)
    {
        cell_t parent = predecessors[goal];
        path.cells.push_back(parent);
        path.length += distance_between_points(utils::grid_point_to_global_point(goal, skeleton),
                                                     utils::grid_point_to_global_point(parent, skeleton));
        goal = parent;
    }

    std::reverse(path.cells.begin(), path.cells.end());
    return path;
}

} // namespace hssh
} // namespace vulcan
