/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_builder.cpp
* \author   Collin Johnson
*
* Definition of SkeletonBuilder and create_skeleton_builder factory.
*/

#include "hssh/local_topological/area_detection/voronoi/skeleton_builder.h"
#include "hssh/local_topological/area_detection/voronoi/brushfire_skeleton_builder.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_metric/lpm.h"
#include "math/angle_range.h"
#include "utils/algorithm_ext.h"
#include <cassert>
#include <iostream>

// #define DEBUG_DEAD_ENDS

namespace vulcan
{
namespace hssh
{

bool is_junction(cell_t skeleton, const VoronoiSkeletonGrid& grid, uint8_t classification);
bool is_dead_end(cell_t skeleton,
                 const SourceCells& anchorCells,
                 uint8_t classification,
                 const VoronoiSkeletonGrid& grid,
                 const CellVector& deadEndsSoFar,        // dead ends found so far during extraction
                 double minSeparationAngle);


SkeletonBuilder::SkeletonBuilder(const skeleton_builder_params_t& params)
: skeletonParams(params)
, islands(params.maxIslandAreaInCells)
{
}


SkeletonBuilder::~SkeletonBuilder(void)
{
}


void SkeletonBuilder::buildSkeleton(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid)
{
    grid.setTimestamp(map.getTimestamp());
    initializeSkeletonExtraction(map, grid);
    extractSkeleton(map, grid, islands);

    grid.setReferenceIndex(map.getReferenceFrameIndex());
    grid.setFrameTransform(map.getTransformFromPreviousFrame());

    // Sort skeleton cells in order of decreasing distance from walls to simplify other processing
    std::sort(grid.junctions.begin(), grid.junctions.end(), [&grid](cell_t lhs, cell_t rhs) {
        return grid.getMetricDistance(lhs.x, lhs.y) > grid.getMetricDistance(rhs.x, rhs.y);
    });
}


void SkeletonBuilder::locateJunctionsAndDeadEnds(VoronoiSkeletonGrid& grid, uint8_t classification)
{
    std::vector<cell_t>& junctions = grid.junctions;
    std::vector<cell_t>& deadEnds  = grid.deadEnds;

    junctions.clear();
    deadEnds.clear();

    NeighborArray neighbors;

    for(auto skeleton : *skeletonPoints)
    {
        if((grid.getClassification(skeleton.x, skeleton.y) & classification))
        {
            if(is_junction(skeleton, grid, classification))
            {
                junctions.push_back(skeleton);
            }

            if((classification == SKELETON_CELL_SKELETON)
                && is_dead_end(skeleton,
                               (*skeletonAnchors)[skeleton],
                               classification,
                               grid,
                               deadEnds,
                               skeletonParams.minDeadEndAngleSeparation))
            {
                // Final sanity check to avoid really useless dead ends
                bool isWideEnough = grid.getMetricDistance(skeleton.x, skeleton.y) > 0.3;
                int frontierCount = 0;
                for(auto& cell : (*skeletonAnchors)[skeleton])
                {
                    if(grid.getClassification(cell) & SKELETON_CELL_FRONTIER)
                    {
                        ++frontierCount;
                    }
                }
                bool isFrontier = frontierCount > 1;

                if(isWideEnough || isFrontier)
                {
                    deadEnds.push_back(skeleton);
                }
            }
            else if(classification == SKELETON_CELL_REDUCED_SKELETON
                && neighbor_cells_with_classification(skeleton,
                                                      SKELETON_CELL_REDUCED_SKELETON,
                                                      grid,
                                                      FOUR_THEN_EIGHT_WAY,
                                                      neighbors) == 1)
            {
                deadEnds.push_back(skeleton);
            }

            if(skeleton == cell_t(681, 1340) && !deadEnds.empty())
            {
                std::cout << skeleton << " de:" << deadEnds.back() << '\n';
            }
        }
    }

    std::cout<<"INFO:SkeletonBuilder:Junctions:"<<junctions.size()<<" Dead ends:"<<deadEnds.size()<<'\n';
}


void SkeletonBuilder::initializeSkeletonExtraction(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid)
{
    skeletonPoints  = &grid.skeletonCells;
    skeletonAnchors = &grid.skeletonSources;
    frontierPoints  = &grid.frontierCells;

    skeletonPoints->clear();
    skeletonAnchors->clear();
    frontierPoints->clear();

    islands.reset();

    if((grid.getWidthInCells() != map.getWidthInCells()) || (grid.getHeightInCells() != map.getHeightInCells()))
    {
        grid.setGridSize(map.getWidthInCells(), map.getHeightInCells());
    }

    assert((grid.getWidthInCells() >= map.getWidthInCells()) && (grid.getHeightInCells() >= map.getHeightInCells()));

    grid.setScale(map.metersPerCell());
    grid.setBottomLeft(map.getBottomLeft());
}


bool is_junction(cell_t skeleton, const VoronoiSkeletonGrid& grid, uint8_t classification)
{
    return num_neighbor_cells_with_classification(skeleton, classification, grid, FOUR_THEN_EIGHT_WAY) > 2; // need three neighbors here
}


bool is_dead_end(cell_t skeleton,
                 const SourceCells& anchorCells,
                 uint8_t classification,
                 const VoronoiSkeletonGrid& grid,
                 const CellVector& deadEndsSoFar,
                 double minSeparationAngle)
{
    // At least three well-separated anchor cells are needed. If there are more than three anchor cells (happens
    // with nearby cells hitting the same skeleton point), then the number of expected good angles changes. The
    // number of poorly separated should only be between two anchor cells for four total, and two sets of two for five total.
    // Therefore, if #cells - #poorlySeparated >= 3, a dead end still exists

    std::set<cell_t> anchorsToConsider(anchorCells.begin(), anchorCells.end());
    NeighborArray    neighbors;
    std::size_t      numNeighbors = neighbor_cells_with_classification(skeleton,
                                                                       classification,
                                                                       grid,
                                                                       FOUR_THEN_EIGHT_WAY,
                                                                       neighbors);

    // For the dead end detection, it must be a junction or an actual dead end because there will be bits of
    // branches coming from the corners where the walls that form the dead end or constriction meet.
    if((numNeighbors == 2) || utils::contains_any(deadEndsSoFar, neighbors))
    {
        return false;
    }
    // If it is a junction, consider the neighbors anchors as well because strange effects can occur right at
    // junctions that cause the anchors to end up on adjacent cells
    else if(numNeighbors > 2)
    {
        for(std::size_t n = 0; n < numNeighbors; ++n)
        {
            anchorsToConsider.insert(grid.beginSourceCells(neighbors[n]), grid.endSourceCells(neighbors[n]));
        }
    }

    assert(!anchorsToConsider.empty());   // if there aren't anchors, then something is drastically wrong with this cell and it is a program error

    std::vector<cell_t> wellSeparatedAnchors;
    wellSeparatedAnchors.push_back(*anchorsToConsider.begin());

    for(auto anchorIt = anchorsToConsider.begin(); anchorIt != anchorsToConsider.end(); ++anchorIt)
    {
        bool isWellSeparated = true;

        for(auto separated : wellSeparatedAnchors)
        {
            double separationAngle = std::abs(angle_between_points(*anchorIt, separated, skeleton));
            if(separationAngle < minSeparationAngle)
            {
                isWellSeparated = false;
                break;
            }
        }

        if(isWellSeparated)
        {
            wellSeparatedAnchors.push_back(*anchorIt);
        }
    }

    if(wellSeparatedAnchors.size() > 2)
    {
        math::angle_range_t range(angle_to_point(skeleton, wellSeparatedAnchors.front()));

        for(auto& anchor : wellSeparatedAnchors)
        {
            range.expand(angle_to_point(skeleton, anchor));
        }

        return range.extent > (M_PI * 125.0 / 180.0);
    }

    return false;
}

} // namespace hssh
} // namespace vulcan
