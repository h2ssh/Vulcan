/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_builder.h
* \author   Collin Johnson
*
* Declaration of SkeletonBuilder base class and create_skeleton_builder factory.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_BUILDER_H

#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/voronoi/island_detector.h>
#include <hssh/local_topological/params.h>
#include <memory>

namespace vulcan
{
struct pose_t;

namespace hssh
{

class LocalPerceptualMap;
class SkeletonBuilder;

/**
* SkeletonBuilder is an interface for extracting the skeleton and labeled place grid for
* the current LPM. The SkeletonBuilder produces two outputs. One is a VoronoiSkeletonGrid with all
* cells labeled appropriately. The other is a list of potential anchor_point_t for gateways.
*
* Embedded in the VoronoiSkeletonGrid is a reduced extended Voronoi graph representation of the
* physical space as indicated by cells labeled PLACE_GRID_SKELETON
*/
class SkeletonBuilder
{
public:

    /**
    * Constructor for SkeletonBuilder.
    */
    SkeletonBuilder(const skeleton_builder_params_t& params);

    virtual ~SkeletonBuilder(void);

    /**
    * buildSkeleton creates a VoronoiSkeletonGrid and set of AnchorPoints from an LPM representation
    * of the world and a set of anchor_point_t that are potential locations for gateways.
    */
    void buildSkeleton(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid);

    /**
    * locateJunctionsAndDeadEnds finds the dead ends and junctions amongst the reduced skeleton cells.
    */
    void locateJunctionsAndDeadEnds(VoronoiSkeletonGrid& grid, uint8_t classification);

protected:

    /**
    * extractSkeleton is the method to be implemented by instantiations of the SkeletonBuilder class. This method
    * is called by buildSkeleton to handle the task of extracting the skeleton and anchor points from an occupancy
    * grid.
    */
    virtual void extractSkeleton(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid, IslandDetector& islands) = 0;

    std::vector<cell_t>* skeletonPoints;
    SkeletonToSources*   skeletonAnchors;
    std::vector<cell_t>* frontierPoints;

    VoronoiDist coastalDistanceInCells;

    skeleton_builder_params_t skeletonParams;

private:

    void initializeSkeletonExtraction(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid);

    IslandDetector islands;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_SKELETON_BUILDER_H
