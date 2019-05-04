/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     voronoi_skeleton_builder.h
* \author   Collin Johnson
*
* Declaration of VoronoiSkeletonBuilder.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_VORONOI_SKELETON_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_VORONOI_SKELETON_BUILDER_H

#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/voronoi/brushfire_skeleton_builder.h>
#include <hssh/local_topological/area_detection/voronoi/skeleton_pruner.h>

namespace vulcan
{
namespace hssh
{

class  SkeletonBuilder;
class  LocalPerceptualMap;

/**
* VoronoiSkeletonBuilder builds a Voronoi skeleton of the current LPM. The skeleton builder creates
* a reduced extended Voronoi graph. The VoronoiSkeletonGrid contains the following information about
* the
*/
class VoronoiSkeletonBuilder
{
public:

    /**
    * Constructor for VoronoiSkeletonBuilder.
    *
    * \param    params          Parameters for the graph builder
    */
    VoronoiSkeletonBuilder(const skeleton_builder_params_t& skeletonParams,
                           const skeleton_pruner_params_t&  prunerParams);

    /**
    * Destructor for VoronoiSkeletonBuilder.
    */
    ~VoronoiSkeletonBuilder(void);

    /**
    * buildVoronoiSkeleton builds the Voronoi skeleton of the current LPM.
    *
    * \param    lpm                 LPM of the environment
    * \param    pose                Current robot pose in the environment
    * \param    pointsOfInterest    Points that should be part of the reduced skeleton (global coords)
    */
    void buildVoronoiSkeleton(const LocalPerceptualMap& lpm,
                              const pose_t& pose,
                              const std::vector<Point<float>>& pointsOfInterest);

    /**
    * getVoronoiSkeleton retrieves the VoronoiSkeletonGrid constructed from the latest LPM.
    *
    * TODO: Update other interfaces such that I can make the returned grid a const.
    */
    VoronoiSkeletonGrid& getVoronoiSkeleton(void) { return skeleton_; }

private:

    BrushfireSkeletonBuilder skeletonBuilder_;
    SkeletonPruner           skeletonPruner_;

    VoronoiSkeletonGrid skeleton_;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_VORONOI_SKELETON_BUILDER_H
