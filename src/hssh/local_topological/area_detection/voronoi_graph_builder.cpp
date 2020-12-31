/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     voronoi_graph_builder.cpp
* \author   Collin Johnson
*
* Definition of VoronoiGraphBuilder.
*/

#include "hssh/local_topological/area_detection/voronoi_graph_builder.h"
#include "hssh/local_topological/area_detection/voronoi/brushfire_skeleton_builder.h"
#include "hssh/local_topological/area_detection/voronoi/frontiers_set.h"

namespace vulcan
{
namespace hssh
{

VoronoiSkeletonBuilder::VoronoiSkeletonBuilder(const skeleton_builder_params_t& skeletonParams,
                                               const skeleton_pruner_params_t&  prunerParams)
: skeletonBuilder_(skeletonParams)
, skeletonPruner_(prunerParams)
{
}


VoronoiSkeletonBuilder::~VoronoiSkeletonBuilder(void)
{
}

void VoronoiSkeletonBuilder::buildVoronoiSkeleton(const LocalPerceptualMap& lpm,
                                                  const pose_t& pose,
                                                  const std::vector<Point<float>>& pointsOfInterest)
{
    skeletonBuilder_.buildSkeleton(lpm, skeleton_);
    skeletonBuilder_.locateJunctionsAndDeadEnds(skeleton_, SKELETON_CELL_SKELETON);

    skeletonPruner_.pruneSkeleton(skeleton_, pose, pointsOfInterest);
    skeletonBuilder_.locateJunctionsAndDeadEnds(skeleton_, SKELETON_CELL_REDUCED_SKELETON);

    skeleton_.exitPoints = skeletonPruner_.getExitPoints();
}

}
}
