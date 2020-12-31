/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     params.h
 * \author   Jong Jin Park and Collin Johnson
 *
 * Declaration of params structs for gridBuilders.
 */

#ifndef MPEPC_GRID_BUILDER_PARAMS_H
#define MPEPC_GRID_BUILDER_PARAMS_H

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace mpepc
{

struct obstacle_distance_grid_builder_params_t
{
    bool isUnobservedObstacle;
    float maxObstacleDistance;
    bool shouldGrowObstacle;
    float growObstacleRadius;

    obstacle_distance_grid_builder_params_t(void){};
    obstacle_distance_grid_builder_params_t(const utils::ConfigFile& config);
};

struct navigation_grid_builder_params_t
{
    bool shouldPropagateObstacleCost;
    float robotShortRadius;
    float robotLongRadius;
    float staticObstacleCostRadius;
    float staticObstacleCost;

    bool shouldAddQuasiStaticObjects;
    float quasiStaticTiming;
    float quasiStaticLength;
    float quasiStaticCostRadius;
    float quasiStaticCostPeak;
    float quasiStaticCostSlope;

    navigation_grid_builder_params_t(void){};
    navigation_grid_builder_params_t(const utils::ConfigFile& config);
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // METRIC_GRID_BUILDER_PARAMS_H
