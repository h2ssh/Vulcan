/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     types.h
* \author   Collin Johnson
*
* Definition of common types used throughout the MPEPC implementation:
*
*   - planning_environment_t : structure holding references to current information about the robot environment
*/

#ifndef MPEPC_TYPES_H
#define MPEPC_TYPES_H

#include <core/line.h>
#include <core/motion_state.h>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }
namespace hssh { class LocalTopoMap; }
namespace mpepc
{

class ObstacleDistanceGrid;
class VisibilityAnalysis;

const int32_t kStraightCellDist = 100;
const int32_t kDiagCellDist = 141;

/**
* planning_environment_t contains the fundamental representations about the environment in which the robot is
* navigating.
*/
struct planning_environment_t
{
    motion_state_t robotState;           // current pose of the robot
    float robotRadius;                          // radius of current robot model
    const hssh::LocalPerceptualMap* lpm;        // never null
    const hssh::LocalTopoMap* ltm;              // can be null
    const ObstacleDistanceGrid* dists;          // never null
    const VisibilityAnalysis* visibility;       // never null
};

/**
* RotationMode defines the different types of rotation that can be performed. If the desire is for the robot to go to
* a specific orientation, then the fixed_orientation mode is used. Otherwise, turn_left or turn_right will have the
* robot continuously turn in one direction or the other without ever actually reaching the goal. They will be tasks
* that the robot performs forever until stopped.
*/
enum class RotationMode
{
    fixed_orientation,
    turn_left,
    turn_right,
};

/**
* NavWaypoint defines an intermediate waypoint along a high-level path. The waypoints are line segments that the robot
* must cross. These waypoints are gateways separating two areas in the map.
*/
using NavWaypoint = Line<double>;

} // namespace vulcan
} // namespace mpepc

#endif // MPEPC_TYPES_H
