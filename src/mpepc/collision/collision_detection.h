/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     collision_detection.h
* \author   Collin Johnson
*
* Declaration of detect_collisions function and robot_collision_model_t for
* representing the robot in collision detection.
*/

#ifndef ROBOT_COLLISION_DETECTION_H
#define ROBOT_COLLISION_DETECTION_H

#include <vector>
#include <core/point.h>

namespace vulcan
{
namespace math  { template <typename T> class Rectangle; }
struct pose_t;
namespace hssh  { class LocalPerceptualMap; }

namespace mpepc
{

class ObstacleDistanceGrid;

/**
* detect_collisions checks for collisions between the robot and the environment at
* a specified pose. The robot is represented as a rectangle. The algorithm checks
* transforms the robot model to the pose, then looks at every cell inside the rectangle
* to see if it is occupied. If so, then a collision is detected.
*
* \param    model           Model of the robot
* \param    poseToCheck     Pose to be checked for collisions
* \param    grid            hssh::LocalPerceptualMap in which to search for collisions
* \return   Number of collisions between the robot and the grid at the specified pose.
*/
unsigned int detect_collisions(const math::Rectangle<float>&   model,
                               const pose_t&            poseToCheck,
                               const hssh::LocalPerceptualMap& lpm);

/**
* closest_obstacle finds the closest obstacle to the robot specified by the model at the given pose.
* The obstacle distance map is assumed to encode the distance to the nearest obstacle at each point.
*
* \param    model           Model of the robot
* \param    poseToCheck     Pose of the robot for which to find the closest obstacle
* \param    distGrid        Grid with the distance to the closest static obstacle marked for each cell
* \return   The closest obstacle to the robot at the given pose.
*/
float closest_obstacle(const math::Rectangle<float>& model,
                       const pose_t&          poseToCheck,
                       const ObstacleDistanceGrid&   distGrip);
}
}

#endif // ROBOT_COLLISION_DETECTION_H
