/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_collision_model.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of RobotCollisionModel interface, ConvexPolygonCollisionModel, RectangleCollisionModel and
* CircleCollisionModel implementations, and create_robot_collision_model factory.
*/

#ifndef ROBOT_COLLISION_MODEL_H
#define ROBOT_COLLISION_MODEL_H

#include <memory>
#include <string>
#include "core/pose.h"
#include "core/line.h"
#include "math/geometry/rectangle.h"
#include "math/geometry/polygon.h"

namespace vulcan
{
namespace robot
{
    struct collision_model_params_t;
}

namespace mpepc
{

class RobotCollisionModel;
class ObstacleDistanceGrid;

const std::string kConvexPolygonCollisionModelType("polygon");
const std::string kRectangleCollisionModelType    ("rectangle");
const std::string kCircleCollisionModelType       ("circle");

/**
* create_robot_collision_model creates an instance of RobotCollisionModel.
*
* NOTE: If an invalid collision model is specified, the program will fail fast.
*
* \param    params      Parameters for the collision model
* \return   Instance of the RobotCollisionModel
*/
std::unique_ptr<RobotCollisionModel> create_robot_collision_model(const robot::collision_model_params_t& params);

/**
* RobotCollisionModel is an interface used for finding collisions between the robot and dynamic and static
* objects in the environment. Dynamic objects are assumed to be circular.
* 
*/
class RobotCollisionModel
{
public:

    virtual ~RobotCollisionModel(void) { }
    
    /**
    * setPose sets a robot pose to use.
    * 
    * \param   robotPose       Pose of the robot
    */
    virtual void setRobotPose(const pose_t& robotPose) = 0;

    /**
    * distanceToObject calculates the minimum distance between the robot and the object at the given pose.
    *
    * \param    objectPose      Pose of the dynamic object
    * \param    objectRadius    Radius of the circle approximating the object bound
    * \param    robotPose  Pose of the robot (optional)
    * \return   The closest distance between the robot and object. 0 distance means there is a collision.
    */
    virtual float distanceToObject(const pose_t& objectPose, float objectRadius) const = 0;

    /**
    * closestObstacle calculates the distance to the closest obstacle in the map from a robot.
    *
    * \param    obstacleDistanceGrid    Map of distance to the nearest staic obstacles
    * \param    robotPose          Pose of the robot (optional)
    * \return   The distance to the closest obstacle. 0 means there is a collision.
    */
    virtual float closestObstacle(const ObstacleDistanceGrid& obstacleDistanceGrid) const = 0;
    
    /**
    * collisionArea calculates overlapping area between the robot body and static obstacles. 0 area means no collision, with increasing value indicating more serious collision.
    *
    * \param    obstacleDistanceGrid    Map of distance to the nearest staic obstacles
    * \param    robotPose          Pose of the robot (optional)
    * \return   The distance to the closest obstacle. 0 means there is a collision.
    */
    virtual float collisionArea(const ObstacleDistanceGrid& obstacleDistanceGrid) const = 0;
    
};


/**
* ConvexPolygonCollisionModel is a collision model that represents the robot boundary as a vector of vertices. The shape is assumed to be symmetric and convex.
*/
class ConvexPolygonCollisionModel : public RobotCollisionModel
{
public:

    /**
    * Constructor for ConvexCollisionModel.
    *
    * \param    vertices            Vertex of the robot
    */
    ConvexPolygonCollisionModel(const robot::collision_model_params_t& params);
    
    // RobotCollisionModel interface
    virtual void  setRobotPose(const pose_t& robotPose) final; /* override */
    virtual float distanceToObject(const pose_t& objectPose, float objectRadius) const final; /* override */
    virtual float closestObstacle (const ObstacleDistanceGrid& obstacleDistanceGrid) const final; /* override */
    virtual float collisionArea   (const ObstacleDistanceGrid& obstacleDistanceGrid) const final; /* override */
    
private:
    
    bool createRobotBodyPointsFromVertices(void);
    
    float shortestRadius_;
    float longestRadius_;
    float bodyCellSize_;
    
    pose_t                   robotPose_;
    math::Polygon<float>            boundary_;
    math::Polygon<float>            transformed_;
    std::vector<Point<float>> robotBodyPoints_;
};


/**
* RectangleCollisionModel is a collision model that represents the robot as a rectangle. The closest distance is calcuated
* by finding the closest distance between the object pose and each of the boundary lines for the rectangle. If the object
* pose is inside the rectangle, that'll be a collision too (obviously!).
*/
class RectangleCollisionModel : public RobotCollisionModel
{
public:

    /**
    * Constructor for RectangleCollisionModel.
    *
    * \param    boundary            Boundary of the robot
    */
    RectangleCollisionModel(const math::Rectangle<float>& boundary);

    // RobotCollisionModel interface
    virtual void  setRobotPose(const pose_t& robotPose) final; /* override */
    virtual float distanceToObject(const pose_t& objectPose, float objectRadius) const final; /* override */
    virtual float closestObstacle (const ObstacleDistanceGrid& obstacleDistanceGrid) const final; /* override */
    virtual float collisionArea   (const ObstacleDistanceGrid& obstacleDistanceGrid) const final; /* override */

private:
    
    pose_t          robotPose_;
    math::Rectangle<float> boundary_;
    math::Rectangle<float> transformed_;
};

/**
* CircleCollisionModel is a simple collision model where the robot is treated as a circle with a fixed radius.
* The collision check is simply the distance between the poses minus their cumulative radii.
*/
class CircleCollisionModel : public RobotCollisionModel
{
public:

    /**
    * Constructor for CircleCollisionModel.
    *
    * \param    radius          Radius of the circle representing the robot
    */
    CircleCollisionModel(float radius);

    // RobotCollisionModel interface
    virtual void  setRobotPose(const pose_t& robotPose) final; /* override */
    virtual float distanceToObject(const pose_t& objectPose, float objectRadius) const final; /* override */
    virtual float closestObstacle (const ObstacleDistanceGrid& obstacleDistanceGrid) const final; /* override */
    virtual float collisionArea   (const ObstacleDistanceGrid& obstacleDistanceGrid) const final; /* override */

private:

    pose_t robotPose_;
    float radius_;
};

} // namespace mpepc
} // namespace vulcan

#endif // ROBOT_COLLISION_MODEL_H
