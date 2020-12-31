/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_collision_model.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of create_robot_collision_model factory, RectangleCollisionModel, and CircleCollisionModel.
*/

#include "mpepc/collision/robot_collision_model.h"
#include "mpepc/collision/collision_detection.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "robot/model/params.h"
#include "core/pose.h"
#include <cassert>

// #define DEBUG_POLYGON
// #define DEBUG_RECTANGLE

namespace vulcan
{
    
namespace mpepc
{

std::unique_ptr<RobotCollisionModel> create_robot_collision_model(const robot::collision_model_params_t& params)
{
    if(params.type == kConvexPolygonCollisionModelType)
    {
        return std::unique_ptr<RobotCollisionModel>(new ConvexPolygonCollisionModel(params));
    }
    if(params.type == kRectangleCollisionModelType)
    {
        return std::unique_ptr<RobotCollisionModel>(new RectangleCollisionModel(params.rectangleModel));
    }
    else if(params.type == kCircleCollisionModelType)
    {
        return std::unique_ptr<RobotCollisionModel>(new CircleCollisionModel(params.circleModelRadius));
    }
    else
    {
        std::cerr<<"ERROR:create_robot_collision_model:Unknown collision model: "<<params.type<<std::endl;
        assert(false);
    }

    return std::unique_ptr<RobotCollisionModel>();
}


///////////////////////////  ConvexPolygonCollisionModel //////////////////////////////////////
ConvexPolygonCollisionModel::ConvexPolygonCollisionModel(const robot::collision_model_params_t& params)
: bodyCellSize_(params.bodyCellSize)
, boundary_(params.convexPolygonModel)
{
    // find the logest and the shortest radius from the center. this method doesn't work if the shape is not convex
    longestRadius_  = 0.0f;
    shortestRadius_ = 1000.0f;
    for(auto vertexIt = boundary_.begin(); vertexIt != boundary_.end()-1; vertexIt++)
//         for (auto vertex : boundary_)
    {
        float distanceToVertex = distance_between_points(*vertexIt, Point<float>(0,0));
        if(distanceToVertex > longestRadius_)
        {
            longestRadius_ = distanceToVertex;
        }
        
        Line<float> side(*vertexIt, *(vertexIt+1));
        float distanceToSide = distance_to_line_segment(Point<float>(0,0), side);
        if(distanceToSide < shortestRadius_)
        {
            shortestRadius_ = distanceToSide;
        }
        
#ifdef DEBUG_POLYGON
        std::cout<<"DEBUG: ConvexPolygonCollisionModel: Vertex: ("<<vertexIt->x<<", "<<vertexIt->y<<")\n";
        std::cout<<"DEBUG: ConvexPolygonCollisionModel: Distance to vertex: "<<distanceToVertex<<"\n";
        std::cout<<"DEBUG: ConvexPolygonCollisionModel: Distance to side  : "<<distanceToSide<<"\n";
#endif
    }
    
#ifdef DEBUG_POLYGON
    std::cout<<"DEBUG: ConvexPolygonCollisionModel: Longest radius : "<<longestRadius_ <<"\n";
    std::cout<<"DEBUG: ConvexPolygonCollisionModel: Shortest radius: "<<shortestRadius_<<"\n";
#endif
        
    assert(longestRadius_  >= shortestRadius_);
    assert(shortestRadius_ >  bodyCellSize_);
    
    createRobotBodyPointsFromVertices();
}


void ConvexPolygonCollisionModel::setRobotPose(const pose_t& robotPose)
{
    robotPose_   = robotPose;
    transformed_ = boundary_;
    transformed_.rotate(robotPose.theta);
    transformed_.translate(robotPose.x, robotPose.y);
}


float ConvexPolygonCollisionModel::distanceToObject(const pose_t& objectPose, float objectRadius) const
{
    float minDistance = transformed_.distanceToPoint(objectPose.toPoint());
    
    return (minDistance - objectRadius < 0.0f) ? 0.0f : minDistance - objectRadius; // object is assumed to be circular
}


float ConvexPolygonCollisionModel::closestObstacle(const ObstacleDistanceGrid& obstacleDistanceGrid) const
{
    // robot pose and initial distance to obstacles
    Point<float> robotPosition(robotPose_.toPoint());
    float distanceToObstacle = obstacleDistanceGrid.getMaxObstacleDistance();
    float distanceToObstacleFromRobotCenter = obstacleDistanceGrid.getObstacleDistance(robotPosition);
    
    // check distance from the center first
    if(distanceToObstacleFromRobotCenter <= shortestRadius_)
    {
        return 0.0f; // if the distance to obstacles from the robot center is less than the shortest radius of the robot, then it is in colliision.
    }
    
//     // check body cells for collision
//     if(obstacleDistanceGrid.getObstacleDistance(robotPosition) <= longestRadius_) // no need to do this if obstacles are far away from the robot center
//     {
//         for(auto bodyPointIt = robotBodyPoints_.begin(); bodyPointIt != robotBodyPoints_.end(); bodyPointIt++)
//         {
//             Point<int> bodyPointCell = obstacleDistanceGrid.positionToCell(*bodyPointIt);
//             if(obstacleDistanceGrid.getObstacleDistance(bodyPointCell) < 0.001f) // querying cell location is fast since it does not interpolate.
//             {
//                 return 0.0f; // if any of the cell containing body points is an obstacle cell, then the robot is in collision.
//             }
//         }
//     }
    
    // walk along boundary and find distance to obstacles
    // determine step size for efficient computation
    float stepSize = obstacleDistanceGrid.metersPerCell(); // a width of a grid cell
    if(distanceToObstacleFromRobotCenter <= longestRadius_ + 2.0*obstacleDistanceGrid.metersPerCell())
    {
        stepSize *= 0.5; // half a grid cell for objects that are near, for more accurate computation.
    }
    else
    {
        stepSize *= 2.0; // twice a grid cell for objects that are not, for efficiency.
    }
    
    // walk along the transformed boundary
    for(auto vertexIt = transformed_.begin(); vertexIt != transformed_.end()-1; vertexIt++)
    {
        Point<float> stepDirection = *(vertexIt+1) - *vertexIt; // direction to the next vertex
        float              lengthToWalk  = distance_between_points(*vertexIt, *(vertexIt+1));
        Point<float> currentPoint = *vertexIt;
        
        while(lengthToWalk > stepSize) // move onto next vertex if the distance left to vertex is less than step size.
        {
            float closestObstacleFromCurrentPoint = obstacleDistanceGrid.getObstacleDistance(currentPoint);
            
            if(closestObstacleFromCurrentPoint < distanceToObstacle)
            {
                distanceToObstacle = closestObstacleFromCurrentPoint;
            }
            
            if(distanceToObstacle < 0.001f)
            {
                return 0.0f; // if any point on boundary is in collision then the robot is in collision so we can break early.
            }
            
            // step to next point and compute length left to walk to the next vertex
            Point<float> step(stepDirection.x*stepSize, stepDirection.y*stepSize);
            currentPoint += step;
            
            lengthToWalk = distance_between_points(currentPoint, *(vertexIt+1));
        }        
    }
    
    return distanceToObstacle;
}


float ConvexPolygonCollisionModel::collisionArea(const ObstacleDistanceGrid& obstacleDistanceGrid) const
{
    float area = 0.0f;
    
    // NOTE: this is a quick and brute force solution that returns an approximate result. check how this works first and elaborate later.
    for(std::size_t n = 0; n < robotBodyPoints_.size(); ++n)
    {
        Point<float> robotBodyPoint = rotate(robotBodyPoints_[n], robotPose_.theta) + robotPose_.toPoint();
        Point<int>   robotBodyCell  = obstacleDistanceGrid.positionToCell(robotBodyPoint);
        
        if(obstacleDistanceGrid.getObstacleDistance(robotBodyCell) < obstacleDistanceGrid.metersPerCell())
        {
            area += bodyCellSize_*bodyCellSize_; // for each point that falls within an occupied cell add its area to collision area.
        }
    }
    
    // TODO: If this turns out to be slow then create robotBodyCore, which is a smaller boundary within a robot, and measure distance from there.
    
    return area;
}


bool ConvexPolygonCollisionModel::createRobotBodyPointsFromVertices(void)
{
    robotBodyPoints_.clear();
    
    std::size_t nSteps = static_cast<int>(longestRadius_ / bodyCellSize_);

    for(std::size_t i = -nSteps; i <= nSteps; ++i)
    {
        for(std::size_t j = -nSteps; j <= nSteps; ++j)
        {
            Point<float> testPoint(i*bodyCellSize_, j*bodyCellSize_);
            if(boundary_.contains(testPoint))
            {
                robotBodyPoints_.push_back(testPoint);
            }
        }
    }

    return robotBodyPoints_.size() > 1;
}


///////////////////////  RectangleCollisionModel //////////////////////////////////////
RectangleCollisionModel::RectangleCollisionModel(const math::Rectangle<float>& boundary)
: boundary_(boundary)
{
}


void RectangleCollisionModel::setRobotPose(const pose_t& robotPose)
{
    robotPose_   = robotPose;
    
    Point<float> robotPosition = robotPose.toPoint();
    transformed_.bottomLeft  = rotate(boundary_.bottomLeft,  robotPose.theta) + robotPosition;
    transformed_.bottomRight = rotate(boundary_.bottomRight, robotPose.theta) + robotPosition;
    transformed_.topLeft     = rotate(boundary_.topLeft,     robotPose.theta) + robotPosition;
    transformed_.topRight    = rotate(boundary_.topRight,    robotPose.theta) + robotPosition;
}


float RectangleCollisionModel::distanceToObject(const pose_t& objectPose, float objectRadius) const
{
    float minDistance = transformed_.distanceToPoint(objectPose.toPoint());

#ifdef DEBUG_RECTANGLE
    if(minDistance - objectRadius < 0.0f)
    {
        std::cout<<"DEBUG:RectCollision:Min:"<<minDistance<<" Boundary:"<<transformed<<" Obj:"<<object<<" ObjRad:"<<objectRadius<<'\n';
    }
#endif

    return (minDistance - objectRadius < 0.0f) ? 0.0f : minDistance - objectRadius;
}


float RectangleCollisionModel::closestObstacle(const ObstacleDistanceGrid& obstacleDistanceGrid) const
{
    return closest_obstacle(boundary_, robotPose_, obstacleDistanceGrid); // TODO: This should also be changed to use transformed points...
}


float RectangleCollisionModel::collisionArea(const ObstacleDistanceGrid& obstacleDistanceGrid) const
{
    return 0.0f; // TODO: Implement this after testing convex polygon collision model.
}


///////////////////// CircleCollisionModel //////////////////////////////////////
CircleCollisionModel::CircleCollisionModel(float radius)
: radius_(radius)
{
}


void CircleCollisionModel::setRobotPose(const pose_t& robotPose)
{
    robotPose_ = robotPose;
}


float CircleCollisionModel::distanceToObject(const pose_t& objectPose, float objectRadius) const
{
    float distanceBetweenCenters  = distance_between_points(robotPose_.toPoint(), objectPose.toPoint());
    float distanceBetweenBoundary = distanceBetweenCenters - objectRadius - radius_;

    return (distanceBetweenBoundary < 0.0f) ? 0.0f : distanceBetweenBoundary;
}


float CircleCollisionModel::closestObstacle(const ObstacleDistanceGrid& obstacleDistanceGrid) const
{
    return std::max(0.0f, obstacleDistanceGrid.getObstacleDistance(robotPose_.toPoint()) - radius_);
}


float CircleCollisionModel::collisionArea(const ObstacleDistanceGrid& obstacleDistanceGrid) const
{
    float distanceToObstacleFromRobotCenter = obstacleDistanceGrid.getObstacleDistance(robotPose_.toPoint());
    if(distanceToObstacleFromRobotCenter < radius_)
    {
        return radius_ - distanceToObstacleFromRobotCenter; // TODO: this is just a measure of how bad the collision is, and not the actual area. need to check if this works.
    }
    
    return 0.0f;
}


} // namespace planner
} // namespace vulcan
