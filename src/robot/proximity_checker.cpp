/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     proximity_checker.cpp
* \author   Collin Johnson
*
* Definition of ProximityChecker.
*/

#include "robot/proximity_checker.h"
#include "robot/proximity_warning_indices.h"
#include "robot/commands.h"
#include "math/geometry/rectangle.h"
#include <cassert>
#include <iostream>

#define DEBUG_BOUNDARY
#define DEBUG_PROXIMITY
// #define DEBUG_SLOWDOWN_FACTOR

namespace vulcan
{
namespace robot
{

struct point_of_interest_t
{
    Point<float> point;
    float              originalDistance;
    float              transformedDistance;

    point_of_interest_t(const Point<float>& point = Point<float>())
        : point(point)
        , originalDistance(0.0f)
        , transformedDistance(0.0f)
    {
    }
};

std::vector<point_of_interest_t> find_points_in_proximity_boundary(const std::vector<Point<float>>& scanPoints,
                                                                   const math::Rectangle<float>&          warningBoundary,
                                                                   const math::Rectangle<float>&          criticalBoundary,
                                                                   proximity_warning_indices_t&           boundaryIndices);

bool scan_point_is_in_boundary(const Point<float>&     scanPoint,
                               const math::Rectangle<float>& proximityBound);

pose_t calculate_pose_transform(const velocity_command_t& command, float timespan);
math::Rectangle<float> transform_boundary(const math::Rectangle<float>& boundary,
                                          const pose_t&          transform);
void calculate_transformed_distance(const math::Rectangle<float>&     boundary,
                                    std::vector<point_of_interest_t>& pointsOfInterest);
float select_slowdown_factor    (const std::vector<point_of_interest_t>& points, const proximity_checker_params_t& params);
float calculate_slowdown_factor (const point_of_interest_t& point, const proximity_checker_params_t& params);


ProximityChecker::ProximityChecker(const proximity_checker_params_t& params)
    : slowdownFactorIsActive(false)
    , params(params)
{
    warningBoundary.bottomLeft  = params.criticalBoundary.bottomLeft  + Point<float>(-params.warningRadius, -params.warningRadius);
    warningBoundary.bottomRight = params.criticalBoundary.bottomRight + Point<float>(params.warningRadius,  -params.warningRadius);
    warningBoundary.topLeft     = params.criticalBoundary.topLeft     + Point<float>(-params.warningRadius,  params.warningRadius);
    warningBoundary.topRight    = params.criticalBoundary.topRight    + Point<float>(params.warningRadius,   params.warningRadius);

    #ifdef DEBUG_BOUNDARY
    std::cout<<"INFO:ProximityChecker:Warning boundary:"<<warningBoundary.bottomLeft<<' '<<warningBoundary.topRight<<'\n';
    std::cout<<"INFO:ProximityChecker:Critical boundary:"<<params.criticalBoundary.bottomLeft<<' '<<params.criticalBoundary.topRight<<'\n';
    #endif
}


ProximityChecker::~ProximityChecker(void)
{
}


velocity_command_t ProximityChecker::adjustCommandIfNeeded(const velocity_command_t&              command,
                                                           const std::vector<Point<float>>& scanPoints,
                                                           proximity_warning_indices_t&           warningIndices)
{
    // For the near term, do the simple thing of just setting the velocity to zero if any scan point is within the
    // proximity rectangle of the robot

    std::vector<point_of_interest_t> pointsOfInterest = find_points_in_proximity_boundary(scanPoints,
                                                                                          warningBoundary,
                                                                                          params.criticalBoundary,
                                                                                          warningIndices);

    pose_t commandTransform = calculate_pose_transform(command, params.lookaheadTime);

    math::Rectangle<float> transformedBoundary = transform_boundary(params.criticalBoundary, commandTransform);

    calculate_transformed_distance(transformedBoundary, pointsOfInterest);

    float slowdownFactor = 1.0f - select_slowdown_factor(pointsOfInterest, params);

    velocity_command_t adjustedCommand = command;
    adjustedCommand.linear  *= slowdownFactor;
    adjustedCommand.angular *= slowdownFactor;

    slowdownFactorIsActive = slowdownFactor < 0.9999f ? true : false;
    
#ifdef DEBUG_PROXIMITY
    // Only display the proximity data if there is actually a point in the boundary
    if(pointsOfInterest.size() > 0)
    {
        std::cout<<"INFO:ProximityChecker:Warning::"<<warningIndices.warningIndices.size()<<" Critical:"<<warningIndices.criticalIndices.size()<<'\n';
        std::cout<<"INFO:ProximityChecker:Initial:("<<command.linear<<','<<command.angular<<") Adjusted:("<<adjustedCommand.linear<<','<<adjustedCommand.angular<<")\n";
        std::cout<<"INFO:ProximityChecker:Slowdown:"<<slowdownFactor<<'\n';
    }
#endif

    if(scanPoints.empty())
    {
        adjustedCommand.linear  = 0.0f;
        adjustedCommand.angular = 0.0f;
    }

    return adjustedCommand;
}


std::vector<point_of_interest_t> find_points_in_proximity_boundary(const std::vector<Point<float>>& scanPoints,
                                                                   const math::Rectangle<float>&          warningBoundary,
                                                                   const math::Rectangle<float>&          criticalBoundary,
                                                                   proximity_warning_indices_t&           boundaryIndices)
{
    std::vector<point_of_interest_t> interestPoints;

    for(int n = scanPoints.size(); --n >= 0;)
    {
        if(scan_point_is_in_boundary(scanPoints[n], warningBoundary))
        {
            point_of_interest_t newPoint(scanPoints[n]);
            newPoint.originalDistance = criticalBoundary.distanceToPoint(newPoint.point);

            interestPoints.push_back(newPoint);

            // only need to check the critical boundary for points already in the warning boundary
            // don't add the indices in two places either, just critical or warning
            if(newPoint.originalDistance == 0.0f)
            {
                boundaryIndices.criticalIndices.push_back(n);
            }
            else // only inside the warning boundary
            {
                boundaryIndices.warningIndices.push_back(n);
            }
        }
    }

    return interestPoints;
}


bool scan_point_is_in_boundary(const Point<float>&     scanPoint,
                               const math::Rectangle<float>& proximityBound)
{
    return (scanPoint.x != 0.0f || scanPoint.y != 0.0f) && proximityBound.contains(scanPoint);
}


pose_t calculate_pose_transform(const velocity_command_t& command, float timespan)
{
    float distance = command.linear  * timespan;
    float rotation = command.angular * timespan;

    return pose_t(distance * cos(rotation/2),
                         distance * sin(rotation/2),
                         rotation);
}


math::Rectangle<float> transform_boundary(const math::Rectangle<float>& boundary,
                                          const pose_t&          transform)
{
    math::Rectangle<float> transformed;
    Point<float>     robotPosition(transform.toPoint());

    transformed.bottomLeft  = rotate(boundary.bottomLeft,  transform.theta) + robotPosition;
    transformed.bottomRight = rotate(boundary.bottomRight, transform.theta) + robotPosition;
    transformed.topLeft     = rotate(boundary.topLeft,     transform.theta) + robotPosition;
    transformed.topRight    = rotate(boundary.topRight,    transform.theta) + robotPosition;

    return transformed;
}


void calculate_transformed_distance(const math::Rectangle<float>&     boundary,
                                    std::vector<point_of_interest_t>& pointsOfInterest)
{
    for(auto& point : pointsOfInterest)
    {
        point.transformedDistance = boundary.distanceToPoint(point.point);
    }
}


float select_slowdown_factor(const std::vector<point_of_interest_t>& points, const proximity_checker_params_t& params)
{
    float maxSlowdown = 0.0f;
    float slowdown    = 0.0f;

    // An object has to be large enough to consider actually dodging. Otherwise, the reading
    // is probably just some sensor noise.
    if(points.size() > 5)
    {
        for(int i = points.size(); --i >= 0;)
        {
            slowdown = calculate_slowdown_factor(points[i], params);

            if(slowdown > maxSlowdown)
            {
                maxSlowdown = slowdown;
            }
        }
    }

    return maxSlowdown;
}


float calculate_slowdown_factor(const point_of_interest_t& point, const proximity_checker_params_t& params)
{
    /*
    * To get the slowdown, if the transformed distance is less than the original distance, then the object is moving
    * closer. The slowdown is based on the transformed distance and the warning radius. The velocity scales linearly between
    * the warning radius and the critical boundary.
    */

    if((point.originalDistance < point.transformedDistance) || (point.transformedDistance > params.warningRadius))
    {
        return 0.0f;
    }
    else if(point.transformedDistance == 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return params.minSlowdownFactor + (params.maxSlowdownFactor-params.minSlowdownFactor) * (1.0f - point.transformedDistance/params.warningRadius);
    }
}

} // namespace robot
} // namespace vulcan
