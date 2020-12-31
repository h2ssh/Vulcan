/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_chart.cpp
 * \author   Jong Jin Park
 *
 * Definition of UnicycleChart, a collection of (bijective) mappings between
 * Cartesian and egocentric polar coordinates, anchored around a pose on a
 * plane.
 */

#include "mpepc/math/unicycle_chart.h"
#include "core/angle_functions.h"

namespace vulcan
{
namespace mpepc
{

UnicycleChart::UnicycleChart(const pose_t& targetPose, double smallRadius)
: targetPose_(targetPose)
, smallRadius_(smallRadius)
{
}

line_of_sight_t UnicycleChart::lineOfSight(const Point<float>& observerPosition) const
{
    line_of_sight_t los;

    float x = targetPose_.x - observerPosition.x;
    float y = targetPose_.y - observerPosition.y;

    los.range = sqrt(x * x + y * y);

    if (los.range < smallRadius_)   // avoid numerical unstability at r = 0.
    {
        los.angle = targetPose_.theta;
    } else {
        los.angle = atan2(y, x);
    }

    return los;
}

reduced_egocentric_polar_coords_t UnicycleChart::point2rp(const Point<float>& point) const
{
    reduced_egocentric_polar_coords_t coords;

    line_of_sight_t los = this->lineOfSight(point);

    coords.r = los.range;
    coords.phi = wrap_to_pi(targetPose_.theta - los.angle);
    coords.lineOfSightAngle = los.angle;

    return coords;
}

Point<float> UnicycleChart::rp2point(double r, double phi) const
{
    Point<float> point;

    float angle = targetPose_.theta - phi;

    point.x = targetPose_.x - r * cos(angle);
    point.y = targetPose_.y - r * sin(angle);

    return point;
}

Point<float> UnicycleChart::rp2point(const reduced_egocentric_polar_coords_t& coords) const
{
    return rp2point(coords.r, coords.phi);
}

egocentric_polar_coords_t UnicycleChart::pose2rpd(const pose_t& robotPose) const
{
    egocentric_polar_coords_t coords;

    reduced_egocentric_polar_coords_t reducedCoords = this->point2rp(targetPose_.toPoint());

    coords.r = reducedCoords.r;
    coords.phi = reducedCoords.phi;
    coords.delta = wrap_to_pi(robotPose.theta - reducedCoords.lineOfSightAngle);
    coords.lineOfSightAngle = reducedCoords.lineOfSightAngle;

    return coords;
}

pose_t UnicycleChart::rpd2pose(double r, double phi, double delta) const
{
    pose_t pose;

    double angle = targetPose_.theta - phi;

    pose.x = targetPose_.x - r * cos(angle);
    pose.y = targetPose_.y - r * sin(angle);
    pose.theta = wrap_to_pi(angle + delta);

    return pose;
}

pose_t UnicycleChart::rpd2pose(const egocentric_polar_coords_t& coords) const
{
    return rpd2pose(coords.r, coords.phi, coords.delta);
}

pose_t UnicycleChart::rpd2target(double r, double phi, double delta, const pose_t& robotPose) const
{
    pose_t pose;

    double angle = robotPose.theta - delta;

    pose.x = robotPose.x + r * cos(angle);
    pose.y = robotPose.y + r * sin(angle);
    pose.theta = wrap_to_pi(angle + phi);

    return pose;
}

pose_t UnicycleChart::rpd2target(const egocentric_polar_coords_t& coords, const pose_t& robotPose) const
{
    return rpd2target(coords.r, coords.phi, coords.delta, robotPose);
}

}   // namespace mpepc
}   // namespace vulcan
