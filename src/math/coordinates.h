/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     coordinates.h
 * \author   Collin Johnson
 *
 * A collection of functions for converting between the robot and global coordinate systems.
 */

#ifndef MATH_COORDINATES_H
#define MATH_COORDINATES_H

#include "core/angle_functions.h"
#include "core/point.h"
#include "core/pose.h"
#include "math/geometry/polygon.h"
#include "math/geometry/rectangle.h"
#include <boost/concept_check.hpp>
#include <cmath>

namespace vulcan
{
namespace math
{

/**
 * ReferenceFrame defines possible references frames used on the robot. The robot frame is egocentric. A local frame
 * is a frame for a local area and is relative to the center of area. The global frame is the current LPM frame or the
 * reference frame of the relaxed global topo map.
 */
enum class ReferenceFrame
{
    ROBOT,
    LOCAL,
    GLOBAL
};

template <typename T>
Point<T> convert_reference_frame(const Point<T>& pointInFromFrame,
                                 ReferenceFrame from,
                                 ReferenceFrame to,
                                 const pose_t& originInToFrame)
{
    if (from == to) {
        return pointInFromFrame;
    }

    Point<T> toFrame;

    if ((from == ReferenceFrame::LOCAL) && (to == ReferenceFrame::GLOBAL)) {
        float rotatedX =
          pointInFromFrame.x * cos(originInToFrame.theta) - pointInFromFrame.y * sin(originInToFrame.theta);
        float rotatedY =
          pointInFromFrame.x * sin(originInToFrame.theta) + pointInFromFrame.y * cos(originInToFrame.theta);

        toFrame.x = rotatedX + originInToFrame.x;
        toFrame.y = rotatedY + originInToFrame.y;
    } else if ((from == ReferenceFrame::GLOBAL) && (to == ReferenceFrame::LOCAL)) {
        float rotatedX =
          pointInFromFrame.x * cos(originInToFrame.theta) + pointInFromFrame.y * sin(originInToFrame.theta);
        float rotatedY =
          -pointInFromFrame.x * sin(originInToFrame.theta) + pointInFromFrame.y * cos(originInToFrame.theta);

        toFrame.x = rotatedX - originInToFrame.x;
        toFrame.y = rotatedY - originInToFrame.y;
    }

    return toFrame;
}

inline pose_t convert_reference_frame(const pose_t& poseInFromFrame,
                                      ReferenceFrame from,
                                      ReferenceFrame to,
                                      const pose_t& originInToFrame)
{
    float newTheta = poseInFromFrame.theta;

    if ((from == ReferenceFrame::LOCAL) && (to == ReferenceFrame::GLOBAL)) {
        newTheta = angle_sum(poseInFromFrame.theta, originInToFrame.theta);
    } else if ((from == ReferenceFrame::GLOBAL) && (to == ReferenceFrame::LOCAL)) {
        newTheta = angle_diff(poseInFromFrame.theta, originInToFrame.theta);
    }

    return pose_t{convert_reference_frame(poseInFromFrame.toPoint(), from, to, originInToFrame), newTheta};
}

template <typename T>
Rectangle<T> convert_reference_frame(const math::Rectangle<T>& rectInFromFrame,
                                     ReferenceFrame from,
                                     ReferenceFrame to,
                                     const pose_t& originInToFrame)
{
    return Rectangle<T>(convert_reference_frame(rectInFromFrame.topLeft, from, to, originInToFrame),
                        convert_reference_frame(rectInFromFrame.topRight, from, to, originInToFrame),
                        convert_reference_frame(rectInFromFrame.bottomRight, from, to, originInToFrame),
                        convert_reference_frame(rectInFromFrame.bottomLeft, from, to, originInToFrame));
}

template <typename T>
Polygon<T> convert_reference_frame(const math::Polygon<T>& polyInFromFrame,
                                   ReferenceFrame from,
                                   ReferenceFrame to,
                                   const pose_t& originInToFrame)
{
    std::vector<Point<T>> toVertices;
    toVertices.reserve(polyInFromFrame.size());
    for (auto& vertex : polyInFromFrame) {
        toVertices.push_back(convert_reference_frame(vertex, from, to, originInToFrame));
    }
    return Polygon<T>(toVertices);
}

template <typename T>
Point<T> robot_frame_to_global_frame(const Point<T>& pointInRobotFrame, const pose_t& robotGlobalPose)
{
    Point<T> globalPoint;

    float rotatedX =
      pointInRobotFrame.x * cos(robotGlobalPose.theta) - pointInRobotFrame.y * sin(robotGlobalPose.theta);
    float rotatedY =
      pointInRobotFrame.x * sin(robotGlobalPose.theta) + pointInRobotFrame.y * cos(robotGlobalPose.theta);

    globalPoint.x = rotatedX + robotGlobalPose.x;
    globalPoint.y = rotatedY + robotGlobalPose.y;

    return globalPoint;
}


template <typename T>
Point<T> global_frame_to_robot_frame(const Point<T>& pointInGlobalFrame, const pose_t& robotGlobalPose)
{
    Point<T> robotPoint;

    float rotatedX =
      pointInGlobalFrame.x * cos(robotGlobalPose.theta) + pointInGlobalFrame.y * sin(robotGlobalPose.theta);
    float rotatedY =
      -pointInGlobalFrame.x * sin(robotGlobalPose.theta) + pointInGlobalFrame.y * cos(robotGlobalPose.theta);

    robotPoint.x = rotatedX - robotGlobalPose.x;
    robotPoint.y = rotatedY - robotGlobalPose.y;

    return robotPoint;
}


inline pose_t robot_frame_to_global_frame(const pose_t& poseInRobotFrame, const pose_t& robotGlobalPose)
{
    pose_t globalPose;

    float rotatedX = poseInRobotFrame.x * cos(robotGlobalPose.theta) - poseInRobotFrame.y * sin(robotGlobalPose.theta);
    float rotatedY = poseInRobotFrame.x * sin(robotGlobalPose.theta) + poseInRobotFrame.y * cos(robotGlobalPose.theta);

    globalPose.x = rotatedX + robotGlobalPose.x;
    globalPose.y = rotatedY + robotGlobalPose.y;
    globalPose.theta = angle_sum(poseInRobotFrame.theta, robotGlobalPose.theta);

    return globalPose;
}


inline pose_t global_frame_to_robot_frame(const pose_t& poseInGlobalFrame, const pose_t& robotGlobalPose)
{
    pose_t robotPose;

    float rotatedX =
      poseInGlobalFrame.x * cos(robotGlobalPose.theta) + poseInGlobalFrame.y * sin(robotGlobalPose.theta);
    float rotatedY =
      -poseInGlobalFrame.x * sin(robotGlobalPose.theta) + poseInGlobalFrame.y * cos(robotGlobalPose.theta);

    robotPose.x = rotatedX - robotGlobalPose.x;
    robotPose.y = rotatedY - robotGlobalPose.y;
    robotPose.theta = angle_diff(poseInGlobalFrame.theta, robotGlobalPose.theta);

    return robotPose;
}

}   // namespace math
}   // namespace vulcan

#endif   // MATH_COORDINATE_TRANSFORMS_H
