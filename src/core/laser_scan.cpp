/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_scan.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of polar_laser_scan_t, cartesian_laser_scan_t, and polar->cartesian conversion functions.
*/

#include <core/laser_scan.h>
#include <core/angle_functions.h>
#include <math/coordinates.h>
#include <core/float_comparison.h>
#include <cassert>
#include <cstring>

namespace vulcan
{

inline bool is_3dof_pose(const pose_6dof_t& pose)
{
    return absolute_fuzzy_equal(pose.phi, 0.0f) && absolute_fuzzy_equal(pose.rho, 0.0f);
}


// Internal optimized conversions for when the pose is 3-dof
void polar_to_cartesian_robot_3dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                   const pose_t&      laserPoseInRobotFrame,
                                   cartesian_laser_scan_t&   cartesianScanInRobotFrame,
                                   bool                      filterBadValues);

void polar_to_cartesian_global_3dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                    const pose_t&      laserPoseInRobotFrame,
                                    const pose_t&      robotPoseInGlobalFrame,
                                    cartesian_laser_scan_t&   cartesianScanInGlobalFrame,
                                    bool                      filterBadValues);

// Slow and steady 6dof version
void polar_to_cartesian_robot_6dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                   const pose_6dof_t& laserPoseInRobotFrame,
                                   cartesian_laser_scan_t&   cartesianScanInRobotFrame,
                                   bool                      filterBadValues);

void polar_to_cartesian_global_6dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                    const pose_6dof_t& laserPoseInRobotFrame,
                                    const pose_t&      robotPoseInGlobalFrame,
                                    cartesian_laser_scan_t&   cartesianScanInGlobalFrame,
                                    bool                      filterBadValues);


// Implementation polar->Cartesian functions
void polar_scan_to_cartesian_scan(const polar_laser_scan_t& polarScan,
                                  cartesian_laser_scan_t&   cartesianScan,
                                  bool                      filterBadValues)
{
    polar_scan_to_cartesian_scan_in_robot_frame(polarScan, pose_6dof_t(0, 0, 0), cartesianScan, filterBadValues);
}


void polar_scan_to_cartesian_scan_in_robot_frame(const polar_laser_scan_t& polarScanInRobotFrame,
                                                 cartesian_laser_scan_t&   cartesianScanInRobotFrame,
                                                 bool                      filterBadValues)
{
    polar_scan_to_cartesian_scan_in_robot_frame(polarScanInRobotFrame, polarScanInRobotFrame.offset, cartesianScanInRobotFrame, filterBadValues);
}


void polar_scan_to_cartesian_scan_in_global_frame(const polar_laser_scan_t& polarScanInRobotFrame,
                                                  const pose_t&      robotPoseInGlobalFrame,
                                                  cartesian_laser_scan_t&   cartesianScanInGlobalFrame,
                                                  bool                      filterBadValues)
{
    polar_scan_to_cartesian_scan_in_global_frame(polarScanInRobotFrame, polarScanInRobotFrame.offset, robotPoseInGlobalFrame, cartesianScanInGlobalFrame, filterBadValues);
}


void polar_scan_to_cartesian_scan_in_robot_frame(const polar_laser_scan_t& polarScanInLaserFrame,
                                                 const pose_6dof_t& laserPoseInRobotFrame,
                                                 cartesian_laser_scan_t&   cartesianScanInRobotFrame,
                                                 bool                      filterBadValues)
{
    if(cartesianScanInRobotFrame.numPoints != polarScanInLaserFrame.numRanges)
    {
        cartesianScanInRobotFrame.scanPoints.resize(polarScanInLaserFrame.numRanges);
        cartesianScanInRobotFrame.numPoints  = polarScanInLaserFrame.numRanges;
    }

    if(is_3dof_pose(laserPoseInRobotFrame))
    {
        polar_to_cartesian_robot_3dof(polarScanInLaserFrame,
                                      laserPoseInRobotFrame.toPose(),
                                      cartesianScanInRobotFrame,
                                      filterBadValues);
    }
    else
    {
        polar_to_cartesian_robot_6dof(polarScanInLaserFrame,
                                      laserPoseInRobotFrame,
                                      cartesianScanInRobotFrame,
                                      filterBadValues);
    }

    cartesianScanInRobotFrame.timestamp             = polarScanInLaserFrame.timestamp;
    cartesianScanInRobotFrame.offsetFromRobotCenter = laserPoseInRobotFrame.toPose();
}


void polar_scan_to_cartesian_scan_in_global_frame(const polar_laser_scan_t& polarScanInLaserFrame,
                                                  const pose_6dof_t& laserPoseInRobotFrame,
                                                  const pose_t&      robotPoseInGlobalFrame,
                                                  cartesian_laser_scan_t&   cartesianScanInGlobalFrame,
                                                  bool                      filterBadValues)
{
    if(cartesianScanInGlobalFrame.numPoints != polarScanInLaserFrame.numRanges)
    {
        cartesianScanInGlobalFrame.scanPoints.resize(polarScanInLaserFrame.numRanges);
        cartesianScanInGlobalFrame.numPoints  = polarScanInLaserFrame.numRanges;
    }

    if(is_3dof_pose(laserPoseInRobotFrame))
    {
        polar_to_cartesian_global_3dof(polarScanInLaserFrame,
                                       laserPoseInRobotFrame.toPose(),
                                       robotPoseInGlobalFrame,
                                       cartesianScanInGlobalFrame,
                                       filterBadValues);
    }
    else
    {
        polar_to_cartesian_global_6dof(polarScanInLaserFrame,
                                       laserPoseInRobotFrame,
                                       robotPoseInGlobalFrame,
                                       cartesianScanInGlobalFrame,
                                       filterBadValues);
    }

    cartesianScanInGlobalFrame.timestamp             = polarScanInLaserFrame.timestamp;
    cartesianScanInGlobalFrame.offsetFromRobotCenter = laserPoseInRobotFrame.toPose();
}


void polar_to_cartesian_robot_3dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                   const pose_t&      laserPoseInRobotFrame,
                                   cartesian_laser_scan_t&   cartesianScanInRobotFrame,
                                   bool                      filterBadValues)
{
    auto& scanPoints = cartesianScanInRobotFrame.scanPoints;

    float theta = angle_sum(polarScanInLaserFrame.startAngle, laserPoseInRobotFrame.theta);
    for(unsigned int i = 0; i < polarScanInLaserFrame.numRanges; ++i)
    {
        if(filterBadValues && polarScanInLaserFrame.ranges[i] <= 0)
        {
            scanPoints[i].x = 0;
            scanPoints[i].y = 0;
        }
        else
        {
            scanPoints[i].x = polarScanInLaserFrame.ranges[i]*cos(theta) + laserPoseInRobotFrame.x;
            scanPoints[i].y = polarScanInLaserFrame.ranges[i]*sin(theta) + laserPoseInRobotFrame.y;
        }

        theta += polarScanInLaserFrame.angularResolution;
    }
}


void polar_to_cartesian_global_3dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                                  const pose_t&      laserPoseInRobotFrame,
                                                  const pose_t&      robotPoseInGlobalFrame,
                                                  cartesian_laser_scan_t&   cartesianScanInGlobalFrame,
                                                  bool                      filterBadValues)
{
    float globalLaserX = robotPoseInGlobalFrame.x + (laserPoseInRobotFrame.x*cos(robotPoseInGlobalFrame.theta) - laserPoseInRobotFrame.y*sin(robotPoseInGlobalFrame.theta));
    float globalLaserY = robotPoseInGlobalFrame.y + (laserPoseInRobotFrame.x*sin(robotPoseInGlobalFrame.theta) + laserPoseInRobotFrame.y*cos(robotPoseInGlobalFrame.theta));
    float rayTheta     = polarScanInLaserFrame.startAngle + laserPoseInRobotFrame.theta + robotPoseInGlobalFrame.theta;

    auto& scanPoints = cartesianScanInGlobalFrame.scanPoints;

    for(unsigned int i = 0; i < polarScanInLaserFrame.numRanges; ++i)
    {
        if(filterBadValues && polarScanInLaserFrame.ranges[i] <= 0)
        {
            scanPoints[i].x = 0;
            scanPoints[i].y = 0;
        }
        else
        {
            scanPoints[i].x = polarScanInLaserFrame.ranges[i]*cos(rayTheta) + globalLaserX;
            scanPoints[i].y = polarScanInLaserFrame.ranges[i]*sin(rayTheta) + globalLaserY;
        }

        rayTheta += polarScanInLaserFrame.angularResolution;
    }
}


void polar_to_cartesian_robot_6dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                   const pose_6dof_t& laserPoseInRobotFrame,
                                   cartesian_laser_scan_t&   cartesianScanInRobotFrame,
                                   bool                      filterBadValues)
{
    auto& scanPoints = cartesianScanInRobotFrame.scanPoints;

    // Put points in laser frame, then transform them into the robot frame
    float theta = polarScanInLaserFrame.startAngle;
    float cosPhi = std::cos(laserPoseInRobotFrame.phi); // pitch
    float cosRho = std::cos(laserPoseInRobotFrame.rho); // roll
    float cosTheta = std::cos(laserPoseInRobotFrame.theta);
    float sinTheta = std::sin(laserPoseInRobotFrame.theta);

    Point<float> endpoint;

    for(unsigned int i = 0; i < polarScanInLaserFrame.numRanges; ++i)
    {
        if(filterBadValues && polarScanInLaserFrame.ranges[i] <= 0)
        {
            scanPoints[i].x = 0;
            scanPoints[i].y = 0;
        }
        else
        {
            endpoint.x = polarScanInLaserFrame.ranges[i]*cos(theta)*cosPhi;
            endpoint.y = polarScanInLaserFrame.ranges[i]*sin(theta)*cosRho;
            scanPoints[i].x = (cosTheta * endpoint.x) - (sinTheta * endpoint.y) + laserPoseInRobotFrame.x;
            scanPoints[i].y = (sinTheta * endpoint.x) + (cosTheta * endpoint.y) + laserPoseInRobotFrame.y;
        }

        theta += polarScanInLaserFrame.angularResolution;
    }
}


void polar_to_cartesian_global_6dof(const polar_laser_scan_t& polarScanInLaserFrame,
                                    const pose_6dof_t& laserPoseInRobotFrame,
                                    const pose_t&      robotPoseInGlobalFrame,
                                    cartesian_laser_scan_t&   cartesianScanInGlobalFrame,
                                    bool                      filterBadValues)
{
    polar_to_cartesian_robot_6dof(polarScanInLaserFrame,
                                  laserPoseInRobotFrame,
                                  cartesianScanInGlobalFrame,
                                  filterBadValues);

    auto& scanPoints = cartesianScanInGlobalFrame.scanPoints;

    float cosTheta = std::cos(robotPoseInGlobalFrame.theta);
    float sinTheta = std::sin(robotPoseInGlobalFrame.theta);
    Point<float> endpoint;

    for(unsigned int i = 0; i < polarScanInLaserFrame.numRanges; ++i)
    {
        endpoint = scanPoints[i];
        scanPoints[i].x = (cosTheta * endpoint.x) - (sinTheta * endpoint.y) + robotPoseInGlobalFrame.x;
        scanPoints[i].y = (sinTheta * endpoint.x) + (cosTheta * endpoint.y) + robotPoseInGlobalFrame.y;
    }
}

} // namespace vulcan
