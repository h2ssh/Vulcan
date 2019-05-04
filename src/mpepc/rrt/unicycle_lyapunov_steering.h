/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unicycle_lyapunov_steering.h
* \author   Jong Jin Park
*
* Declaration of UnicycleLyapunovSteering, which implements numerically stable
* steering-along-vector-field via control-Lyapunov function.
*/

#ifndef UNICYCLE_LYAPUNOV_STEERING_H
#define UNICYCLE_LYAPUNOV_STEERING_H

#include <core/pose.h>
#include <vector>

namespace vulcan
{
namespace mpepc
{

class UnicycleLyapunovDistance;

// struct containing a path segment
struct unicycle_path_segment_t
{
    std::vector<pose_t>      steps;
    std::vector<Point<float>> path;
};

// parameters for UnicycleSteering
struct unicycle_lyapunov_steering_params_t
{
    double incrementalRotation = 0.03;
    double incrementalTranslation = 0.03;
};

class UnicycleLyapunovSteering
{
public:

    /**
    * Constructor for UnicycleLyapunovSteering
    */
    explicit UnicycleLyapunovSteering(const unicycle_lyapunov_steering_params_t& params);

    UnicycleLyapunovSteering(void) {};

    unicycle_path_segment_t extend(const pose_t& robotPose, const UnicycleLyapunovDistance& lyap, double maxExtension) const;

    unicycle_path_segment_t steer (const pose_t& robotPose, const UnicycleLyapunovDistance& lyap) const;

    std::vector<Point<float>> steerAway(const Point<float>& point, const UnicycleLyapunovDistance& lyap, double maxExtension) const;

private:

    pose_t incrementalRotation   (const pose_t& robotPose, const UnicycleLyapunovDistance& lyap, double maxRotation, bool* isAligned) const;
    pose_t incrementalTranslation(const pose_t& robotPose, const UnicycleLyapunovDistance& lyap, double maxTranslation, std::vector<Point<float>>* points, bool* isConverged) const;

    unicycle_lyapunov_steering_params_t params_;
};

} // mpepc
} // vulcan

#endif // UNICYCLE_LYAPUNOV_STEERING_H
