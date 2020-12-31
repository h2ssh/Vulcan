/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motor_model.cpp
* \author   Jong Jin Park and Paul Foster
*
*/

#include "robot/model/motor_model.h"
#include <cmath>

namespace vulcan
{
namespace robot
{

inline double sign(double x) { return double((x > 0) - (x < 0)); }

drive_wheel_t motor_model(const drive_wheel_t& wheelState,
                          double u_drive,
                          double h,
                          double mu,
                          double beta,
                          double gamma,
                          double alpha,
                          double* friction)
{
    // All calculations are 0 if there is no time delta
    if(h == 0.0)
    {
        return wheelState;
    }

    drive_wheel_t nextWheelState;

    // non-linear friction model for discrete time system
    double friction_threshold = -wheelState.speed/h - wheelState.motorAccel;
    double u_friction;

    if(fabs(friction_threshold) <= mu)
    {
        u_friction = friction_threshold;
    }
    else
    {
        u_friction = sign(friction_threshold)*mu;
    }

    // report friction as needed
    if(friction)
    {
        *friction = u_friction;
    }

    // state propagation without friction and control input
    nextWheelState.speed      =  wheelState.speed + h*wheelState.motorAccel;
    nextWheelState.motorAccel = -beta*h*wheelState.speed + (1-gamma*h)*wheelState.motorAccel;

    // add acceleration from friction
    nextWheelState.speed      += h*u_friction;

    // add control input to motor acceleration
    nextWheelState.motorAccel += alpha*h*u_drive;

    return nextWheelState;
}

} // namespace planner
} // namespace vulcan
