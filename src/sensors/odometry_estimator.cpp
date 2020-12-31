/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     odometry_estimator.cpp
* \author   Collin Johnson
*
* Definition of create_odometry_estimator.
*/

#include "sensors/odometry_estimator.h"
#include "sensors/wheel_encoders.h"
#include "sensors/wheel_encoders_params.h"
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace sensors
{

std::unique_ptr<OdometryEstimator> create_odometry_estimator(const std::string& type, const utils::ConfigFile& config)
{
    if(type == WHEEL_ENCODERS_TYPE)
    {
        wheel_encoders_params_t params = load_wheel_encoders_params(config);
        return create_wheel_encoders(params.encoderType, params);
    }

    std::cerr<<"ERROR: create_odometry_estimator: Unknown estimator type: "<<type<<std::endl;
    assert(false);

    return std::unique_ptr<OdometryEstimator>();
}

}
}
