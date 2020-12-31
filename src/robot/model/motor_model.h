/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motor_model.h
* \author   Jong Jin Park and Paul Foster
*
*/

#include "core/drive_wheel.h"

#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

namespace vulcan
{

namespace robot
{

drive_wheel_t motor_model(const drive_wheel_t& wheelState,
                          double u_drive,
                          double h,
                          double mu,
                          double beta,
                          double gamma,
                          double alpha,
                          double* friction);

} // robot
} // vulcan

#endif // MOTOR_MODEL_H

