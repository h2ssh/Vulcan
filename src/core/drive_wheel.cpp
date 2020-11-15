/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     drive_wheel.cpp
* \author   Collin Johnson
*
* Definition of utility functions for drive_wheel_t.
*/

#include <core/drive_wheel.h>
#include <iostream>

namespace vulcan
{

// Output operator: (wheelSpeed, motorAccel)
std::ostream& operator<<(std::ostream& out, const drive_wheel_t& driveWheel)
{
    out<<'('<<driveWheel.speed<<','<<driveWheel.motorAccel<<')';
    return out;
}

} // namespace vulcan
