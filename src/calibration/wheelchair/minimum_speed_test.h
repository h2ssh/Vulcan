/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     minimum_speed_test.h
* \author   Collin Johnson
*
* Declaration of:
*   - MinimumSpeedTest abstract base class
*   - MinimumStartSpeedTest subclass
*   - MinimumRollingSpeedTest subclass
*/

#ifndef CALIBRATION_WHEELCHAIR_MINIMUM_SPEED_TEST_H
#define CALIBRATION_WHEELCHAIR_MINIMUM_SPEED_TEST_H

#include <calibration/wheelchair/wheelchair_test.h>

namespace vulcan
{
namespace calibration
{

/**
* MinimumSpeedTest
*/
class MinimumSpeedTest : public WheelchairTest
{
public:

private:
};

}
}

#endif // CALIBRATION_WHEELCHAIR_MINIMUM_SPEED_TEST_H
