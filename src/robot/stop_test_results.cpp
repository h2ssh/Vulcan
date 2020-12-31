/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "sensors/sensor_log.h"
#include <iostream>

using namespace vulcan;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: stop_test_results 'stop log file'\n";
        return 1;
    }

    sensors::SensorLog log(argv[1]);


    return 0;
}
