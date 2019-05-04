/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     log_converter.cpp
* \author   Collin Johnson
* 
*/

#include <logging/converter/converters.h>

using namespace vulcan;

int main(int argc, char** argv)
{
    logging::convert_lcm_to_dpslam(argv[1], true);
    
    return 0;
}
