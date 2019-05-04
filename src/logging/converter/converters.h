/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     converters.h
* \author   Collin Johnson
* 
* Declaration of conversion functions for log:
* 
*   - convert_lcm_to_dpslam : convert an LCM log to a dpslam log
* 
*/

#ifndef LOGGING_CONVERTER_CONVERTERS_H
#define LOGGING_CONVERTER_CONVERTERS_H

#include <string>

namespace vulcan
{
namespace logging
{
    
/**
* convert_lcm_to_dpslam converts from the Vulcan LCM logs to the format used for 
* DP-SLAM. The filename input is filename and the output is filename.dpslam.
* 
* The input LCM log needs to be readable by the lcm::LogFile class. The output
* format is:
* 
*   Laser < sequence of laser data >
*   Odometry x y theta
* 
* Unfortunately, the parameters of the laser must be hard-coded into the compiled version of DP-SLAM.
* If the robot configuration changes, then those parameters need to be changed. The files to be changed
* and the associated parameters will be output to stdout when the conversion process finishes.
* 
* \param    filename                The name of the LCM log to be loaded
* \param    useOldLogFormat         Flag indicating if the old 
* \return   True if the log was converted successfully.
*/
bool convert_lcm_to_dpslam(const std::string& filename, bool useOldLogFormat);
    
}
}

#endif // LOGGING_CONVERTER_CONVERTERS_H
