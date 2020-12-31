/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include<core/pose.h"
#include<core/laser_scan.h"

namespace vulcan{
	namespace glass{
		void update_glass(const pose_t& currentPose, const polar_laser_scan_t& sensorData);
	}
}



