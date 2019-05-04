/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "glass_interface.h"
#include "glass_input.h"

using namespace vulcan;
void vulcan::glass::update_glass(const pose_t& currentPose, const polar_laser_scan_t& slamLaser){
	//load up bot pose
	static pose_t oldPose=currentPose;//the pose in global coordinates
	glass::bot.x=currentPose.x-oldPose.x;
	glass::bot.y=currentPose.y-oldPose.y;
	glass::bot.dir=currentPose.theta-oldPose.theta;
	
	//load up laser scan
	static cartesian_laser_scan_t cartesianScan;//the cartesian scan holder
	polar_scan_to_cartesian_scan_in_global_frame(slamLaser, currentPose, cartesianScan, true);
	
	glass::laser.x=std::vector<double>(cartesianScan.numPoints);
	glass::laser.y=std::vector<double>(cartesianScan.numPoints);
	glass::laser.ang=std::vector<double>(cartesianScan.numPoints);
	for (int i=1;i<cartesianScan.numPoints;i++){
		Point<float>& xy=cartesianScan.scanPoints[i];
		glass::laser.x[i]=xy.x;
		glass::laser.y[i]=xy.y;
		glass::laser.ang[i]=glass::bot.dir + slamLaser.offset.theta + slamLaser.startAngle + slamLaser.angularResolution*i;
		
	}
	glass::laser.lo=glass::bot.dir + slamLaser.offset.theta + slamLaser.startAngle;
	glass::laser.hi=glass::bot.dir + slamLaser.offset.theta + slamLaser.startAngle + slamLaser.angularResolution*cartesianScan.numPoints;
	
	
}
