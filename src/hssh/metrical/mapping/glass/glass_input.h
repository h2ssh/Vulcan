/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/*
 *  glass_input.h
 *  Created by Paul Foster on 10/24/12.
 *
 */

/*
Inputs (in glass_input.h):
laser.range		//the range to the target in meters
     .inten		//intensity of the return, >100 is reasonable intensity, >1000 is high confidence, >4000 is brighter than brightest white
     .ang       //the angle of the ray in world coordinates
     .lo		//lowest scan angle in the scan
	 .hi		//highest scan angle in the scan
     .uncert    //the uncertainty of the ray (not used right now, in theory should be)
     .x         //x in grid coords of ray tip (0,0 is upper left corner of upper left pixel)
     .y         //y in grid coords
     
bot.x        //x in grid coords
   .y        //y in grid coords
   .dir      //the direction of motion of the robot as an angle in world coords

gridsize				 //the size of the grid
*/
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
namespace vulcan{
	namespace glass{
		//Constants that should be changed to fit application
		double BEAM_WIDTH=(.25*M_PI/180.0);//the angular width of the laser beam, usually ~= to the angular step size

		//The Scan:
		struct LaserScan{
			std::vector<double> range;
			std::vector<double> inten;
			std::vector<double> ang;
			double		   lo;
			double		   hi;
			double		   uncert;
			std::vector<double> x;
			std::vector<double> y;
		};
		LaserScan laser;

		//The Robot (really lidar) Location:
		struct BotLoc{
			double x;
			double y;
			double dir;
		};
		BotLoc bot;

		//The size of the grid
		cv::Size gridsize;
	}
}
