/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/*
 *  glass.h
 *  This is the header that lets all of the glass modules know what types we need.
 *
 *  Created by Paul Foster on 10/27/12.
 *
 */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::vector;
using cv::Mat_;
using cv::Size;

//Sector Type
struct sector{
	//sectors represent sectors of a circle, with a width from [0,2pi).
	//If the endpoints are the same, it has width 0, i.e. a single angle.
	//If valid==0 the sector is empty.
	//All angles are interpreted as mod 2pi and the sector is the range 
	//  from lo to hi in the positive direction.
	//Sectors cannot represent a complete circle.
	cv::Mat_<double> lo   ; //the low angle of the sector
    cv::Mat_<double> hi   ; //the high angle of the sector
    cv::Mat_<bool> valid  ; //whether the sector contains any angles
    sector(){}
	sector(Size s){
		lo		= Mat_<double>(s);
		hi		= Mat_<double>(s);
		valid	= Mat_<bool>(s);
	}
};


