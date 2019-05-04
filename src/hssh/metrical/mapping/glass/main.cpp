/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/* 
 * File:   main.cpp
 * Author: paul
 *
 * Created on November 14, 2012, 1:54 PM
 */

#include <cstdlib>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "glass.h"
#define inf (std::numeric_limits<double>::infinity())
using namespace cv;  // The new C++ interface API is inside this namespace. Import it.
using namespace std;
using namespace vulcan::glass;


/*
 * 
 */
int main(int argc, char** argv) {
	Mat_<double> M = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, -INFINITY, 0, INFINITY, NAN);
	M.at<double>(0,2)=.707;
	Mat M2=M*2;
	FileStorage fs("test.xml", FileStorage::WRITE);
	fs<<"Mat1"<<M;
	fs<<"Mat2"<<M2;
	Mat inten, range, ang, lo, hi, laserx, lasery, botlocx,  botlocy, dir;
	FileStorage fs2("bbbdat.xml", FileStorage::READ);
	fs2["inten"]>>inten;
	fs2["range"]>>range;
	fs2["ang"]>>ang;
	fs2["lo"]>>lo;
	fs2["hi"]>>hi;
	fs2["laserx"]>>laserx;
	fs2["lasery"]>>lasery;
	fs2["botlocx"]>>botlocx;
	fs2["botlocy"]>>botlocy;
	fs2["dir"]>>dir;
	
	
	
	for(int i=0;i<inten.rows;i+=1){
		laser.inten=std::vector<double>((double*)inten.ptr(i),	(double*)inten.ptr(i)+inten.cols);
		laser.range=std::vector<double>((double*)range.ptr(i),	(double*)range.ptr(i)+range.cols);
		laser.ang=	std::vector<double>((double*)ang.ptr(i),	(double*)ang.ptr(i)+ang.cols);
		laser.lo=lo.at<double>(i);
		laser.hi=hi.at<double>(i);
		laser.x=std::vector<double>((double*)laserx.ptr(i),(double*)laserx.ptr(i)+laserx.cols);
		laser.y=std::vector<double>((double*)lasery.ptr(i),(double*)lasery.ptr(i)+lasery.cols);

		bot.x=botlocx.at<double>(i);
		bot.y=botlocy.at<double>(i);
		bot.dir=dir.at<double>(i);
		
		gridsize=Size(1017,1017);

		//calculateLabels();
		updateFromLaserScan(laser,gridsize);
		

	}
	cvWaitKey();
	
	/*
    std::ifstream fin("num.txt");
    double d;
    fin >> d;
    std::cout << d << '\n';
    return 0;*/
}

