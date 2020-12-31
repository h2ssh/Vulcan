/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/*
 * File:   glass.h
 * Author: Paul Foster
 *
 * Created on November 28, 2012, 8:00 PM
 */

#ifndef GLASS_H
#define GLASS_H
#include "glass_input.h"

void updateFromLaserScan(vulcan::glass::LaserScan& laser, cv::Size& gridsize);


#endif /* GLASS_H */
