/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <iostream>
#include <iomanip>
#include "utils/timestamp.h"
#include "core/laser_scan.h"
#include "sensors/hokuyo_urg_laser.h"


using vulcan::polar_laser_scan_t;
using vulcan::sensors::HokuyoURGLaser;
using vulcan::utils::system_time_us;


// Helper function for displaying a subset of the scan data
void display_laser_scan_data(const polar_laser_scan_t& scan);


/**
* hokuyo_urg_laser_test tests connecting to the URG laser and acquiring data. 
*
* Command-line: hokuyo_urg_laser_test < laser port >
*/
int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout<<"Expected command-line: hokuyo_urg_laser_test <laser port>"<<std::endl;
        return -1;
    }
    
    HokuyoURGLaser     laser(argv[1], 0);
    polar_laser_scan_t scan;
    
    laser.startRangefinder();
    
    int64_t startTime = 0;
    int64_t endTime   = 0;
    int64_t deltaTime = 0;
    
    int numReadings = 0;
    
    while(1)
    {
        startTime = system_time_us();
        
        laser.getLaserScan(scan);
        
        endTime    = system_time_us();        
        deltaTime += endTime - startTime;
        
        if(deltaTime > vulcan::utils::sec_to_usec(1))
        {
            std::cout<<"Laser update rate: "<<(static_cast<float>(numReadings)*vulcan::utils::usec_to_sec(deltaTime))<<" Hz"<<std::endl;
            
            numReadings = 0;
            deltaTime   = 0;
        }
        
        if(numReadings % 40 == 0)
        {
            display_laser_scan_data(scan);
        }
        
        ++numReadings;
    }

    return 0;
}


// Helper function for displaying a subset of the scan data
void display_laser_scan_data(const polar_laser_scan_t& scan)
{
    // Display the middle 5 values along with resolution and number of ranges
    int numRangesToShow = scan.numRanges;
    int firstRangeToShow = 0;//scan.numRanges/2 - NUM_RANGES_TO_SHOW/2;
    
    std::cout<<"Res: "<<scan.angularResolution<<" Num: "<<scan.numRanges;
    
    for(int x = firstRangeToShow; x < firstRangeToShow + numRangesToShow; ++x)
    {
        std::cout<<std::setprecision(3)<<' '<<scan.ranges[x];
    }
    
    std::cout<<std::endl;
}
