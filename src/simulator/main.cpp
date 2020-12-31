/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file 	main.cpp
*	\author Zongtai Luo
*	main is the main environment for running serveral mpepc controlled robot on simulated map.
*/

#include "utils/timestamp.h"
#include "simulator/simulator_utils.h"
#include "simulator/simulator_params.h"
#include "simulator/robot_group.h"

using namespace vulcan;

/**
* Options for running the program: 
*     -m   notslam starting the simulator with knowledge of the map
*     -case select case for running from 1, 2 and 3. 1 is the narrow corridor; 2 is the wide-narrow corridor and 3 is the T section
*/

int main(int argc, char** argv)
{

    // reading value from command line
    sim::simulator_set_up_t simulator_settings = sim::init_simulator_params(argc, argv);

    // loading config file
    utils::ConfigFile config_simulator("simulator_params.cfg");
    sim::simulator_params_t simulator_params = sim::load_simulator_params(config_simulator,simulator_settings.case_no);

    // Initialize robots
    sim::RobotObjectGroup robots_group(simulator_params);
    robots_group.subscribe(); // subscribe to motion command

    printf("%s\n", "Done Initialize");

    // Start simulation
    // refreshing rate(Hz): map 1, laser 40, encoder and odometry 125, local_pose 20.
    while(true) 
    {
        int64_t startTime = utils::system_time_us();

        robots_group.runGroupUpdates();

        // if (!robots_group.getLoopingFlag())
        // {
        //     robots_group.startGroupLooping();
        // }
        
        int64_t endTime = utils::system_time_us();
        if(endTime - startTime < 1000)
        {
            usleep(1000 - (endTime - startTime));
        }
    }

    return 0;
}
