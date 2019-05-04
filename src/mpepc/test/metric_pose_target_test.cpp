/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <lcmtypes/mpepc/metric_pose_target_t.h>
#include <lcmtypes/mpepc/metric_pose_target_status_t.h>
#include <system/module_communicator.h>
#include <utils/command_line.h>
#include <iostream>
#include <random>
#include <string>
#include <cassert>
#include <ctime>

using namespace vulcan;


// Write a class to listen for the status message and output it until the target has been reached
class StatusListener
{
public:
    
    bool isDone(void) const { return taskCompleted_; }
    
    void handleData(const vulcan_lcm::metric_pose_target_status_t& status, const std::string& channel)
    {
        std::cout << "INFO: Target status for task " << status.target_id << ": ";
        
        switch(status.status)
        {
        case vulcan_lcm::metric_pose_target_status_t::IDLE:
            std::cout << "idle";
            break;
            
        case vulcan_lcm::metric_pose_target_status_t::IN_PROGRESS:
            std::cout << "in progress";
            break;
            
        case vulcan_lcm::metric_pose_target_status_t::REACHED_TARGET:
            taskCompleted_ = true;
            std::cout << "SUCCESS: reached target";
            break;
            
        case vulcan_lcm::metric_pose_target_status_t::FAILED_CANNOT_FIND_SOLUTION:
            std::cout << "FAILURE: cannot find solution";
            break;
            
        case vulcan_lcm::metric_pose_target_status_t::FAILED_ROBOT_IS_STUCK:
            std::cout << "FAILURE: robot is stuck";
            break;
            
        case vulcan_lcm::metric_pose_target_status_t::FAILED_TARGET_CANNOT_BE_ASSIGNED:
            std::cout << "FAILURE: target cannot be assigned";
            break;
        }
        
        std::cout << '\n';
    }
    
private:
    
    bool taskCompleted_ = false;
};


int main(int argc, char** argv)
{
    vulcan_lcm::metric_pose_target_t target;
    target.id = 1001;
    target.x = atof(argv[1]);
    target.y = atof(argv[2]);

    if(argc > 3)
    {
        target.theta = atof(argv[3]);
        target.is_position = false;
    }
    else
    {
        target.is_position = true;
    }

    system::ModuleCommunicator communicator;
    communicator.sendMessage(target);
    
    StatusListener listener;
    communicator.subscribeTo<vulcan_lcm::metric_pose_target_status_t>(&listener);
    
    while(!listener.isDone())
    {
        communicator.processIncoming();
        usleep(10000);
    }

    return 0;
}
