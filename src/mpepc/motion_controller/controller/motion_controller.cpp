/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_controller.cpp
* \author   Collin Johnson and Jong Jin Park
* 
* Definition of create_motion_controller factory.
*/

#include "mpepc/motion_controller/controller/motion_controller.h"
#include "mpepc/motion_controller/controller/motion_target_following_controller.h"
#include "mpepc/motion_controller/controller/path_following_controller.h"
#include <cassert>

namespace vulcan
{
    
namespace mpepc
{

std::unique_ptr<MotionController> create_motion_controller(const std::string& type, const motion_controller_params_t& params)
{
    if(type == MOTION_TARGET_FOLLOWING_CONTROLLER_TYPE)
    {
        return std::unique_ptr<MotionController>(new MotionTargetFollowingController(params.targetFollowerParams));
    }
    else if(type == PATH_FOLLOWING_CONTROLLER_TYPE)
    {
        return std::unique_ptr<MotionController>(new PathFollowingController(params.followerParams));
    }
    
    std::cerr<<"ERROR:create_motion_controller: Unknown MotionController type:"<<type<<std::endl;
    assert(false);
    
    return std::unique_ptr<MotionController>();
}

} // mpepc
} // vulcan
