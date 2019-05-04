/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose.cpp
* \author   Collin Johnson
* 
* Definition of GlobalPose.
*/

#include <hssh/global_metric/pose.h>

namespace vulcan
{
namespace hssh
{
    
GlobalPose::GlobalPose(void)
: mapId_(-1)
{
}


GlobalPose::GlobalPose(const pose_t& pose, int32_t mapId)
: mapId_(mapId)
, pose_(pose, MultivariateGaussian(3))
{
}


GlobalPose::GlobalPose(const pose_distribution_t& pose, int32_t mapId)
: mapId_(mapId)
, pose_(pose)
{
}

}
}