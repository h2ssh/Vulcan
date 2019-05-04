/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose.h
* \author   Collin Johnson
* 
* Definition of GlobalPose.
*/

#ifndef HSSH_GLOBAL_METRIC_POSE_H
#define HSSH_GLOBAL_METRIC_POSE_H

#include <core/pose_distribution.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{
    
/**
* GlobalPose represents the pose of the robot within a GlobalMetricMap.
*/
class GlobalPose
{
public:
    
    /**
    * Default constructor for GlobalPose.
    */
    GlobalPose(void);
    
    /**
    * Constructor for GlobalPose.
    */
    GlobalPose(const pose_t& pose, int32_t mapId);
    
    /**
    * Constructor for GlobalPose.
    */
    GlobalPose(const pose_distribution_t& pose, int32_t mapId);
    
    // Accessors
    int32_t                    mapId(void)            const { return mapId_;         }
    pose_t              pose(void)             const { return pose_.toPose(); }
    pose_distribution_t poseDistribution(void) const { return pose_;          }
    
private:
    
    int32_t                    mapId_;
    pose_distribution_t pose_;
    
    // Serialization support
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (mapId_,
            pose_);
    }
};
    
}
}

DEFINE_SYSTEM_MESSAGE(hssh::GlobalPose, ("HSSH_GLOBAL_POSE"))

#endif // HSSH_GLOBAL_METRIC_POSE_H
