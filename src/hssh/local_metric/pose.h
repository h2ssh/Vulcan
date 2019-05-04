/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_pose.h
* \author   Collin Johnson
* 
* Definition of LocalPose.
*/

#ifndef HSSH_LOCAL_METRIC_POSE_H
#define HSSH_LOCAL_METRIC_POSE_H

#include <core/pose_distribution.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{
    
/**
* LocalPose represents the robot's pose within an LPM. The pose includes the covariance distribution for the
* uncertainty of the pose in the map. The index of the reference frame in which the pose was found is also included to
* allow conversion of the pose to another reference frame if needed.
*/
class LocalPose
{
public:
    
    /**
    * Default constructor for LocalPose.
    */
    LocalPose(void)
    : frameIndex_(0)
    {
    }
    
    /**
    * Constructor for LocaPose.
    * 
    * \param    pose            Pose of the robot
    * \param    frameIndex      Reference frame index for map in which pose was found
    */
    LocalPose(const pose_distribution_t& pose,
              int32_t                           frameIndex)
    : frameIndex_(frameIndex)
    , pose_(pose)
    {
    }
    
    /**
    * timestamp retrieves the time the pose was measured.
    */
    int64_t timestamp(void) const { return pose_.timestamp; }
    
    /**
    * referenceFrameIndex retrieves the index of the reference frame of the map in which this
    * pose was generated.
    */
    int32_t referenceFrameIndex(void) const { return frameIndex_; }
    
    /**
    * pose retrieves the pose of the robot.
    */
    pose_t pose(void) const { return pose_.toPose(); }
    
    /**
    * poseDistribution retrieves the full error distribution of the pose.
    */
    pose_distribution_t poseDistribution(void) const { return pose_; }
    
private:
    
    int32_t                    frameIndex_;
    pose_distribution_t pose_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (frameIndex_,
            pose_);
    }
};
    
} // namespace hssh
} // namespace vulcan


DEFINE_SYSTEM_MESSAGE(hssh::LocalPose, ("HSSH_LOCAL_POSE"))


#endif // HSSH_LOCAL_METRIC_POSE_H
