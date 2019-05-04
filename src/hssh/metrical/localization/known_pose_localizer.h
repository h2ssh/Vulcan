/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     known_pose_localizer.h
* \author   Collin Johnson
* 
* Declaration of KnownPoseLocalizer.
*/

#ifndef HSSH_METRICAL_LOCALIZATION_KNOWN_POSE_LOCALIZER_H
#define HSSH_METRICAL_LOCALIZATION_KNOWN_POSE_LOCALIZER_H

#include <hssh/metrical/localization/localizer.h>
#include <core/pose.h>

namespace vulcan
{
namespace hssh
{
    
const std::string kKnownPoseType("known-pose");


/**
* KnownPoseLocalizer is the simplest possible localizer. It requires odometry data and if odometry is available, it
* simply assumes that the odometry is correct. Thus, it builds maps using dead reckoning, or if the poses come from some
* oracle, a correct map that by-passes the normal particle filter calculations.
* 
* The returned pose will always be relative to the pose provided in resetPoseEstimate. Thus, the reset pose puts the
* robot in the map at (0, 0, 0).
*/
class KnownPoseLocalizer : public Localizer
{
public:
    
    // Localizer interface
    pose_distribution_t initializeLocalization(const metric_slam_data_t& data) override;
    void resetPoseEstimate(const pose_t& pose) override;
    pose_distribution_t updatePoseEstimate(const metric_slam_data_t& data, 
                                                  const OccupancyGrid& map,
                                                  particle_filter_debug_info_t* debug) override;
                                                  
private:
    
    pose_t initialPose_;
};

}
}

#endif // HSSH_METRICAL_LOCALIZATION_KNOWN_POSE_LOCALIZER_H
