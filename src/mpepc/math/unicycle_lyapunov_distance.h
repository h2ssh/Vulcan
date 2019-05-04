/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unicycle_lyapunov_distance.h
* \author   Jong Jin Park
* 
* Declaration of UnicycleLyapunovDistance, implementation of control-Lyapunov
* distance functions for unicycle-type vehicles.
*/

#ifndef UNICYCLE_LYAPUNOV_DISTANCE_H
#define UNICYCLE_LYAPUNOV_DISTANCE_H

#include <mpepc/math/unicycle_nonholonomic_distance.h>
#include <mpepc/math/unicycle_chart.h>

namespace vulcan
{
namespace mpepc
{

struct unicycle_lyapunov_distance_params_t
{
    double kPhi;
    double kDelta;
    double smallRadius;
    stabilizing_vector_field_type_t vectorFieldType;
};

// UnicycleLyapunovDistance encodes distance-to-target-pose.
class UnicycleLyapunovDistance
{
public:
    
    /**
    * Constructor for UnicycleLyapunovDistance
    * 
    * \param    targetPose        target pose
    * \param    params            parameters
    */
    UnicycleLyapunovDistance(const pose_t& targetPose, const unicycle_lyapunov_distance_params_t& params);
    
    // returns the target pose
    pose_t getTargetPose(void) const { return chart_.getTargetPose(); };
    
    /**
    * stabilizingDeltaStar returns stabilizing unicycle heaidng in egocentric polar coordinates.
    * 
    * \param    point           a point on a plane
    * \return   stabilizing heading in egocentric polar coordinates
    */
    double stabilizingDeltaStar(const Point<float> point) const;
    
    /**
    * stabilizingHeading returns stabilizing unicycle heaidng in Cartesian coordinates.
    * 
    * \param    point           a point on a plane
    * \return   stabilizing heading in Cartesian coordinates.
    */
    double stabilizingHeading(const Point<float> point) const;
    
    /**
    * distanceFromPoint computes distance-to-go to the taget pose from a point, assuming forward motion
    * 
    * \param    point           a point on a plane
    * \return   orientation-weighted distance to the target pose
    */
    double distanceFromPoint(const Point<float>& point) const;
    
    /**
    * distanceFromPose computes distance-to-go to the taget pose from a pose, assuming forward motion
    * 
    * \param    pose            a pose on a plane
    * \return   control-lyapunov distance-to-go to the target pose
    */
    double distanceFromPose(const pose_t& pose) const;
    
private:
    
    friend class UnicycleLyapunovSteering;
    
    UnicycleChart chart_; // mapping around the target pose
    
    unicycle_lyapunov_distance_params_t params_;
};

} // mpepc
} // vulcan

#endif // UNICYCLE_LYAPUNOV_DISTANCE_H
