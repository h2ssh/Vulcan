/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_track.h
* \author   Collin Johnson
*
* Declaration of ObjectTrack and object_track_data_t.
*/

#ifndef TRACKER_EVALUATION_OBJECT_TRACK_H
#define TRACKER_EVALUATION_OBJECT_TRACK_H

#include "tracker/dynamic_object.h"
#include "core/multivariate_gaussian.h"

namespace vulcan
{
namespace tracker
{
    
/**
* object_track_timestep_t stores the data associated with each time step of an object's track. All relevant state
* estimates are stored, along with their full covariance.
*/
struct object_track_timestep_t
{
    int64_t timestamp;      ///< Time the data was gathered
    std::vector<MultivariateGaussian> stateEstimates_;        ///< All different estimates of state maintained
                                                                    ///< for the object
};

/**
* ObjectTrack stores the tracked state 
*/
class ObjectTrack
{
public:
    
    using size_type = std::vector<object_track_data_t>::size_type;
    using const_iterator = std::vector<object_track_data_t>::const_iterator;

    /**
    * Constructor for ObjectTrack.
    *
    * \param    object          Initial estimate of the object to be tracked
    * \pre object != nullptr
    */
    explicit ObjectTrack(DynamicObject::ConstPtr object);

    /**
    * id retrieves the id associated with this track.
    */
    ObjectId id(void) const { return currentEstimate_->id(); }

    /**
    * addEstimate
    */
    void addEstimate(DynamicObject::ConstPtr estimate);

    
    
    // Iteration support for the state
    size_type size(void) const { return data_.size(); }
    bool empty(void) const { return data_.empty(); }
    
    const_iterator begin(void) const { return data_.begin(); }
    const_iterator end(void) const { return data_.end(); }

private:

    DynamicObject::ConstPtr currentEstimate_;
    std::vector<object_track_data_t> data_;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_EVALUATION_OBJECT_TRACK_H
