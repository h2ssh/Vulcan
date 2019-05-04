/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_track_prediction_error.h
* \author   Collin Johnson
* 
* Declaration of ObjectTrackPredictionError.
*/

#ifndef TRACKER_EVALUATION_OBJECT_TRACK_PREDICTION_ERROR_H
#define TRACKER_EVALUATION_OBJECT_TRACK_PREDICTION_ERROR_H

#include <core/position.h>

namespace vulcan
{
namespace tracker
{
    
class ObjectTrack;

/**
* PredictionError 
*/
class PredictionError
{
public:
    
    /**
    * Constructor for PredictionError
    * 
    * \param    timestamp       Time at which the prediction occurs
    * \param    measured        Measured position for the time
    * \param    prediction      Predicted position for the time
    */
    PredictionError(int64_t timestamp, position_t measured, position_t prediction)
    : timestamp_(timestamp)
    , measured_(measured)
    , predicted_(prediction)
    {
    }
    
    /**
    * error calculates the error associated with the given prediction. The error is the Euclidean distance between the
    * measured and predicted positions of the robot at the given time.
    */
    double error(void) const { return distance_between_points(measured_, predicted_); }
    
    /**
    * measuredPosition retrieves the measured position for the given time.
    */
    position_t measuredPosition(void) const { return measured_; }
    
    /**
    * predictedPosition retrieves the predicted position for the given time.
    */
    position_t predictedPosition(void) const { return predicted_; }
    
    /**
    * timestamp retrieves the time at which this prediction error was measured.
    */
    int64_t timestamp(void) const { return timestamp_; }
    
private:
    
    int64_t timestamp_;
    position_t measured_;
    position_t predicted_;
};


/**
* ObjectTrackPredictionErrors contains errors between a measurement and prediction track 
*/
class ObjectTrackPredictionErrors
{
public:
    
    using size_type = std::vector<PredictionError>::size_type;
    using const_iterator = std::vector<PredictionError>::const_iterator;
    
    /**
    * Constructor for ObjectTrackPredictionErrors.
    * 
    * \param    measurement         Estimated track from the tracker incorporating all measurements
    * \param    prediction          Predicted track from a trajectory prediction function
    */
    ObjectTrackPredictionErrors(const ObjectTrack& measurement, const ObjectTrack& prediction);

    
    // Iterate through all the individual error calculations
    size_type size(void) const { return errors_.size(); }
    bool empty(void) const { return errors_.empty(); }
    PredictionError at(size_type n) { return errors_.at(n); }
    const_iterator begin(void) const { return errors_.begin(); }
    const_iterator end(void) const { return errors_.end(); }
    
private:
    
    std::vector<PredictionError> errors_;
    double minError_;
    double meanError_;
    double maxError_;
    double stdError_;
    double medianError_;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_EVALUATION_OBJECT_TRACK_PREDICTION_ERROR_H
