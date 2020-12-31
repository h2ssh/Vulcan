/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stationary.h
* \author   Collin Johnson
* 
* Declaration of StationaryMotion.
*/

#ifndef TRACKER_MOTIONS_STATIONARY_H
#define TRACKER_MOTIONS_STATIONARY_H

#include "tracker/object_motion.h"

namespace vulcan
{
namespace tracker 
{

/**
* StationaryMotion represents an object that is sitting still. Stationary motion is the least sophisticated type of
* motion as it doesn't estimate the velocity and determines the position based solely on the detected positin of the
* laser object.
* 
* StationaryMotion has:
*   
*   - no velocity
*   - a position equal to the most recent laser detection
*/
class StationaryMotion : public ObjectMotion
{
public:

    /**
    * Default constructor for StationaryMotion.
    */
    StationaryMotion(void);
    
    /**
    * Constructor for StationaryMotion.
    * 
    * \param    position            Detected position of the object
    * \param    maxStationaryDist   Maximum distance an object can move from its initial position to be considered
    *                               stationary (optional, default = 0.25m)
    */
    StationaryMotion(Position position, float maxStationaryDist = 0.25f);
    
    // ObjectMotion interface
    void accept(ObjectMotionVisitor& visitor) const override;
    std::unique_ptr<ObjectMotion> clone(void) const override;
    
private:
    
    Position initialPosition_;
    float    kMaxStationaryDist_;
    
    // ObjectMotion interface
    ObjectMotionStatus               modelStatus(void) const override;
    object_motion_state_t updateMotionEstimate(const LaserObject& object) override;
    Position estimateFuturePosition(int deltaTimeMs) const override;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<ObjectMotion>(this),
            initialPosition_);
    }
};

}
}

#endif // TRACKER_MOTIONS_STATIONARY_H
