/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     turn_around.h
* \author   Collin Johnson
*
* Declaration of TurnAroundEvent.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENTS_TURN_AROUND_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENTS_TURN_AROUND_H

#include <hssh/local_topological/event.h>
#include <hssh/local_topological/areas/path_segment.h>
#include <hssh/types.h>
#include <cereal/types/base_class.hpp>
#include <memory>

namespace vulcan
{
namespace hssh
{

class LocalPathSegment;

/**
* TurnedAroundEvent is a path event that occurs whenever the robot turns around while navigating along a path segment
* in the environment.
*/
class TurnAroundEvent : public LocalAreaEvent
{
public:

    /**
    * Default constructor for TurnAroundEvent.
    */
    TurnAroundEvent(void) = default;

    /**
    * Constructor for TurnAroundEvent.
    *
    * \param    timestamp           Time at which the event occurred
    * \param    path                Path segment along which the event occurred
    * \param    current             Current direction of the robot
    * \param    previous            Previous direction of the robot
    * \param    turnAroundPose      Pose at which the robot turned around in the path segment's reference frame
    */
    TurnAroundEvent(int64_t                           timestamp,
                    std::shared_ptr<LocalPathSegment> path,
                    TopoDirection                     current,
                    TopoDirection                     previous,
                    const LocalPose&                  turnAroundPose);

    /**
    * areaId retrieves the id of the area in which this event occurred.
    */
    Id areaId(void) const { return pathSegment_->id(); }

    /**
    * currentDirection retrieves the direction the robot is now moving down the path.
    */
    TopoDirection currentDirection(void) const { return current_; }

    /**
    * previousDirection retrieves the direction the robot was previously moving down the path.
    */
    TopoDirection previousDirection(void) const { return previous_; }

    /**
    * path retrieves the path on which this event occurred.
    */
    std::shared_ptr<LocalPathSegment> path(void) const { return pathSegment_; }

    /**
    * turnAroundPose retrieves the pose at which the robot started turning around. The pose is in the
    * path segment's reference frame.
    */
    LocalPose turnAroundPose(void) const { return pose(); }

    // LocalAreaEvent interface
    IdIter beginAreaIds(void) const override { return areaIds_.begin(); }
    IdIter endAreaIds(void) const override { return areaIds_.end(); }
    std::string       description(void) const override;
    void              accept     (LocalAreaEventVisitor& visitor) const override;
    LocalAreaEventPtr clone      (void) const override;

private:

    std::vector<Id> areaIds_;
    std::shared_ptr<LocalPathSegment> pathSegment_;
    TopoDirection current_;
    TopoDirection previous_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (cereal::base_class<LocalAreaEvent>(this));
        ar (areaIds_,
            pathSegment_,
            current_,
            previous_);
    }
};

}
}

// Polymorphic serialization support
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
CEREAL_REGISTER_TYPE(vulcan::hssh::TurnAroundEvent)

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENTS_TURN_AROUND_H
