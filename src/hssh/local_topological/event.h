/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     event.h
 * \author   Collin Johnson
 *
 * Declaration of the LocalAreaEvent interface.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENT_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENT_H

#include "hssh/local_metric/pose.h"
#include "hssh/utils/id.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{

class LocalAreaEvent;
class LocalAreaEventVisitor;

// Types used for containing various events and event sequences
using LocalAreaEventPtr = std::shared_ptr<LocalAreaEvent>;
using LocalAreaEventVec = std::vector<LocalAreaEventPtr>;

/**
 * LocalAreaEvent is the base class for every event that happens in the system. The following is true of every
 * event:
 *
 *   1) Occurs at an area                    (area)
 *   2) Has a monotonically increasing id    (sequenceId)
 *   3) Supports the LocalAreaEventVisitor   (accept)
 *
 * The LocalAreaEvent hierarchy is mostly just a discriminated union. The events themselves are merely informative, but
 * don't perform operations, hence just some observer methods.
 */
class LocalAreaEvent
{
public:
    using IdIter = std::vector<Id>::const_iterator;

    virtual ~LocalAreaEvent(void) { }

    /**
     * timestamp retrieves the time at which the event occurred.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * sequenceId retrieves the id for the event.
     */
    int32_t sequenceId(void) const { return id_; }

    /**
     * pose retrieves the approximate pose of the robot when the event occurred.
     */
    LocalPose pose(void) const { return pose_; }

    /**
     * beginAreaIds is the start iterator for all area ids involved with this area.
     */
    virtual IdIter beginAreaIds(void) const = 0;

    /**
     * endAreaIds is the end iterator for all area ids involved with this area.
     */
    virtual IdIter endAreaIds(void) const = 0;

    /**
     * description produces a text description of the event that occurred.
     */
    virtual std::string description(void) const = 0;

    /**
     * clone produces a copy of the LocalAreaEvent.
     */
    virtual LocalAreaEventPtr clone(void) const = 0;

    /**
     * accept accepts a LocalAreaEventVisitor and calls the appropriate visitXXXX method.
     */
    virtual void accept(LocalAreaEventVisitor& visitor) const = 0;

protected:
    /**
     * Constructor for LocalAreaEvent.
     *
     * Optionally sets the id for the event. A new id should be created whenever an actually new event is being created.
     * The default constructor for an event, used solely for serialization purposes, should not be setting the id, as it
     * will be loaded from a file.
     *
     * \param    timestamp           Time at which the event occurred
     * \param    pose                Pose when the event occurred (approximately)
     * \param    needNewId           Flag indicating if a new event id should be generated
     */
    LocalAreaEvent(int64_t timestamp, const LocalPose& pose, bool needNewId)
    : timestamp_(timestamp)
    , pose_(pose)
    , id_(needNewId ? nextEventId() : -1)
    {
    }

    /**
     * Default constructor for LocalAreaEvent.
     */
    LocalAreaEvent(void) = default;

private:
    int64_t timestamp_ = -1;
    LocalPose pose_;
    int32_t id_ = -1;

    int32_t nextEventId(void)
    {
        static int32_t eventId = 0;
        return eventId++;
    }

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp_, pose_, id_);
    }
};

}   // namespace hssh
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::LocalAreaEventVec, ("HSSH_LOCAL_TOPO_EVENTS"))

#endif   // HSSH_LOCAL_TOPOLOGICAL_EVENT_H
