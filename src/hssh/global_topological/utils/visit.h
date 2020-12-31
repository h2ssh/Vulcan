/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visit.h
* \author   Collin Johnson
*
* Declaration of TopologicalVisit.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_UTILS_VISIT_H
#define HSSH_GLOBAL_TOPOLOGICAL_UTILS_VISIT_H

#include "hssh/local_metric/pose.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/events/turn_around.h"
#include "hssh/utils/id.h"
#include <boost/optional.hpp>
#include <cereal/access.hpp>
#include <memory>

namespace vulcan
{
namespace hssh
{

class LocalTopoMap;

/**
* TopologicalVisit defines all topological events that occur while the robot is at a particular LocalArea. A visit
* includes the following information:
*
*   - the id of the local area being visited
*   - the entry transition (always available)
*   - the exit transition (once it occurs)
*   - any path events that happen (depends on the type of area visited)
*   - the entry pose (in the LPM)
*   - the exit pose (once it occurs in the LPM)
*
* The entry and exit pose are useful information to help determine if some sort of place detection error has occurred.
* If the robot has driven further or shorter than expected, then the entry and exit pose can be used to help determine
* where the robot actually is.
*/
class TopologicalVisit
{
public:

    using Ptr = std::shared_ptr<TopologicalVisit>;
    using PathEventIter = std::vector<TurnAroundEvent>::const_iterator;

    /**
    * Constructor for TopologicalVisit.
    *
    * \param    depth           Monotonically increasing depth value -- indicates which event number this visit
    *   corresponds to. The depth will be the depth in the tree of maps for hypotheses including this visit.
    * \param    entryEvent      Event that brought carried the robot into the area being visited
    */
    TopologicalVisit(int depth, const AreaTransitionEvent& entryEvent);

    /**
    * depth retrieves the unique id associated with the visit.
    */
    int depth(void) const { return depth_; }

    /**
    * areaId retrieves the local area id of the area being visited.
    */
    Id areaId(void) const { return areaId_; }

    /**
    * localArea retrieves a pointer to the local area that was visited.
    */
    const LocalArea* localArea(void) const;

    /**
    * entryEvent retrieves the entry event for the area. As entry to an area must always occur for the robot to be
    * located at that area, the entry event always exists.
    */
    AreaTransitionEvent entryEvent(void) const { return entryEvent_; }

    /**
    * exitEvent retrieves the event that occurred when the robot exited the area. This event won't exist for the
    * area the robot is currently located in.
    */
    boost::optional<AreaTransitionEvent> exitEvent(void) const;

    /**
    * beginPathEvents retrieves the starting iterator over the path events that have occurred while on a path.
    */
    PathEventIter beginPathEvents(void) const { return pathEvents_.begin(); }

    /**
    * endPathEvents is the end iterator for the path events.
    */
    PathEventIter endPathEvents(void) const { return pathEvents_.end(); }

    /**
    * entryPose retrieves the entry pose of the robot when it first entered the area.
    */
    LocalPose entryPose(void) const { return entryPose_; }

    /**
    * lastPose retrieves the last pose for the robot when it was in the area. If the robot is still in the area, it
    * is the most recent pose associated received while in the area. If the robot has left the area, it was the final
    * pose of the robot while it was still in the area.
    */
    LocalPose lastPose(void) const { return lastPose_; }

    // Modifiers for updating a visit while it is ongoing

    /**
    * setExitEvent sets the event that occurred when the area was exited. The exit event must occur after the entry
    * event, so the sequence id must come after the entry id.
    *
    * \pre  exitEvent.sequenceId() > entryEvent.sequenceId()
    * \param    exitEvent       Event that exited the current area
    * \return   True if the event was valid for this visit, i.e. the types of the areas matched.
    */
    bool setExitEvent(const AreaTransitionEvent& exitEvent);

    /**
    * addPathEvent adds a new path event to the visit. The path event must occur after the entry event, so the
    * sequence id must be higher than the entry event.
    *
    * \pre  pathEvent area id == areaId()
    * \pre  pathEvent.seque
    */
    void addPathEvent(const TurnAroundEvent& pathEvent);

    /**
    * updatePose updates the current pose of the robot within the area.
    */
    void updatePose(const LocalPose& pose);

private:

    Id depth_;      ///< Depth of this event in the sequence sequence, i.e. what number event it is
    Id areaId_;     ///< Id of the area being visited

    AreaTransitionEvent entryEvent_;            ///< Event that started the visit
    AreaTransitionEvent exitEvent_;             ///< Event that ended the visit
    std::vector<TurnAroundEvent> pathEvents_;   ///< Any non-transition events that occurred during the visit
    LocalArea::Ptr localArea_;                  ///< Most recent local area representation

    LocalPose entryPose_;
    LocalPose lastPose_;

    // Serialization support
    TopologicalVisit(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( depth_,
            areaId_,
            entryEvent_,
            exitEvent_,
            pathEvents_,
            localArea_,
            entryPose_,
            lastPose_
        );
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_UTILS_VISIT_H
