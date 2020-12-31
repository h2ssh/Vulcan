/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visit_sequence.h
* \author   Collin Johnson
*
* Declaration of TopologicalVisitSequence.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_UTILS_VISIT_SEQUENCE_H
#define HSSH_GLOBAL_TOPOLOGICAL_UTILS_VISIT_SEQUENCE_H

#include "hssh/global_topological/utils/visit.h"
#include "hssh/local_topological/event_visitor.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

/**
* TopologicalVisitSequence contains the sequence of local areas visited by the robot during its current navigation. The
* visit sequence creates a new topological visit for each local area the robot enters.
*
* Each TopologicalVisit is assigned a monotonically increasing id number. This id number can be used to reference a
* particular visit or to manipulate the visit sequence, since the id is guaranteed to increase monotonically.
*
* TODO: Determine how best to handle errors in the visit sequence. Right now, text is output to the console, but
*       errors like mismatched entered/exited transitions need to be handled elsewhere. What exceptions to throw?
*/
class TopologicalVisitSequence : public LocalAreaEventVisitor
{
public:

    /**
    * Default constructor for TopologicalVisitSequence.
    */
    TopologicalVisitSequence(void) = default;

    /**
    * addEvent adds a new event to the sequence. The event will either generate a new visit or update the current
    * visit.
    *
    * \param    event           Topological event that occurred
    * \param    eventMap        Map in which the event was recorded
    */
    void addEvent(const LocalAreaEvent& event, const LocalTopoMap& eventMap);

    /**
    * addPose adds a new pose to the sequence. The pose is maintained for each visit to help robustness in the face
    * of a dynamic topological environment.
    *
    * \param    pose            Current robot pose
    */
    void addPose(const LocalPose& pose);

    /**
    * eraseVisitsBefore erases all visit that occurred before the provided visit depth. Events with depth in the range
    * [0, depth) will be removed.
    *
    * \param    depth           Visit depth to erase visits before
    * \return   The number of visits erased.
    */
    int eraseVisitsBefore(int depth);

    /**
    * numVisits retrieves the number of visits in the sequence.
    *
    * \return   Number of visits that have occurred.
    */
    int numVisits(void) const;

    /**
    * visitAt retrieves the visit associated with the given index.
    *
    * \param    index           Index of the visit to get
    * \return   Visit with the given index. nullptr if no such visit exists.
    */
    TopologicalVisit::Ptr visitAt(int index) const;

    // LocalAreaEventVisitor interface
    void visitAreaTransition(const AreaTransitionEvent& event) override;
    void visitTurnAround(const TurnAroundEvent& event) override;

private:

    int nextVisitDepth_ = 0;
    LocalPose lastPose_;
    std::vector<TopologicalVisit::Ptr> sequence_;
    TopologicalVisit* currentVisit_ = nullptr;
    const LocalTopoMap* currentMap_ = nullptr;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( nextVisitDepth_,
            sequence_,
            lastPose_
        );

        currentVisit_ = sequence_.back().get();
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_UTILS_VISIT_SEQUENCE_H
