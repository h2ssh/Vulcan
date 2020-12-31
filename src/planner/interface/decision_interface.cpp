/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_interface.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionInterface.
 */

#include "planner/interface/decision_interface.h"
#include "hssh/local_topological/areas/decision_point.h"
#include "hssh/local_topological/areas/destination.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/location.h"
#include "utils/stub.h"

namespace vulcan
{
namespace planner
{

void create_action_for_transition_affordance(const hssh::TransitionAffordance& transition,
                                             hssh::AreaType areaType,
                                             DecisionDirection direction,
                                             bool isAbsolute,
                                             std::vector<Decision>& actions);
void create_actions_for_transition_affordances(const std::vector<hssh::TransitionAffordance>& transitions,
                                               hssh::AreaType areaType,
                                               std::vector<Decision>& actions);


int DecisionInterface::determineActions(const hssh::LocalTopoMap& map,
                                        const hssh::LocalLocation& location,
                                        const pose_t& pose)
{
    // If the location is unchanged, then there are no new actions to consider.
    if (!decisions_.empty() && (location_ == location)) {
        return 0;
    }

    // Otherwise proceed with the normal task of finding actions
    decisions_.clear();
    map_ = &map;
    location_ = location;
    pose_ = pose;

    auto currentArea = map_->areaWithId(location_.areaId());
    if (currentArea) {
        currentArea->accept(*this);
    }

    return decisions_.size();
}


void DecisionInterface::visitDecisionPoint(const hssh::LocalDecisionPoint& decision)
{
    // The decision point actions carry the robot to the associated path segments via forward/left/
    // right. The destinations are reached absolutely, as they don't easily map to a particular
    // relative direction. If the star has more than two paths, absolute directions will be used for the path
    // segments as well

    // TODO: Always create a backward action for the entry gateway

    PRINT_PRETTY_STUB()

    //     auto star = decision.star();
    //
    //     // If more than two paths, create an absolute action for each path segment
    //     if(star.getNumPaths() > 2)
    //     {
    //         create_actions_for_transition_affordances(decision.adjacent(), hssh::AreaType::path_segment, decisions_);
    //     }
    //     else if(!location_.isInitialArea())
    //     {
    //         auto entry = decision.findGatewayFragment(location_.entryGateway());
    //         assert(entry);
    //
    //         create_action_for_transition_affordance(decision.affordanceForFragment(entry.get()).get(),
    //                                                 hssh::AreaType::path_segment,
    //                                                 DecisionDirection::backward,
    //                                                 true,
    //                                                 decisions_);
    //
    //         auto otherFrag = star.otherFragmentOnPath(entry.get());
    //         if(otherFrag.navigable)
    //         {
    //             create_action_for_transition_affordance(decision.affordanceForFragment(otherFrag).get(),
    //                                                     hssh::AreaType::path_segment,
    //                                                     DecisionDirection::forward,
    //                                                     true,
    //                                                     decisions_);
    //         }
    //
    //         // The fragments are ordered from the outward angles. The right/left distinction when entering is the
    //         inbound
    //         // direction which is the opposite of outbound. Thus, the fragments to the robot's left are those to the
    //         right
    //         // in the star
    //         auto leftFrags = star.fragmentsRightOf(entry.get());
    //         for(auto& f : leftFrags)
    //         {
    //             if(f.navigable)
    //             {
    //                 create_action_for_transition_affordance(decision.affordanceForFragment(f).get(),
    //                                                         hssh::AreaType::path_segment,
    //                                                         DecisionDirection::left,
    //                                                         true,
    //                                                         decisions_);
    //             }
    //         }
    //
    //         // Similar to above, the right fragments are to the left in the star reference frame
    //         auto rightFrags = star.fragmentsLeftOf(entry.get());
    //         for(auto& f : rightFrags)
    //         {
    //             if(f.navigable)
    //             {
    //                 create_action_for_transition_affordance(decision.affordanceForFragment(f).get(),
    //                                                         hssh::AreaType::path_segment,
    //                                                         DecisionDirection::right,
    //                                                         true,
    //                                                         decisions_);
    //             }
    //         }
    //     }
    //     // Initial area started in a decision point
    //     else
    //     {
    //         std::cout << "STUB! DecisionInterface: Cannot handle starting at a decision point. Just creating absolute
    //         "
    //             << "actions for now.\n";
    //         // TODO: Handle the initial case of starting at a decision point
    //
    //             create_actions_for_transition_affordances(decision.adjacent(),
    //                                                       hssh::AreaType::path_segment,
    //                                                       decisions_);
    //     }
    //
    //     create_actions_for_transition_affordances(decision.adjacent(), hssh::AreaType::destination, decisions_);
}


void DecisionInterface::visitDestination(const hssh::LocalDestination& destination)
{
    // For a destination, there is an absolute action associated with each exit. The orientation points away from
    // the destination.
    create_actions_for_transition_affordances(destination.adjacent(), hssh::AreaType::destination, decisions_);
}


void DecisionInterface::visitPathSegment(const hssh::LocalPathSegment& path)
{
    // For a path segment, there are two relative actions associated with going to the different endpoints of the
    // path segment. Each destination will be an absolute action.

    auto plusDirection = DecisionDirection::backward;
    auto minusDirection = DecisionDirection::forward;

    // If the entered from the minus direction, then it is heading opposite of the normal plus entry
    if (path.minusTransition().gateway() == location_.entryGateway()) {
        plusDirection = DecisionDirection::forward;
        minusDirection = DecisionDirection::backward;
    }

    // The robot moves toward the minus direction. The plus direction is where it came from.
    auto plusDir = path.moveAlongPlus();
    decisions_.emplace_back(plusDirection,
                            hssh::AreaType::decision_point,
                            plusDir.target().toPoint(),
                            plusDir.target().theta,
                            false);

    auto minusDir = path.moveAlongMinus();
    decisions_.emplace_back(minusDirection,
                            hssh::AreaType::decision_point,
                            minusDir.target().toPoint(),
                            minusDir.target().theta,
                            false);

    create_actions_for_transition_affordances(path.leftDestinations(), hssh::AreaType::destination, decisions_);
    create_actions_for_transition_affordances(path.rightDestinations(), hssh::AreaType::destination, decisions_);
}


void create_action_for_transition_affordance(const hssh::TransitionAffordance& transition,
                                             hssh::AreaType areaType,
                                             DecisionDirection direction,
                                             bool isAbsolute,
                                             std::vector<Decision>& actions)
{
    actions.emplace_back(direction, areaType, transition.target().toPoint(), transition.target().theta, isAbsolute);
}


void create_actions_for_transition_affordances(const std::vector<hssh::TransitionAffordance>& transitions,
                                               hssh::AreaType areaType,
                                               std::vector<Decision>& actions)
{
    for (auto& affordance : transitions) {
        create_action_for_transition_affordance(affordance, areaType, DecisionDirection::absolute, true, actions);
    }
}

}   // namespace planner
}   // namespace vulcan
