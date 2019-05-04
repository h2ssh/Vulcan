/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_transition_detector.cpp
* \author   Collin Johnson
*
* Implementation of AreaTransitionDetector.
*/

#include <hssh/local_topological/event_detection/area_transition_detector.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/area.h>
#include <hssh/local_topological/events/area_transition.h>
#include <utils/algorithm_ext.h>
#include <boost/optional/optional_io.hpp>
#include <iostream>

#define DEBUG_TRANSITIONS

namespace vulcan
{
namespace hssh
{

// If more than one possibility, go with the area that is the same as the current area
LocalAreaPtr area_containing_pose(pose_t pose,
                                  const LocalAreaPtr& currentArea,
                                  const math::Rectangle<float>& currentBoundary,
                                  const LocalTopoMap& map);
LocalAreaPtr extent_containing_pose(pose_t pose, const LocalAreaVec& areas);
bool is_uncertain_area(const LocalAreaPtr& area);
gateway_crossing_t find_crossing_gateway(const LocalArea&     previous,
                                         const LocalArea&     current,
                                         const utils::PoseTrace::ConstIter& beginPoses,
                                         const utils::PoseTrace::ConstIter& endPoses,
                                         std::ostream& log);


AreaTransitionDetector::AreaTransitionDetector(const area_transition_detector_params_t& params)
: isInitialArea_(true)
, poses_(0.01f, 100000) // store up to 1000m of driving
, params_(params)
, transitionLog_("transitions.log")
{
}


LocalAreaEventVec AreaTransitionDetector::detectEvents(const LocalTopoMap& map)
{
    static int updateCount = 0;
    ++updateCount;

    /*
    * A transition occurs when the gateway boundary between two areas is crossed.
    */
    LocalAreaEventVec events;

    // Nothing to process if there are no poses available.
    if(poses_.empty())
    {
        return events;
    }

    // For the initial area, there isn't any prior area to disambiguate with
    // Search for a valid pose in the first area. If a pose has no unambiguous area, then it should be erased.
    LocalAreaPtr currentArea;

    while(!poses_.empty() && !currentArea)
    {
        currentArea = area_containing_pose(poses_.front(),
                                           LocalAreaPtr(),
                                           previousBoundary_,
                                           map);
        // Erase poses that don't belong to an area
        if(!currentArea)
        {
            transitionLog_ << "Popped null pose: " << poses_.front() << '\n';
            poses_.pop_front();
        }
    }

    int64_t latestEventTime = 0;

    if(!currentArea)
    {
        std::cout << "Not currently in an area.\n";
        return events;
    }

    previousBoundary_ = currentArea->boundary();

    // The first certain area in which the robot is located always generates an event
    if(isInitialArea_ && !is_uncertain_area(currentArea))
    {
        pose_t currentPose = poses_.front();
        events.emplace_back(new AreaTransitionEvent(currentPose.timestamp,
                                                    localPoses_.nearest_value(currentPose.timestamp),
                                                    currentArea));
        isInitialArea_ = false;
    }

    int entryIndex = 0;
    double distSinceLastEvent = 0.0;
    double turnSinceLastEvent = 0.0;

    // meters to move before automatically going to next area
    // without an event being detected -- caused by a small pose jump or mapping error, so doesn't need to be large
    const double kMaxFailureDistBeforeSkip = 0.5;

    // Distance traveled during a failure condition -- a gateway transition not being detected.
    double failureDist = 0.0;
    std::size_t failStartIndex = poses_.size();
    Gateway lastGateway = latestGateway_;
    LocalAreaEventPtr deferredEvent;    // currently deferred event when a destination was entered

    for(std::size_t n = 0; n < poses_.size(); ++n)
    {
        auto& pose = poses_[n];

        // Only consider poses up until the time of the current map. Since there is processing delay, don't try
        // and create events based on poses in the LPM not considered in the current map labeling.
        if((pose.timestamp > map.timestamp()) && !params_.ignoreStaleMapTimestamps)
        {
            break;
        }

        // Maintain how long the robot has traveled since the last event. In the case of crossing the same gateway,
        // then some minimum amount of travel must have occurred before the event will fire. This hysteresis is
        // necessary to keep small changes in gateway positions from causing spurious area transitions
        if((pose.timestamp > latestEventTime) && (n > 0))
        {
            distSinceLastEvent += distance_between_points(poses_[n-1].toPoint(), poses_[n].toPoint());
            turnSinceLastEvent += angle_diff_abs(poses_[n].theta, poses_[n-1].theta);
        }

        LocalAreaPtr poseArea = area_containing_pose(pose, currentArea, previousBoundary_, map);

        // If we aren't in an area or the area hasn't changed, then no event has occurred for this pose
        if(!poseArea || (poseArea == currentArea) || (poseArea->type() == AreaType::destination))
        {
            continue;
        }

        transitionLog_ << "Update " << updateCount << ": Area changed from " << currentArea->boundary() << " to "
            << poseArea->boundary() << " at " << poses_[n] << " Time (ms): " << (poses_[n].timestamp / 1000)
            << " Dist since last event:" << distSinceLastEvent << '\n';

        // See if a gateway has been crossed
        auto crossing = find_crossing_gateway(*currentArea,
                                              *poseArea,
                                              poses_.begin() + entryIndex,
                                              poses_.begin() + n + 1,
                                              transitionLog_); // want to include this pose in the search

        // If so, consider creating an event
        if(crossing.gateway)
        {
            bool isUncertainArea = is_uncertain_area(poseArea);

            transitionLog_ << "Update " << updateCount << ": Crossed gateway: " << crossing.gateway
                << " Similar to previous? " << lastGateway.isSimilarTo(crossing.gateway.get())
                << " Time(ms): " << (crossing.timestamp / 1000)
                << " Dist since last event: " << distSinceLastEvent << "m Turn: " << turnSinceLastEvent
                << " Uncertain? " << isUncertainArea << '\n';

            // If the gateway is similar to the last gateway crossed, then a twitch in the map might have caused
            // this new event to be detected. Only fire an event if the robot has moved enough since last crossing
            // this same gateway. If the gateways aren't similar, then fire away!
            if(false     //!lastGateway.isSimilarTo(crossing.gateway.get())
                || (distSinceLastEvent > params_.minDistBetweenSameGatewayTransitions)
                || (turnSinceLastEvent > M_PI_2))
            {
                auto event = std::make_shared<AreaTransitionEvent>(crossing.timestamp,
                                                                   localPoses_.nearest_value(crossing.timestamp),
                                                                   currentArea,
                                                                   poseArea,
                                                                   crossing.gateway.get());
                // If a second event occurs while an event has been deferred, then the deferred event
                // should become a real event, as the area was both entered and exited and thus is stable enough
                // to add to the map
                if(isUncertainArea && !deferredEvent)
                {
                    deferredEvent = event;
                    transitionLog_ << "Deferring event: " << event->description() << '\n';
                }
                else
                {
                    // If a deferred event was hanging around, then it needs to get added first as it must have
                    // come before this latest event
                    if(deferredEvent)
                    {
                        transitionLog_ << "Finalizing deferred event: " << deferredEvent->description() << '\n';
                        events.push_back(deferredEvent);
                        deferredEvent.reset();
                    }

                    transitionLog_ << "Adding event: " << event->description() << '\n';
                    events.push_back(event);

                    // When an event is officially created, then store the state so the next update will start
                    // with the correct value. Don't not do this for deferred events because they might get thrown
                    // away and we don't want to then miss detecting other events.
                    // Deferred events will get detected over and over until the ambiguity is resolved.
                    latestEventTime = crossing.timestamp;
                    latestGateway_ = crossing.gateway.get();
                    previousBoundary_ = currentArea->boundary();
                }
            }

            // Whenever a gateway is detected crossed, update the position of the gateway and the time at which
            // it was crossed to keep small pose twitches from slowly accumlating and resulting in an event firing.
            distSinceLastEvent = 0.0;   // reset the distance travelled by the robot, as a transition was detected
            turnSinceLastEvent = 0.0;   // reset the rotation of the robot, since a transition was detected
            failureDist = 0.0;  // a successful crossing was found, so no failure has occurred
            failStartIndex = poses_.size();
            lastGateway = crossing.gateway.get();

            // Only change the current area if a gateway was actually crossed
            entryIndex = n;
            currentArea = poseArea;
        }
        // NOTE: This error isn't critical because it only happens when the robot pose is in a cell right on the
        // boundary of two areas, which will be cleaned up on the next iteration of the transition detector.
        else
        {
            transitionLog_ << "WARNING:AreaTransitionDetector: Did not find an entry gateway when the areas changed!\n"
                << "\tOld area:" << currentArea->boundary() << "\n\tNew area:" << poseArea->boundary()
                << " Failure distance: " << failureDist << '\n';
            if(n > 0)
            {
                failureDist += distance_between_points(poses_[n-1].toPoint(), poses_[n].toPoint());
            }

            failStartIndex = std::min(failStartIndex, n);

            // Jump back to the previous failure point, but with the new index set
            if(failureDist > kMaxFailureDistBeforeSkip)
            {
                transitionLog_ << "INFO::AreaTransitionDetector: Reached the max failure distance. Jumping from idx "
                    << n << " to " << failStartIndex << " with new area " << currentArea->boundary() << '\n';

                // Only change the current area if a gateway was actually crossed
                currentArea = poseArea;
                previousBoundary_ = currentArea->boundary();
                n = failStartIndex;
                failureDist = 0.0;  // a successful crossing was found, so no failure has occurred
                failStartIndex = poses_.size();
            }
        }
    }

    // Erase all poses up to the point where the last event occurred to ensure no duplicate events happen
    poses_.clearBefore(latestEventTime + 1);
    localPoses_.erase_before(latestEventTime + 1);

    return events;
}


void AreaTransitionDetector::addPose(const LocalPose& pose)
{
    poses_.addPose(pose.pose());
    localPoses_.push_back(pose);
}


LocalAreaPtr area_containing_pose(pose_t pose,
                                  const LocalAreaPtr& currentArea,
                                  const math::Rectangle<float>& currentBoundary,
                                  const LocalTopoMap& map)
{
    auto containingAreas = map.allAreasContaining(pose.toPoint());

    // If no area, then a nullptr
    if(containingAreas.empty())
    {
        return nullptr;
    }
    // If one area, then return it
    if(containingAreas.size() == 1)
    {
        return containingAreas.front();
    }
    // Otherwise, try to stay in the previous area if possible.
    else
    {
        // If the extent has a unique area, that's where we are
        auto extentArea = extent_containing_pose(pose, containingAreas);

        if(extentArea)
        {
            return extentArea;
        }

        // Prefer remaining in the same area if possible
        if(utils::contains(containingAreas, currentArea))
        {
            return currentArea;
        }

        // Otherwise, select the area with the most overlap with the current boundary -- which is needed if the areas
        // in the local topo map change slightly for some reason.
        auto maxIt = std::max_element(containingAreas.begin(),
                                      containingAreas.end(),
                                      [&currentBoundary](const LocalAreaPtr& lhs, const LocalAreaPtr& rhs) {
            return lhs->boundary().intersection(currentBoundary).area() >
                rhs->boundary().intersection(currentBoundary).area();
        });

        assert(maxIt != containingAreas.end());
        return *maxIt;
    }
}


LocalAreaPtr extent_containing_pose(pose_t pose, const LocalAreaVec& areas)
{
    auto position = pose.toPoint();
    std::vector<bool> in;

    for(auto& a : areas)
    {
        in.push_back(a->extent().cellContains(position));
    }

    // If match more than one or none, right on the border, so say we aren't anywhere.
    if(std::count(in.begin(), in.end(), true) != 1)
    {
        return nullptr;
    }

    // Otherwise, find the area that we're in.
    return areas[std::distance(in.begin(), std::find(in.begin(), in.end(), true))];
}


bool is_uncertain_area(const LocalAreaPtr& area)
{
    // If there isn't an area, obviously it is uncertain!
    if(!area)
    {
        return true;
    }

    return (area->type() == AreaType::destination) && (area->extent().frontierRatio() > 0.01);
}


gateway_crossing_t find_crossing_gateway(const LocalArea&     previous,
                                         const LocalArea&     current,
                                         const utils::PoseTrace::ConstIter& beginPoses,
                                         const utils::PoseTrace::ConstIter& endPoses,
                                         std::ostream& log)
{
    gateway_crossing_t finalCrossing{boost::none, 0};

    // Search through all poses for the last transition between previous and current. Up to this point, there
    // may have been some twitches between the two that weren't deemed significant. Thus, the final pose going between
    // the two areas is the one that matters.
    for(auto nextIt = beginPoses + 1; nextIt != endPoses; ++nextIt)
    {
        pose_t prevPose = *(nextIt - 1);
        pose_t currentPose = *nextIt;

        // The area has changed -- find the transition gateway
        auto entryCrossing = current.findTransitionGateway(prevPose, currentPose);
        auto exitCrossing  = previous.findTransitionGateway(currentPose, prevPose);

        // If a gateway was crossed, then return that information
        if(entryCrossing.gateway && (entryCrossing.gateway == exitCrossing.gateway))
        {
            finalCrossing = entryCrossing;
            log << "Crossed " << *entryCrossing.gateway << " at " << currentPose << " Time(ms):"
                << (entryCrossing.timestamp / 1000) << '\n';
        }
    }

    return finalCrossing;
}

} // namespace hssh
} // namespace vulcan
