/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map.cpp
* \author   Collin Johnson
*
* Definition of TopologicalMap.
*/

#include <hssh/global_topological/topological_map.h>
#include <hssh/global_topological/chi.h>
#include <hssh/global_topological/global_location.h>
#include <hssh/global_topological/utils/local_to_global.h>
#include <hssh/global_topological/utils/visit.h>
#include <utils/algorithm_ext.h>
#include <utils/stub.h>
#include <boost/range/adaptor/map.hpp>
#include <cassert>

#define DEBUG_CREATE_MAP

namespace vulcan
{
namespace hssh
{

bool exited_through_same_gateway(const TopologicalVisit& visit);


std::pair<TopologicalMap::Ptr, GlobalLocation> TopologicalMap::CreateTopologicalMap(
    const TopologicalVisit& initialVisit)
{
    Ptr map(new TopologicalMap(next_id()));
    GlobalLocation initialLocation;
    initialLocation.areaId = kFrontierId;
    initialLocation.areaType = initialVisit.localArea()->type();
    auto location = map->addConcreteArea(initialLocation, initialVisit);
    location.pathDirection = TopoDirection::plus;       // just assume moving in the plus direction

    return std::make_pair(map, location);
}


const TopologicalMap::PathMap& TopologicalMap::paths(void) const
{
    PRINT_PRETTY_STUB();
    return paths_;
}


bool TopologicalMap::hasArea(Id id) const
{
    return areas_.find(id) != areas_.end();
}


const GlobalPlace* TopologicalMap::getDestination(Id id) const
{
    return getPlace(id);
}


const GlobalPlace* TopologicalMap::getDecisionPoint(Id id) const
{
    return getPlace(id);
}


const GlobalPlace* TopologicalMap::getPlace(Id id) const
{
    auto placeIt = places_.find(id);
    return (placeIt == places_.end()) ? nullptr : placeIt->second.get();
}


const GlobalPathSegment* TopologicalMap::getPathSegment(int id) const
{
    auto segmentIt = segments_.find(id);
    return (segmentIt == segments_.end()) ? nullptr : segmentIt->second.get();
}


std::pair<TopologicalMap::Ptr, GlobalLocation> TopologicalMap::addArea(const GlobalLocation& location,
                                                                       const TopologicalVisit& visit) const
{
    // Make sure that the current location is a frontier, as that's the only location areas can be added.
    // Create a copy of the existing map.
    // Create the appropriate type of area for the visit.
    // Add the area to the set of GlobalAreas, along with the more specific instance type.
    // Remove the frontier transition.
    // Replace the previous saved transition with the updated transition that includes the correct area on the
    //  other side.

    if(location.areaId != kFrontierId)
    {
        return std::make_pair(nullptr, GlobalLocation());
    }

    Ptr newMap(new TopologicalMap(*this));

    auto newLocation = newMap->addConcreteArea(location, visit);
    utils::erase_remove(newMap->frontiers_, location.entryTransition);

    // Need to duplicate the exited area, as it will be modified by the following
    assert(hasArea(exited_area(location).id()));
    newMap->duplicateArea(exited_area(location));
    newMap->updateTransition(location.entryTransition, newLocation.entryTransition);

    return std::make_pair(newMap, newLocation);
}


std::pair<TopologicalMap::Ptr, GlobalLocation> TopologicalMap::closeLoop(const GlobalLocation& location,
                                                                         const GlobalArea& loopArea,
                                                                         const GlobalTransition& loopTrans,
                                                                         const TopologicalVisit& visit) const
{
    // Make sure that the current location is a frontier, as that's the only way a loop can be closed.
    // Create a copy of the existing map.
    // Create a new transition with the correct areas on each side.
    // Replace existing transitions with this new transition.
    // Remove any existing frontier transitions from the replaced transitions.
    // Update information for the newly entered area.

    if(location.areaId != kFrontierId)
    {
        return std::make_pair(nullptr, GlobalLocation());
    }

    Ptr newMap(new TopologicalMap(*this));

    GlobalArea prevArea = exited_area(location);
    GlobalTransition newTransition(next_id(),
                                   loopArea,
                                   prevArea,
                                   NavigationStatus::navigable,
                                   ExplorationStatus::explored);

    // Both areas involved in the loop closure need to be duplicated, as they are pointers to the parent map, but
    // they are changed areas.
    assert(hasArea(prevArea.id()));
    assert(hasArea(loopArea.id()));

    newMap->duplicateArea(prevArea);
    newMap->duplicateArea(loopArea);
    newMap->updateTransition(location.entryTransition, newTransition);
    newMap->updateTransition(loopTrans, newTransition);
    int numErased = 0;
    numErased += utils::erase_remove(newMap->frontiers_, location.entryTransition);
    numErased += utils::erase_remove(newMap->frontiers_, loopTrans);

    std::cout << "Creating loop between " << location.entryTransition << " and " << loopTrans << " Erased:" << numErased << '\n';

    auto newLocation = newMap->locationInMap(loopArea, newTransition);
    newMap->incorporateVisit(newLocation, visit);

    return std::make_pair(newMap, newLocation);
}


TopologicalMap::Ptr TopologicalMap::revisitArea(const GlobalLocation& location, const TopologicalVisit& visit) const
{
    // Make sure the current area exists in the map.
    // Create a copy of the map.
    //

    if(!hasArea(location.areaId))
    {
        return nullptr;
    }

    Ptr newMap(new TopologicalMap(*this));
    newMap->duplicateArea(current_area(location));
    newMap->incorporateVisit(location, visit);

    return newMap;
}


void TopologicalMap::incorporateVisit(const GlobalLocation& location, const TopologicalVisit& visit)
{
    // If the areas aren't the same type, then there's nothing to be done.
    if(location.areaType != visit.localArea()->type())
    {
        return;
    }

    if(location.areaType == AreaType::path_segment)
    {
        updatePathSegment(location, visit);
    }
    else
    {
        updatePlace(location, visit);
    }
}


void TopologicalMap::setReferenceFrames(const Chi& chi)
{
    chi_ = chi;
}


pose_distribution_t TopologicalMap::referenceFrame(Id id) const
{
    return chi_.getPlacePose(id);
}


GlobalLocation TopologicalMap::addConcreteArea(const GlobalLocation& location, const TopologicalVisit& visit)
{
    // In the map initialization case, there's no entry transition, so all transition handling should be disabled
    bool shouldUpdateEntry = location.entryTransition.valid();

    // Create the new area that will be added to the map
    GlobalArea newArea(next_id(), location.areaType);
    GlobalArea exitedArea = exited_area(location);
    // The transition changes to point to the new area
    GlobalTransition newTransition(next_id(),
                                   // The previous area is the other area from the current location entry
                                   exitedArea,
                                   newArea,
                                   NavigationStatus::navigable,
                                   ExplorationStatus::explored);

    // Create the new location -- the id and entry transition will be different, but other details will be the same
    auto newLocation = location;
    newLocation.areaId = newArea.id();

    if(shouldUpdateEntry)
    {
        newLocation.entryTransition = newTransition;
    }

#ifdef DEBUG_CREATE_MAP
    std::cout << "DEBUG::TopologicalMap: Added new area " << newArea << " at " << location << ".\n";
#endif

    // Convert the LocalArea into the appropriate type of global area -- either GlobalPathSegment or GlobalPlace.
    if(location.areaType == AreaType::path_segment)
    {
        bool isExplored = location.entryTransition.valid()
            && !exited_through_same_gateway(visit);
        auto newSegment = create_global_path_segment_from_local_path_segment(newArea,
                                                               visit.entryEvent().transitionGateway(),
                                                               *visit.localArea(),
                                                               isExplored);
        // Need to replace the entry transition from the fresh area with the entry transition linking with the
        // existing map
        if(shouldUpdateEntry)
        {
            newSegment.first.replaceTransition(newSegment.second, newTransition);
        }
        addPathSegmentFrontiers(newSegment.first);
        segments_[newArea.id()] = std::make_shared<GlobalPathSegment>(newSegment.first);

        // For the path, need to update the path direction
        if(newSegment.first.minusTransition() == newTransition)
        {
            newLocation.pathDirection = TopoDirection::plus;    // entered at minus so moving in plus
        }
        else if(newSegment.first.plusTransition() == newTransition)
        {
            newLocation.pathDirection = TopoDirection::minus;   // entered at plus so moving in minus
        }
        else
        {
            newLocation.pathDirection = TopoDirection::null;    // entered from a sequence, so no direction yet
        }
    }
    else // if(location.areaType == AreaType::decision_point || location.areaType == AreaType::destination)
    {
        auto newPlace = create_global_place_from_local_place(newArea,
                                                 visit.entryEvent().transitionGateway(),
                                                 *visit.localArea());
        if(shouldUpdateEntry)
        {
            newPlace.first.replaceTransition(newPlace.second, newTransition);
        }
        addPlaceFrontiers(newPlace.first);
        places_[newArea.id()] = std::make_shared<GlobalPlace>(newPlace.first);

        auto referenceFrame = estimateReferenceFrame(exitedArea, location.entryTransition);
        chi_.setPlacePose(newArea.id(), referenceFrame);
        // id + entry are sufficient to describe a place's location
    }

    // Add the area the stash of areas
    areas_[newArea.id()] = newArea;

#ifdef DEBUG_CREATE_MAP
    std::cout << " Updated location is " << newLocation << '\n';
#endif

    return newLocation;
}


void TopologicalMap::addPathSegmentFrontiers(const GlobalPathSegment& segment)
{
    if(segment.minusTransition().isFrontier())
    {
        frontiers_.push_back(segment.minusTransition());
    }

    if(segment.plusTransition().isFrontier())
    {
        frontiers_.push_back(segment.plusTransition());
    }

    for(auto& t : segment.leftSequence())
    {
        if(t.isFrontier())
        {
            frontiers_.push_back(t);
        }
    }

    for(auto& t : segment.rightSequence())
    {
        if(t.isFrontier())
        {
            frontiers_.push_back(t);
        }
    }
}


void TopologicalMap::addPlaceFrontiers(const GlobalPlace& place)
{
    // Iterate through all transitions in the place. If the transition is a frontier, add it to the set of frontiers
    for(auto& t : place.cycle())
    {
        if(t.isFrontier())
        {
            frontiers_.push_back(t);
        }
    }
}


void TopologicalMap::updatePathSegment(const GlobalLocation& location, const TopologicalVisit& visit)
{
    auto segment = getModifiableSegment(location.areaId);
    auto exitEvent = visit.exitEvent();

    // Nothing to do if the segment doesn't exist or there wasn't actual an exit that occurred
    if(!segment || !exitEvent)
    {
        return;
    }

    // Check if we exited from the same transition that we entered from. If so, then don't update the path segment
    // TODO: The logic can be smarter than this to determine if the new visit produced new information not included
    // in prior visits
    if(exited_through_same_gateway(visit))
    {
        return;
    }

    const LocalPathSegment* localSegment = static_cast<const LocalPathSegment*>(visit.localArea());
    auto newSegment = create_global_path_segment_from_local_path_segment(
        current_area(location),
        visit.entryEvent().transitionGateway(),
        *localSegment,
        true);  // would only be getting here if we have a valid location and didn't loop on self, so explored

    // Otherwise, get the lambda from the path segment and add it to the stored lambdas
    Lambda lambda = localSegment->lambda();
    // Lambda runs plus to minus in local frame, so might need to invert measurement if plus and minus are reversed
    // in the new path -- must update lambda before adjtusing the segment transitions
    if((segment->plusTransition() == location.entryTransition)
        != (newSegment.first.plusTransition() == newSegment.second))
    {
        lambda = lambda.invert();
    }

    segment->addLambda(lambda);

    // Create the new segment
    // If a frontier segment, then need to replace the frontier end with a non-frontier from the updated segment
    if(segment->isFrontier() && location.entryTransition.valid())
    {
        auto newEnd = opposite_end(newSegment.first, newSegment.second);
        auto oldEnd = opposite_end(*segment, location.entryTransition);

        std::replace(frontiers_.begin(),
                     frontiers_.end(),
                     oldEnd,
                     newEnd);

        // Swap out the transition to whatever new end has been detected
        segment->replaceTransition(oldEnd, newEnd);
    }
    // Otherwise if not a valid entry, then we're in the situation where potentially both end are frontiers,
    // so just replace them because this either has no effect, since we didn't drive through either end, so they
    // won't have been otherwise updated, or it makes them non-frontier, which is needed for successful mapping
    else if(!location.entryTransition.valid())
    {
        std::replace(frontiers_.begin(),
                     frontiers_.end(),
                     segment->plusTransition(),
                     newSegment.first.plusTransition());
        std::replace(frontiers_.begin(),
                     frontiers_.end(),
                     segment->minusTransition(),
                     newSegment.first.minusTransition());

        segment->replaceTransition(segment->plusTransition(), newSegment.first.plusTransition());
        segment->replaceTransition(segment->minusTransition(), newSegment.first.minusTransition());
        assert(segment->isFrontier());  // only went out one side, so must still be a frontier
    }
}


void TopologicalMap::updatePlace(const GlobalLocation& location, const TopologicalVisit& visit)
{
    auto placeIt = places_.find(location.areaId);

    if(placeIt != places_.end())
    {
        placeIt->second->changeMetricId(placeIt->second->metricPlaceId(), visit.localArea()->id());
    }
//     PRINT_PRETTY_STUB();
//     std::cout << "TODO: Handle incorporating data from multiple visits into a GlobalPlace.\n";
}


void TopologicalMap::updateTransition(const GlobalTransition& oldTrans, const GlobalTransition& newTrans)
{
    // For each area associated with the old transition, if the area is a non-frontier, then replace it with
    // the new transition.

    if(hasArea(oldTrans.plusArea().id()))
    {
        replaceTransitionForArea(oldTrans.plusArea(), oldTrans, newTrans);
    }

    if(hasArea(oldTrans.minusArea().id()))
    {
        replaceTransitionForArea(oldTrans.minusArea(), oldTrans, newTrans);
    }
}


GlobalLocation TopologicalMap::locationInMap(const GlobalArea& area, const GlobalTransition& entry) const
{
    // If on a path segment, then need to get the specific segment location
    if(area.type() == AreaType::path_segment)
    {
        const GlobalPathSegment* segment = getPathSegment(area.id());
        assert(segment);        // the map is constructed in invalid way if a transition leads to an undefined area
        return segment->locationOnSegment(entry);
    }
    // Otherwise, it is just a simple location with entry + area
    return GlobalLocation(area.id(), area.type(), entry);
}


bool TopologicalMap::replaceTransitionForArea(const GlobalArea& area,
                                              const GlobalTransition& oldTrans,
                                              const GlobalTransition& newTrans)
{
    if(area.type() == AreaType::path_segment)
    {
        auto segment = getModifiableSegment(area.id());
        assert(segment);
        return segment->replaceTransition(oldTrans, newTrans);
    }
    else
    {
        auto place = getModifiablePlace(area.id());
        assert(place);
        return place->replaceTransition(oldTrans, newTrans);
    }
}


pose_distribution_t TopologicalMap::estimateReferenceFrame(const GlobalArea& exitedArea,
                                                                  const GlobalTransition& exitTransition)
{
    // If the exitedArea is a path segment, then use the lambda value from the other end to get the estimated location
    // of this area. If there's a frontier at the other end, then it'll just assign 0,0,0 to start, but that's fine.
    // This estimate serves only to make the optimization more efficient by providing a good guess for new areas

    // If there's no exited area, then must be the initial area, so it has the default pose
    if((exitedArea.id() == kInvalidId) || (exitedArea.id() == kFrontierId))
    {
        return pose_distribution_t(0.0f, 0.0f, 0.0f, INFINITY, INFINITY, INFINITY);
    }
    // If a place was exited, then just use the previous reference frame as the guess
    if(exitedArea.type() != AreaType::path_segment)
    {
        return referenceFrame(exitedArea.id());
    }

    // Otherwise, it is a path segment, so get the area at the other end, apply the lambda and call it a day
    auto segment = getPathSegment(exitedArea.id());
    auto lambda = segment->lambda();

    GlobalArea otherArea;

    if(segment->plusTransition() == exitTransition)
    {
        otherArea = segment->minusTransition().otherArea(exitedArea);

        // Lambda is the transform from plus to minus. If we entered from the minus area, then the lambda needs
        // to be inverted to compute the relative motion along the path segment
        lambda = lambda.invert();
    }
    else // if(segment->minusTransition() == exitTransition)
    {
        otherArea = segment->plusTransition().otherArea(exitedArea);
    }

    // If there isn't an other place, then again just assume at the origin
    if((otherArea.id() == kInvalidId) || (otherArea.id() == kFrontierId))
    {
        return pose_distribution_t(0.0f, 0.0f, 0.0f, INFINITY, INFINITY, INFINITY);
    }

    auto otherPlace = getPlace(otherArea.id());

    return lambda.apply(referenceFrame(otherPlace->id()));
}


GlobalPlace* TopologicalMap::getModifiablePlace(Id id)
{
    auto placeIt = places_.find(id);
    return (placeIt == places_.end()) ? nullptr : placeIt->second.get();
}


GlobalPathSegment* TopologicalMap::getModifiableSegment(Id id)
{
    auto segmentIt = segments_.find(id);
    return (segmentIt == segments_.end()) ? nullptr : segmentIt->second.get();
}


void TopologicalMap::duplicateArea(GlobalArea area)
{
    if(is_place_type(area.type()))
    {
        auto newPlace = std::make_shared<GlobalPlace>(*getPlace(area.id()));
        places_[area.id()] = newPlace;
    }
    else if(area.type() == AreaType::path_segment)
    {
        auto newSegment = std::make_shared<GlobalPathSegment>(*getPathSegment(area.id()));
        segments_[area.id()] = newSegment;
    }
}


bool exited_through_same_gateway(const TopologicalVisit& visit)
{
    auto exitEvent = visit.exitEvent();

    if(!exitEvent)
    {
        return false;
    }

    auto entryGwy = visit.entryEvent().transitionGateway();
    auto exitGwy = exitEvent->transitionGateway();
    return entryGwy && exitGwy && entryGwy->isSimilarTo(*exitGwy);
}

} // namespace hssh
} // namespace vulcan
