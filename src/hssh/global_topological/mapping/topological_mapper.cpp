/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_mapper.cpp
* \author   Collin Johnson
*
* Definition of create_topological_mapper factory.
*/

#include <hssh/global_topological/mapping/topological_mapper.h>
#include <hssh/global_topological/mapping/tree_exploration_mapper.h>
#include <hssh/global_topological/mapping/exhaustive_search_mapper.h>
#include <hssh/global_topological/mapping/lazy_evaluation_mapper.h>
#include <hssh/global_topological/messages.h>
#include <hssh/global_topological/params.h>
#include <hssh/utils/local_place_transform.h>
#include <iostream>
#include <cassert>

// #define DEBUG_LOOP_CONNECTIONS
// #define DEBUG_NEW_PLACE

namespace vulcan
{
namespace hssh
{

std::unique_ptr<TopologicalMapper> create_topological_mapper(const std::string&                 mapperType,
                                                             const topological_mapper_params_t& params,
                                                             MetricMapCache&                manager)
{
    if(mapperType == TREE_EXPLORATION_MAPPER_TYPE)
    {
        return std::unique_ptr<TopologicalMapper>(new TreeExplorationMapper(manager));
    }
    else if(mapperType == EXHAUSTIVE_SEARCH_MAPPER_TYPE)
    {
        return std::unique_ptr<TopologicalMapper>(new ExhaustiveSearchMapper(params, manager));
    }
    else if(mapperType == LAZY_EVALUATION_MAPPER_TYPE)
    {
        return std::unique_ptr<TopologicalMapper>(new LazyEvaluationMapper(params, manager));
    }
    else
    {
        std::cerr<<"ERROR: Unknown TopologicalMapper type: "<<mapperType<<std::endl;
        assert(false);
    }

    return std::unique_ptr<TopologicalMapper>();
}

///////////////////////////// TopologicalMapper implementation ///////////////////////////////////

TopologicalMapper::TopologicalMapper(MetricMapCache& manager)
    : manager(manager)
{
}


void TopologicalMapper::setCorrectMap(uint32_t id)
{
    tree.setRootHypothesis(id);
}


void TopologicalMapper::setCorrectMap(TopoMapPtr map)
{
    tree.setRootHypothesis(map);
}


void TopologicalMapper::setGlobalLocationInMap(const set_global_location_message_t& message)
{
    TopoMapPtr map = tree.getMapFromId(message.mapId);

    if(map.get())
    {
        map->setGlobalLocation(message.location);
    }
}


bool TopologicalMapper::atFrontierPlace(const TopoMapPtr& hypothesis)
{
    return hypothesis->getGlobalLocation().placeId == PLACE_FRONTIER_ID;
}


bool TopologicalMapper::validLoopClosure(const place_connection_t& start, const place_connection_t& end)
{
    return (start.placeId != end.placeId) || (start.eventFragment.fragmentId != end.eventFragment.fragmentId);
}


void TopologicalMapper::initializeTree(const topology_action_t& action, const topology_measurements_t& measurements)
{
    LargeScaleStar actionStar(action.enteredAction.topology, action.enteredAction.entryPath);

    GlobalPlace newPlace(PLACE_FRONTIER_ID, measurements.placeId, actionStar);

    tree.createInitialHypothesis(newPlace);

    #ifdef DEBUG_PLACE_ENTRY
    std::cout<<"DEBUG:TopologicalMapper:Created initial map\n";
    #endif
}


TopoMapPtr TopologicalMapper::moveAlongKnownPath(const LargeScaleStar& star,
                                                 const Lambda&         lambda,
                                                 uint32_t              localPlaceId,
                                                 TopoMapPtr            hypothesis)
{
    GlobalPlace newPlace(PLACE_FRONTIER_ID, localPlaceId, star);

    // Use the rotation from the previous place because the defined the relative direction the robot was moving in between places
    // Once arrived, then want the transform in the current place because appearance models, etc. need that correlation
    GlobalLocation location = hypothesis->getGlobalLocation();

    #ifdef DEBUG_NEW_PLACE
    std::cout<<"DEBUG:TopologicalMapper:Moved to known place "<<newPlace.getId()<<" with lambda "<<lambda.rotate(location.localToReferenceTransform.theta)<<'\n';
    #endif

    TopoMapPtr newHypothesis = tree.copyMapToNextDepth(hypothesis, lambda.rotate(location.localToReferenceTransform.theta));

    location = newHypothesis->getGlobalLocation();
    location.localToReferenceTransform = findTransformBetweenPlaces(newPlace, newHypothesis->getPlace(location.placeId), star.getEntryPathFragment().fragmentId);
    newHypothesis->setGlobalLocation(location);

    return newHypothesis;
}


TopoMapPtr TopologicalMapper::createNewPlaceHypothesis(const LargeScaleStar& star,
                                                       const Lambda&         lambda,
                                                       uint32_t              localPlaceId,
                                                       TopoMapPtr            hypothesis)
{
    GlobalPlace newPlace(PLACE_FRONTIER_ID, localPlaceId, star);

    // Use the rotation from the previous place because the defined the relative direction the robot was moving in between places
    // Once arrived, then want the transform in the current place because appearance models, etc. need that correlation
    GlobalLocation location = hypothesis->getGlobalLocation();

    #ifdef DEBUG_NEW_PLACE
    std::cout<<"DEBUG:TopologicalMapper:Adding new place "<<newPlace.getId()<<" with lambda "<<lambda.rotate(location.localToReferenceTransform.theta)<<'\n';
    #endif

    TopoMapPtr newHypothesis = tree.addNewPlace(hypothesis, newPlace, star.getEntryPathFragment().fragmentId, lambda.rotate(location.localToReferenceTransform.theta));

    return newHypothesis;
}


std::vector<TopoMapPtr> TopologicalMapper::createLoopClosureHypotheses(const LargeScaleStar& star,
                                                                       const Lambda&         lambda,
                                                                       uint32_t              localPlaceId,
                                                                       TopoMapPtr            hypothesis)
{
    /*
    * Take a slow approach for now, and iterate through all places, checking each to see if it is a frontier or not, and
    * if so, whether the stars match. If the hypothesis has a better internal rep, this search can be drastically reduced.
    *
    * The LargeScaleStar happily contains potentialEntrySegments() method that will determine all possible links between two
    * stars, given the topology and layout of frontiers. For each of these segments, create a place_connection_t and spawn
    * a new map hypothesis.
    */

    GlobalPlace             currentPlace(PLACE_FRONTIER_ID, localPlaceId, star);
    std::vector<TopoMapPtr> newHypotheses;

    // Use the rotation from the previous place because the defined the relative direction the robot was moving in between places
    // Once arrived, then want the transform in the current place because appearance models, etc. need that correlation
    GlobalLocation  location         = hypothesis->getGlobalLocation();
    assert(location.pathState.entryPlaceId != PLACE_FRONTIER_ID);
    GlobalPlace&       previousPlace = hypothesis->getPlace(location.pathState.entryPlaceId);
    place_connection_t start         = {previousPlace.getId(), previousPlace.getStar().getFragmentWithId(location.pathState.entryFragmentId)};

    const std::unordered_map<int, GlobalPlace>& places = hypothesis->getPlaces();

    std::vector<global_path_fragment_t> potentialFragments;

    Lambda rotated(lambda.rotate(location.localToReferenceTransform.theta));

    for(auto placeIt = places.begin(), placeEnd = places.end(); placeIt != placeEnd; ++placeIt)
    {
        assert(placeIt->second.getId() != PLACE_FRONTIER_ID);

        GlobalPlace&          topoPlace = hypothesis->getPlace(placeIt->second.getId());
        const LargeScaleStar& placeStar = topoPlace.getStar();

        potentialFragments = placeStar.potentialEntryFragments(star, star.getEntryPathFragment());

        #ifdef DEBUG_LOOP_CONNECTIONS
        if(placeStar == star && !potentialFragments.empty())
        {
            std::cout<<"DEBUG:TopologicalMapper:Potential match with "<<placeIt->second.getId()<<" Num segments:"<<potentialFragments.size()<<'\n';
        }
        #endif

        for(auto segmentIt = potentialFragments.begin(), segmentEnd = potentialFragments.end(); segmentIt != segmentEnd; ++segmentIt)
        {
            place_connection_t end = {topoPlace.getId(), *segmentIt};

            if(validLoopClosure(start, end))
            {
                pose_t transform     = findTransformBetweenPlaces(currentPlace, topoPlace, end.eventFragment.fragmentId);
                TopoMapPtr    newHypothesis = tree.connectPlaces(hypothesis, start, end, rotated, transform);

                newHypotheses.push_back(newHypothesis);

                assert(start.placeId != end.placeId || start.eventFragment.fragmentId != end.eventFragment.fragmentId);
                #ifdef DEBUG_LOOP_CONNECTIONS
                std::cout<<"DEBUG:TopologicalMapper:Connected "<<star.getEntryPathFragment().fragmentId<<" to "
                         <<end.placeId<<':'<<end.eventFragment.fragmentId<<" with lambda "<<rotated<<'\n';
                #endif
            }
        }
    }

    return newHypotheses;
}


pose_t TopologicalMapper::findTransformBetweenPlaces(const GlobalPlace& currentPlace, const GlobalPlace& referencePlace, int referenceId)
{
    const LocalPlace& current   = manager.load(currentPlace.getMetricPlaceId());
    const LocalPlace& reference = manager.load(referencePlace.getMetricPlaceId());

    int currentId = currentPlace.getEntryFragment().fragmentId;

    local_path_fragment_t currentFragment   = current.getStar().getFragmentWithId(currentId);
    local_path_fragment_t referenceFragment = reference.getStar().getFragmentWithId(referenceId);

    return calculate_transform_to_reference_place(current, reference, currentFragment, referenceFragment);
}

} // namespace hssh
} // namespace vulcan
