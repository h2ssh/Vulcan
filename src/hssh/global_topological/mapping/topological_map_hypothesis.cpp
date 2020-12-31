/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     topological_map_hypothesis.cpp
 * \author   Collin Johnson
 *
 * Definition of TopologicalMapHypothesis as defined in topological_map_hypothesis.h
 */

#include "hssh/global_topological/mapping/topological_map_hypothesis.h"
#include "core/angle_functions.h"
#include "hssh/global_topological/topological_map.h"
#include <cassert>
#include <iostream>
#include <queue>

// #define DEBUG_PATH_CREATION
// #define DEBUG_TOPO_MAP_CONTENTS
// #define DEBUG_TO_SINGLE_REF
// #define DEBUG_CONNECT_PLACES
// #define DEBUG_MERGE_PATHS

namespace vulcan
{
namespace hssh
{

path_direction_t path_frontier_direction(const GlobalPath& path,
                                         path_state_t state);   // find direction of frontier from this place
path_direction_t path_frontier_direction(const GlobalPath& path,
                                         path_transition_t transition);   // find direction of frontier from this place
void print_places_and_paths(const std::unordered_map<int, GlobalPlace>& places,
                            const std::unordered_map<int, GlobalPath>& paths);
void print_path(const GlobalPath& path);


TopologicalMapHypothesis::TopologicalMapHypothesis(const GlobalPlace& initialPlace)
: needOptimization(false)
, nextPlaceId(0)
, nextPathId(0)
, pruned(false)
{
    places.insert(std::make_pair(nextPlaceId, initialPlace));
    GlobalPlace& initial = places[nextPlaceId];
    initial.id = nextPlaceId++;

    location.placeId = initial.id;
    location.placeState.entryFragmentId = initial.getEntryFragment().fragmentId;

    createPathsForPlace(initial, location.placeState.entryFragmentId, Lambda());

    chi.setPlacePose(initial.getId(), initial.referenceFrame());
}


TopologicalMapHypothesis::TopologicalMapHypothesis(const TopologicalMap& map)
: TopologicalMap(map)
, needOptimization(true)
, nextPlaceId(0)
, nextPathId(0)
, pruned(false)
{
    // Need to find the max place id and max path id so the next id can be set for these values
    // Need to set the place pose in chi so the optimization has somewhere to get started

    for (auto placeIt = places.begin(), placeEnd = places.end(); placeIt != placeEnd; ++placeIt) {
        chi.setPlacePose(placeIt->second.getId(), placeIt->second.referenceFrame());

        nextPlaceId = std::max(placeIt->second.getId() + 1, nextPlaceId);
    }

    for (auto pathIt = paths.begin(), pathEnd = paths.end(); pathIt != pathEnd; ++pathIt) {
        nextPathId = std::max(pathIt->second.getId() + 1, nextPathId);
    }
}


TopologicalMapHypothesis::TopologicalMapHypothesis(const TopologicalMapHypothesis& toCopy)
: TopologicalMap(toCopy)
, chi(toCopy.chi)
, needOptimization(toCopy.needOptimization)
, nextPlaceId(toCopy.nextPlaceId)
, nextPathId(toCopy.nextPathId)
, pruned(toCopy.pruned)
{
    // Only thing to not copy is the children
    assert(!toCopy.pruned);
}


GlobalPlace& TopologicalMapHypothesis::getPlace(int placeId)
{
    return places[placeId];
}


const GlobalPlace& TopologicalMapHypothesis::getPlace(int placeId) const
{
    return places.find(placeId)->second;
}


GlobalPath& TopologicalMapHypothesis::getPath(int pathId)
{
    return paths[pathId];
}


const GlobalPath& TopologicalMapHypothesis::getPath(int pathId) const
{
    return paths.find(pathId)->second;
}


void TopologicalMapHypothesis::setGlobalLocation(const GlobalLocation& newLocation)
{
    location = newLocation;

    if (!location.onPath
        && hasPlace(location.placeId))   // if at a place, then need to set entry fragment for the place
    {
        GlobalPlace& place = getPlace(location.placeId);
        place.setEntryFragment(place.getStar().getFragmentWithId(location.placeState.entryFragmentId));
    }
}


void TopologicalMapHypothesis::setChi(const Chi& newChi)
{
    chi = newChi;
    needOptimization = false;

    setReferenceFramesBasedOnChi();
}


TopologicalMap TopologicalMapHypothesis::toSingleReference(void) const
{
    return TopologicalMap(id, depth, places, paths, location, probability);
}


void TopologicalMapHypothesis::updateLambdaOnLastPathSegment(const Lambda& lambda)
{
    GlobalPath& path = paths[location.pathId];
    path_transition_t lastTransition(location.pathState.entryPlaceId, location.pathState.entryFragmentId);
    GlobalPathSegment segment = path.nextSegmentAlongPath(lastTransition, location.pathDirection);

    for (size_t n = 0; n < path.segments.size(); ++n) {
        if ((path.segments[n].getPlusTransition() == segment.getPlusTransition())
            && (path.segments[n].getMinusTransition() == segment.getMinusTransition())) {
            //             path.segments[n].addLambda((state.pathState.direction == PATH_MINUS) ? lambda :
            //             lambda.invert()); std::cout<<"Added a lambda to segment. Now
            //             "<<path.segments[n].getAllLambdas().size()<<" lambdas for segment.\n"; auto lambdas =
            //             path.segments[n].getAllLambdas(); std::for_each(lambdas.begin(), lambdas.end(), [](const
            //             Lambda& lambda) { std::cout<<lambda<<'\n'; });
        }
    }
}


int TopologicalMapHypothesis::addNewPlace(GlobalPlace& newPlace, unsigned int entryFragmentId, const Lambda& lambda)
{
    places.insert(std::make_pair(nextPlaceId, newPlace));

    GlobalPlace& createdPlace = places[nextPlaceId];
    createdPlace.id = nextPlaceId++;

    createPathsForPlace(createdPlace, entryFragmentId, lambda);

    // If there was an entry place for the path, then mark its entry segment as explored because the robot
    // came from a known location to arrive at this place.
    if (location.pathState.entryPlaceId != PATH_FRONTIER_ID) {
        GlobalPlace& entryPlace = places[location.pathState.entryPlaceId];
        entryPlace.star.setFragmentExploration(location.pathState.entryFragmentId, PATH_FRAGMENT_EXPLORED);
        createdPlace.star.setFragmentExploration(entryFragmentId, PATH_FRAGMENT_EXPLORED);
    }

    location.placeId = createdPlace.id;
    location.pathState.targetPlaceId = createdPlace.id;
    location.placeState.entryFragmentId = entryFragmentId;

    // When adding a new place, the robot is in the original reference frame of the place, so no transformation
    // needs to be made
    location.localToReferenceTransform = pose_t(0, 0, 0);

    newPlace.id = createdPlace.id;

    assert(location.placeId != PLACE_FRONTIER_ID);

    return createdPlace.id;
}


path_direction_t
  TopologicalMapHypothesis::createPathsForPlace(GlobalPlace& newPlace, int entryFragmentId, const Lambda& lambda)
{
    // TODO: Cleanup this method. Super-ugly. Find the abstractions.

    LargeScaleStar& placeStar = newPlace.star;
    std::vector<global_path_fragment_t> segments = placeStar.getPathFragments();

    assert(segments.size() % 2 == 0);

    bool havePath = paths.find(location.pathId) != paths.end();
    int numPaths = segments.size() / 2;

    path_direction_t direction = PATH_NONE;

    for (size_t n = 0; n < segments.size() / 2; ++n) {
        assert(segments[n].pathId == segments[n + numPaths].pathId);

        if (((segments[n].fragmentId == entryFragmentId) || (segments[n + numPaths].fragmentId == entryFragmentId))
            && havePath) {
            GlobalPath& entryPath = paths[location.pathId];
            global_path_fragment_t starFragment = placeStar.getFragmentWithId(entryFragmentId);
            global_path_fragment_t otherFragment = placeStar.getOtherFragmentOnPath(entryFragmentId);
            path_direction_t direction = path_frontier_direction(entryPath, location.pathState);

            addPlaceToExistingPath(entryPath, newPlace, starFragment, otherFragment, lambda, direction);
        } else   // create a new path
        {
            createNewPath(newPlace, segments[n], segments[n + numPaths]);
        }
    }

    return direction;
}


path_transition_t TopologicalMapHypothesis::addPlaceToExistingPath(GlobalPath& path,
                                                                   GlobalPlace& newPlace,
                                                                   const global_path_fragment_t& entryFragment,
                                                                   const global_path_fragment_t& otherFragment,
                                                                   const Lambda& lambda,
                                                                   path_direction_t direction)
{
    LargeScaleStar& placeStar = newPlace.star;

    if (!entryFragment.navigable) {
        std::cerr << "ERROR:TopoMap:Assigning path to a non-navigable path segment. Place:" << newPlace.id
                  << " Path segment:" << entryFragment.fragmentId << " Path:" << path.getId() << '\n';
        assert(false);
    }

    path_transition_t endTransition(newPlace.getId(), entryFragment.fragmentId);

    // When adding the place, the measured lambda is going to be from the old place TO the new place. This corresponds
    // to motion in the PATH_MINUS direction. If PATH_PLUS, then the lambda needs to be flipped because the direction of
    // the place is reversed.
    Lambda pathLambda = (direction == PATH_MINUS) ? lambda : lambda.invert();

    if (!path.addPlace(endTransition, pathLambda, direction)) {
        std::cerr << "ERROR:TopoMap:Failed to add place " << newPlace.getId() << ':' << entryFragment.fragmentId
                  << " to path " << path.getId() << '\n';
        endTransition.placeId = PATH_ENDPOINT_ID;
        assert(false);
    }

    if (otherFragment.navigable) {
        path_transition_t frontierTransition(newPlace.getId(), otherFragment.fragmentId);

        if ((otherFragment.exploration == PATH_FRAGMENT_FRONTIER) && !path.addFrontier(frontierTransition, direction)) {
            std::cerr << "ERROR:TopoMap:Failed to add frontier at " << newPlace.getId() << ':'
                      << otherFragment.fragmentId << " to path " << path.getId() << '\n';
            assert(false);
        }
    }

    // Assign a reference frame for the new place based on the previous place on the path
    path_transition_t previousPlace = path.nextPlaceAlongPath(endTransition, opposite_direction(direction));
    assert(hasPlace(previousPlace.placeId));
    pose_t previousPose = places[previousPlace.placeId].referenceFrame();

    newPlace.referenceFrame =
      pose_t(previousPose.x + lambda.x, previousPose.y + lambda.y, angle_sum(previousPose.theta, lambda.theta));
    chi.setPlacePose(newPlace.id, newPlace.referenceFrame);

    // Assign the segments in the star to have the correct path ids now that they have been reassigned
    // If moving along the path, the entry segment will have the opposite direction, as leaving through
    // that segment would result in motion in the opposite direction
    placeStar.assignPath(entryFragment.fragmentId, path.getId(), opposite_direction(direction));
    placeStar.assignPath(otherFragment.fragmentId, path.getId(), direction);

    return endTransition;
}


int TopologicalMapHypothesis::createNewPath(GlobalPlace& newPlace,
                                            const global_path_fragment_t& plusFragment,
                                            const global_path_fragment_t& minusFragment)
{
    int pathId = nextPathId++;

    paths.insert(std::make_pair(pathId, GlobalPath(pathId)));

    LargeScaleStar& placeStar = newPlace.star;

    placeStar.assignPath(plusFragment.fragmentId, pathId, PATH_PLUS);
    placeStar.assignPath(minusFragment.fragmentId, pathId, PATH_MINUS);

    placeStar.setFragmentExploration(plusFragment.fragmentId, PATH_FRAGMENT_FRONTIER);
    placeStar.setFragmentExploration(minusFragment.fragmentId, PATH_FRAGMENT_FRONTIER);

    GlobalPath& addedPath = paths[pathId];

    // Now add the frontiers for these fragments if needed
    if (plusFragment.navigable
        && !addedPath.addFrontier(path_transition_t(newPlace.getId(), plusFragment.fragmentId), PATH_PLUS)) {
        std::cerr << "ERROR:TopoMap:Failed to add frontier at " << newPlace.getId() << ':' << plusFragment.fragmentId
                  << " to path " << addedPath.getId() << '\n';
        assert(false);
    }

    if (minusFragment.navigable
        && !addedPath.addFrontier(path_transition_t(newPlace.getId(), minusFragment.fragmentId), PATH_MINUS)) {
        std::cerr << "ERROR:TopoMap:Failed to add frontier at " << newPlace.getId() << ':' << minusFragment.fragmentId
                  << " to path " << addedPath.getId() << '\n';
        assert(false);
    }

    return pathId;
}


void TopologicalMapHypothesis::connectPlaces(const place_connection_t& to,
                                             const place_connection_t& from,
                                             const Lambda& lambda,
                                             const pose_t& transform)
{
#ifdef DEBUG_CONNECT_PLACES
    std::cout << "DEBUG:TopoMap:connecting:" << from.eventFragment.pathId << " to " << to.eventFragment.pathId << '\n';
    std::cout << "Connecting places with state:" << state << '\n';
#endif

    assert((paths.find(to.eventFragment.pathId) != paths.end())
           && (paths.find(from.eventFragment.pathId) != paths.end()));

    // Merge the two paths if they are different. It's possible to have single place on path twice if a loop occurs
    if (from.eventFragment.pathId == to.eventFragment.pathId) {
        connectPlacesOnSamePath(to, from, lambda);
    } else {
        connectPlacesOnDifferentPaths(to, from, lambda);
    }

    location.placeId = to.placeId;
    location.pathState.targetPlaceId = to.placeId;
    location.placeState.entryFragmentId = to.eventFragment.fragmentId;
    location.localToReferenceTransform = transform;

    needOptimization = true;

    assert(location.placeId != PLACE_FRONTIER_ID);
}


void TopologicalMapHypothesis::connectPlacesOnDifferentPaths(const place_connection_t& to,
                                                             const place_connection_t& from,
                                                             const Lambda& lambda)
{
    GlobalPlace& fromPlace = places[from.placeId];
    GlobalPlace& toPlace = places[to.placeId];

    GlobalPath& fromPath = paths[from.eventFragment.pathId];
    GlobalPath& toPath = paths[to.eventFragment.pathId];

    LargeScaleStar& fromStar = fromPlace.star;

    global_path_fragment_t starFragment = fromStar.getFragmentWithId(from.eventFragment.fragmentId);
    global_path_fragment_t otherStarFragment = fromStar.getOtherFragmentOnPath(from.eventFragment.fragmentId);

    path_transition_t toTransition(to.placeId, to.eventFragment.fragmentId);
    path_transition_t fromTransition(from.placeId, from.eventFragment.fragmentId);

#ifdef DEBUG_CONNECT_PLACES
    std::cout << "Connecting path " << fromPath << " to " << toPath << '\n';
#endif

    path_direction_t toFrontierDirection = path_frontier_direction(toPath, toTransition);
    path_direction_t fromFrontierDirection = path_frontier_direction(fromPath, fromTransition);
    // When connecting this place to the destination path, the lambda needs to be inverted because the logic for adding
    // a place normally assumes the place was creating traveling from a place on the path to the frontier, not from the
    // frontier to the place. Hence, inverting lambda.
    path_transition_t mergeTransition =
      addPlaceToExistingPath(toPath, fromPlace, starFragment, otherStarFragment, lambda.invert(), toFrontierDirection);

    fromStar.setFragmentExploration(from.eventFragment.fragmentId, PATH_FRAGMENT_EXPLORED);

    toPlace.star.setFragmentExploration(to.eventFragment.fragmentId, PATH_FRAGMENT_EXPLORED);
    toPlace.star.setEntryPathFragment(to.eventFragment.fragmentId);

    // Both path frontiers were in the same direction, then the direction for the from path segments will
    // be flipped because the from path direction would have to be reversed to ensure the frontiers aligned properly
    mergePaths(fromPath, toPath, mergeTransition, opposite_direction(fromFrontierDirection), toFrontierDirection);
    renamePath(fromPath, toPath.getId(), (fromFrontierDirection == toFrontierDirection));

    // Erase the old path now that it has been merged and properly renamed
    paths.erase(fromPath.getId());
}


void TopologicalMapHypothesis::connectPlacesOnSamePath(const place_connection_t& to,
                                                       const place_connection_t& from,
                                                       const Lambda& lambda)
{
    GlobalPlace& fromPlace = places[from.placeId];
    GlobalPlace& toPlace = places[to.placeId];

    GlobalPath& path = paths[from.eventFragment.pathId];

    if (path.createLoop(lambda)) {
        // Mark the segments as explored, but otherwise no changes needed because they already exist on the same path
        fromPlace.star.setFragmentExploration(from.eventFragment.fragmentId, PATH_FRAGMENT_EXPLORED);
        toPlace.star.setFragmentExploration(to.eventFragment.fragmentId, PATH_FRAGMENT_EXPLORED);
        toPlace.star.setEntryPathFragment(to.eventFragment.fragmentId);
    }
}


void TopologicalMapHypothesis::mergePaths(GlobalPath& source,
                                          GlobalPath& destination,
                                          const path_transition_t& mergeTransition,
                                          path_direction_t sourceDirection,
                                          path_direction_t destinationDirection)
{
    /*
     * Path merging appends one path onto another. source already contains mergePlaceId and now all places in source
     * need to be moved to destination.
     *
     * destinationDirection is location of mergePlaceId along the destination path from the previous endpoint of the
     * path. sourceDirection is direction of frontier along the the source path. If the directions are the same, then
     * the lambdas of the source path need their signs flipped, as the places along the source path are moving in the
     * opposite direction of the destination path. i.e.
     *
     *   source:  4 3 -1    dir: minus
     *   dest:    1 2 3 -1  dir: minus
     *   final:   1 2 3 4
     *
     * Clearly, 4 comes after 3 in the merged path, so lambda has to be changed around.
     */

    path_transition_t currentTransition = mergeTransition;
    path_transition_t nextTransition;

    GlobalPathSegment nextSegment;

    // Continuing copying over the places until the current place is a frontier, meaning the end of a path, or the
    // endpoint of the path is found
    while (currentTransition.placeId >= PATH_MIN_PLACE_ID) {
        nextSegment = source.nextSegmentAlongPath(currentTransition, sourceDirection);
        nextTransition =
          (sourceDirection == PATH_PLUS) ? nextSegment.getPlusTransition() : nextSegment.getMinusTransition();

#ifdef DEBUG_MERGE_PATHS
        std::cout << "Current:" << currentTransition.placeId << " Next:" << nextTransition.placeId
                  << " Source:" << source << " Destination:" << destination << '\n';
#endif

        if (nextSegment.getPlusTransition().placeId >= PATH_MIN_PLACE_ID
            || nextSegment.getMinusTransition().placeId >= PATH_MIN_PLACE_ID) {
            assert(destination.addGlobalPathSegment((sourceDirection != destinationDirection) ? nextSegment.reverse()
                                                                                              : nextSegment,
                                                    destinationDirection));

            GlobalPlace& currentPlace = places[currentTransition.placeId];
            if (nextTransition.placeId != PATH_FRONTIER_ID) {
                currentPlace.star.setFragmentExploration(currentTransition.transitionFragmentId,
                                                         PATH_FRAGMENT_EXPLORED);
            }
        }

        currentTransition = nextTransition;
    }

#ifdef DEBUG_MERGE_PATHS
    std::cout << "Final:" << destination << '\n';
#endif
}


void TopologicalMapHypothesis::renamePath(GlobalPath& oldPath, int newId, bool reverseDirection)
{
    // Go through the LargeScaleStars and change the path id for all segments involved on the path
    std::vector<int> placesToChange = oldPath.getPlaces();

    for (size_t n = 0; n < placesToChange.size(); ++n) {
        if (placesToChange[n] >= PATH_MIN_PLACE_ID) {
            GlobalPlace& placeToChange = places[placesToChange[n]];
            placeToChange.star.changePath(oldPath.getId(), newId, reverseDirection);
        }
    }

    if (location.pathId == oldPath.getId()) {
        location.pathId = newId;
    }

    if (reverseDirection) {
        location.pathDirection = opposite_direction(location.pathDirection);
    }
}


void TopologicalMapHypothesis::setReferenceFramesBasedOnChi(void)
{
    for (auto placeIt = places.begin(), placeEnd = places.end(); placeIt != placeEnd; ++placeIt) {
        GlobalPlace& place = placeIt->second;
        place.referenceFrame = chi.getPlacePose(place.getId());
    }
}


path_direction_t path_frontier_direction(const GlobalPath& path, path_state_t state)
{
    path_transition_t transition(state.entryPlaceId, state.entryFragmentId);

    return path_frontier_direction(path, transition);
}


path_direction_t path_frontier_direction(const GlobalPath& path,
                                         path_transition_t transition)   // find direction of frontier from this place
{
    GlobalPathSegment nextSegment = path.nextSegmentAlongPath(transition, PATH_MINUS);

    if (nextSegment.getPlusTransition() == transition && nextSegment.getMinusTransition().placeId == PATH_FRONTIER_ID) {
        return PATH_MINUS;
    }

    nextSegment = path.nextSegmentAlongPath(transition, PATH_PLUS);

    if (nextSegment.getMinusTransition() == transition && nextSegment.getPlusTransition().placeId == PATH_FRONTIER_ID) {
        return PATH_PLUS;
    } else {
        std::cerr << "WARNING:TopoMap:No frontier on path " << path.getId() << " on either side of place "
                  << transition.placeId << '\n';
        return PATH_NONE;
    }
}


void print_places_and_paths(const std::unordered_map<int, GlobalPlace>& places,
                            const std::unordered_map<int, GlobalPath>& paths)
{
    std::cout << "DEBUG:TopoMap:\nPlaces:";
    for (auto placeIt = places.begin(); placeIt != places.end(); ++placeIt) {
        std::cout << placeIt->second.getId() << ' ';
    }
    std::cout << '\n';

    for (auto pathIt = paths.begin(); pathIt != paths.end(); ++pathIt) {
        std::vector<int> pathPlaces = pathIt->second.getPlaces();

        std::cout << "Path " << pathIt->second.getId() << ':';

        for (size_t n = 0; n < pathPlaces.size(); ++n) {
            std::cout << pathPlaces[n] << ' ';
        }
        std::cout << '\n';
    }
}


void print_path(const GlobalPath& path)
{
    std::vector<int> pathPlaces = path.getPlaces();

    std::cout << "Path " << path.getId() << ':';

    for (size_t n = 0; n < pathPlaces.size(); ++n) {
        std::cout << pathPlaces[n] << ' ';
    }
    std::cout << '\n';
}

}   // namespace hssh
}   // namespace vulcan
