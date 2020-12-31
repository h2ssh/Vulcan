/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_topo_planner.cpp
 * \author   Collin Johnson
 *
 * Definition of GoalPlanner.
 */

#include "planner/goal/goal_planner.h"
#include "hssh/global_topological/topological_map.h"
#include "planner/decision/decision_target.h"
#include "planner/goal/debug_info.h"
#include "planner/goal/goal_target.h"
#include "planner/goal/search.h"
#include <cassert>
#include <iostream>

#define DEBUG_MAP
#define DEBUG_PATH
#define DEBUG_ROUTE
#define DEBUG_SEQUENCE

namespace vulcan
{
namespace planner
{

void print_map(const hssh::TopologicalMap& map);
void print_path(const graph::Path<hssh::TopologicalVertex>& path);
void print_route(const GoalRoute& route);
void print_sequence(const DecisionTargetSequence& sequence);


GoalPlanner::GoalPlanner(void) : haveGraph(false), graphId(0), routeId(0)
{
}


bool GoalPlanner::plan(const GoalTarget& target, const hssh::TopologicalMap& map, goal_debug_info_t& info)
{
    hssh::TopologicalVertex start = hssh::convert_location_to_vertex(map.getGlobalLocation(), map);
    hssh::TopologicalVertex goal = hssh::convert_place_to_vertex(target.getTargetPlace());

    initializeGraph(map);

    AStarSearch searcher(graph);

    bool foundPath = searcher.search(start, goal);

    print_path(searcher.getPath());

    if (foundPath) {
        convertPathToRoute(searcher.getPath(), map);
        convertRouteToSequence(map);
    }

    info.searchInfo = searcher.getDebugInfo();
    info.searchInfo.graph = graph;

    return foundPath;
}


void GoalPlanner::initializeGraph(const hssh::TopologicalMap& map)
{
    if (!haveGraph || (map.getId() != graphId)) {
        graph = hssh::convert_map_to_graph(map);

        haveGraph = true;
        graphId = map.getId();

        print_map(map);
    }
}


void GoalPlanner::convertPathToRoute(const graph::Path<hssh::TopologicalVertex>& path, const hssh::TopologicalMap& map)
{
    const std::vector<hssh::TopologicalVertex>& vertices = path.getPath();

    std::vector<goal_route_element_t> elements;

    for (auto vertexIt = vertices.begin(), vertexEnd = vertices.end(); vertexIt != vertexEnd; ++vertexIt) {
        hssh::NodeData node = vertexIt->getVertexData();

        if (node.isSegment) {
            elements.push_back(goal_route_element_t(*map.getPathSegment(node.id)));
        } else   // place!
        {
            elements.push_back(goal_route_element_t(*map.getPlace(node.id)));
        }
    }

    route = GoalRoute(routeId++, elements);

    print_route(route);
}


void GoalPlanner::convertRouteToSequence(const hssh::TopologicalMap& map)
{
    const std::vector<goal_route_element_t>& elements = route.getSequence();

    std::vector<std::shared_ptr<DecisionTarget>> targets;

    // If a single element, then already at the target, so no need to actually create a new sequence
    if (elements.size() > 1) {
        targets.push_back(createInitialTarget(elements[0], elements[1], map.getGlobalLocation()));

        createTargetsForRemainingElements(elements, targets);
    }

    sequence = DecisionTargetSequence(routeId++, targets);

    print_sequence(sequence);
}


GoalPlanner::fragment_info_t
  GoalPlanner::getFragmentInfoFromSegment(const hssh::GlobalPathSegment& segment, int placeId, bool entryFragment)
{
    assert((segment.getPlusTransition().placeId == placeId) || (segment.getMinusTransition().placeId == placeId));

    fragment_info_t info;

    info.pathId = segment.getPathId();

    if (segment.getPlusTransition().placeId == placeId) {
        info.direction = entryFragment ? hssh::PATH_MINUS : hssh::PATH_PLUS;
    } else if (segment.getMinusTransition().placeId == placeId) {
        info.direction = entryFragment ? hssh::PATH_PLUS : hssh::PATH_MINUS;
    } else {
        std::cout << "ERROR:GoalPlanner:Place " << placeId
                  << " was not on the specified path segment:" << segment.getPlusTransition().placeId << "->"
                  << segment.getMinusTransition().placeId << '\n';
        assert(false);
    }

    return info;
}


GoalPlanner::fragment_info_t GoalPlanner::getFragmentInfoFromLocation(const hssh::GlobalLocation& location)
{
    fragment_info_t info;

    // Remember the direction of a fragment is the direction you would move if EXITING through it. The placeState
    // direction will be the direction through which the place was entered, thus need to flip the directions to
    // determine the entry fragment
    info.pathId = location.pathId;
    info.direction = (location.pathDirection == hssh::PATH_PLUS) ? hssh::PATH_MINUS : hssh::PATH_PLUS;

    return info;
}


std::shared_ptr<DecisionTarget> GoalPlanner::createInitialTarget(const goal_route_element_t& firstElement,
                                                                 const goal_route_element_t& secondElement,
                                                                 const hssh::GlobalLocation& location)
{
    assert(firstElement.isPathSegment || firstElement.isPlace);

    if (firstElement.isPathSegment) {
        std::cout << "initial was path\n";
        return createInitialPathTarget(firstElement.segment, secondElement.place.getId(), location);
    } else   // if(firstElement.isPlace)
    {
        fragment_info_t entry = getFragmentInfoFromLocation(location);
        fragment_info_t exit = getFragmentInfoFromSegment(secondElement.segment, firstElement.place.getId(), true);

        std::cout << "initial was place:" << entry.pathId << ':' << entry.direction << ' ' << exit.pathId << ':'
                  << exit.direction << '\n';
        return createPlaceTarget(firstElement.place.getStar(), entry, exit);
    }
}


void GoalPlanner::createTargetsForRemainingElements(const std::vector<goal_route_element_t>& elements,
                                                    std::vector<std::shared_ptr<DecisionTarget>>& targets)
{
    // Don't need to process the final element because the robot will be at the goal place at that point in time
    for (size_t n = 1; n < elements.size() - 1; ++n) {
        if (elements[n].isPathSegment) {
            std::cout << "adding path\n";
            targets.push_back(createPathTarget());
        } else if (elements[n].isPlace) {
            fragment_info_t entry =
              getFragmentInfoFromSegment(elements[n - 1].segment, elements[n].place.getId(), false);
            fragment_info_t exit = getFragmentInfoFromSegment(elements[n + 1].segment, elements[n].place.getId(), true);

            std::cout << "adding place:" << entry.pathId << ':' << entry.direction << ' ' << exit.pathId << ':'
                      << exit.direction << '\n';
            targets.push_back(createPlaceTarget(elements[n].place.getStar(), entry, exit));
        } else {
            assert("ERROR:GoalPlanner: Element was invalid\n" && false);
        }
    }
}


std::shared_ptr<DecisionTarget> GoalPlanner::createInitialPathTarget(const hssh::GlobalPathSegment& segment,
                                                                     int nextPlaceId,
                                                                     const hssh::GlobalLocation& location)
{
    /*
     * For the initial path target, the robot is going to be moving in some direction along the global and local path.
     * If the target place of the current segment is not the next place along the route, then the robot needs to drive
     * to the beginning of the local path, rather than the end. After the initial path target, the robot will always be
     * driving to the end of the path.
     */

    return std::shared_ptr<DecisionTarget>(new LocalPathTarget(
      (location.pathState.targetPlaceId != nextPlaceId) ? LOCAL_TOPO_PATH_START : LOCAL_TOPO_PATH_END));
}


std::shared_ptr<DecisionTarget>
  GoalPlanner::createPlaceTarget(const hssh::LargeScaleStar& star, fragment_info_t entry, fragment_info_t exit)
{
    /*
     * The PlaceNeighborhoodTarget needs the SmallScaleStar and entry and exit local path fragments for the neighborhood
     * to be navigated. Need to find the correct global path fragment for the entry and exit of the global place, then
     * get the associate local path fragments to successfully do the downward conversion from the global topological to
     * local topological layers of the HSSH.
     */
    hssh::SmallScaleStar smallStar = star.toSmallScaleStar();
    hssh::local_path_fragment_t entryFragment;
    hssh::local_path_fragment_t exitFragment;

    std::vector<hssh::global_path_fragment_t> globalFragments = star.getPathFragments();

    bool haveEntry = false;
    bool haveExit = false;

    for (size_t n = 0; n < globalFragments.size(); ++n) {
        if ((globalFragments[n].pathId == entry.pathId) && (globalFragments[n].direction == entry.direction)) {
            entryFragment = smallStar.getFragmentWithId(globalFragments[n].fragmentId);
            haveEntry = true;
        }

        if ((globalFragments[n].pathId == exit.pathId) && (globalFragments[n].direction == exit.direction)) {
            exitFragment = smallStar.getFragmentWithId(globalFragments[n].fragmentId);
            haveExit = true;
        }
    }

    if (!haveEntry) {
        std::cout << "ERROR:GoalPlanner: Failed to find entry path fragment: " << entry.pathId << ':' << entry.direction
                  << '\n';
        std::cout << star << '\n';
    }

    if (!haveExit) {
        std::cout << "ERROR:GoalPlanner: Failed to find exit path fragment: " << exit.pathId << ':' << exit.direction
                  << '\n';
        std::cout << star << '\n';
    }

    assert(haveEntry && haveExit);

    return std::shared_ptr<DecisionTarget>(new PlaceNeighborhoodTarget(smallStar, entryFragment, exitFragment));
}


std::shared_ptr<DecisionTarget> GoalPlanner::createPathTarget(void)
{
    // When navigating along anything but the initial path, the robot will always go to the end of the LocalPath because
    // it is moving to the next place on the path, which is always the end in the local path frame of reference
    return std::shared_ptr<DecisionTarget>(new LocalPathTarget(LOCAL_TOPO_PATH_END));
}


void print_map(const hssh::TopologicalMap& map)
{
#ifdef DEBUG_MAP
    auto places = map.getPlaces();
    auto paths = map.getPaths();

    for (auto placeIt = places.begin(), placeEnd = places.end(); placeIt != placeEnd; ++placeIt) {
        std::cout << "Place " << placeIt->second.getId() << ':' << placeIt->second.getStar() << '\n';
    }

    for (auto pathIt = paths.begin(), pathEnd = paths.end(); pathIt != pathEnd; ++pathIt) {
        std::cout << pathIt->second << '\n';
    }
#endif
}


void print_path(const graph::Path<hssh::TopologicalVertex>& path)
{
#ifdef DEBUG_PATH
    if (path.getLength() > 0) {
        std::cout << "DEBUG:GoalPlanner: Path through graph:\n";

        auto vertices = path.getPath();

        for (auto vertexIt = vertices.begin(), vertexEnd = vertices.end(); vertexIt != vertexEnd; ++vertexIt) {
            std::cout << vertexIt->getId() << ' ';
        }
        std::cout << '\n';
    } else {
        std::cout << "DEBUG:GoalPlanner: Failed to find a path!\n";
    }
#endif
}


void print_route(const GoalRoute& route)
{
#ifdef DEBUG_ROUTE
    std::cout << "DEBUG:GoalPlanner: Route through map:\n";

    auto elements = route.getSequence();

    for (auto elementIt = elements.begin(), elementEnd = elements.end(); elementIt != elementEnd; ++elementIt) {
        if (elementIt->isPlace) {
            std::cout << 'P' << elementIt->place.getId() << ' ';
        } else if (elementIt->isPathSegment) {
            std::cout << 'S' << elementIt->segment.getPathId() << ' ';
        } else {
            assert("ERROR:Element must be either a place or a path segment" && false);
        }
    }
    std::cout << '\n';

#endif
}


void print_sequence(const DecisionTargetSequence& sequence)
{
#ifdef DEBUG_SEQUENCE
    const std::vector<std::shared_ptr<DecisionTarget>>& targets = sequence.getSequence();

    std::cout << "DEBUG:GoalPlanner: Local target sequence:\n";
    for (size_t n = 0; n < targets.size(); ++n) {
        std::cout << targets[n]->getDescription() << (n == targets.size() - 1 ? "\n" : " -> ");
    }
#endif
}

}   // namespace planner
}   // namespace vulcan
