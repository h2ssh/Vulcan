/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     goal_planner.h
 * \author   Collin Johnson
 *
 * Declaration of GoalPlanner.
 */

#ifndef PLANNER_GOAL_GOAL_PLANNER_H
#define PLANNER_GOAL_GOAL_PLANNER_H

#include "graph/path.h"
#include "hssh/global_topological/graph.h"
#include "planner/decision/decision_target_sequence.h"
#include "planner/goal/goal_route.h"

namespace vulcan
{
namespace hssh
{
class TopologicalMap;
}

namespace planner
{

class GoalTarget;
struct goal_debug_info_t;

/**
 * GoalPlanner plans a path from the current robot place in the map to some
 * target place. The planner converts the map into a graph. An A* search is run on the
 * graph to find the desired path. Once a path has been found, it is converted into a
 * GoalRoute and a DecisionTargetSequence. The GoalRoute is for higher-level
 * debugging and reasoning, while the target sequence will be executed by the local topo planner
 * for actually moving the robot along the planned route.
 */
class GoalPlanner
{
public:
    /**
     * Constructor for GoalPlanner.
     */
    GoalPlanner(void);

    /**
     * plan plans a route from the current position in the topological map to the target place.
     * If the place is in the map, the planning should always be successful because the map is a
     * connected graph, so a route must exist between any two places. That said, something could
     * break and cause the map to be invalid.
     *
     * \param    target          Target for which to find a route
     * \param    map             Map in which to search for a target
     * \param    info            Debug info for the planning process
     * \return   True if a route was found.
     */
    bool plan(const GoalTarget& target, const hssh::TopologicalMap& map, goal_debug_info_t& info);

    /**
     * getRoute retrieves the most recent route that was successfully planned.
     */
    const GoalRoute& getRoute(void) const { return route; }

    /**
     * getTargetSequence retrieves the target sequence to be executed in order to follow the planned route.
     */
    const DecisionTargetSequence& getTargetSequence(void) const { return sequence; }

private:
    // Fragment info provides the path and direction of a path to be navigated. From the fragment info the associated
    // global_path_fragment_t can be found in a GlobalPlace for creating PlaceNeighborhoodTargets.
    struct fragment_info_t
    {
        int pathId;
        hssh::path_direction_t direction;
    };

    void initializeGraph(const hssh::TopologicalMap& map);
    void convertPathToRoute(const graph::Path<hssh::TopologicalVertex>& path, const hssh::TopologicalMap& map);
    void convertRouteToSequence(const hssh::TopologicalMap& map);

    fragment_info_t getFragmentInfoFromSegment(const hssh::GlobalPathSegment& segment, int placeId, bool entryFragment);
    fragment_info_t getFragmentInfoFromLocation(const hssh::GlobalLocation& location);

    std::shared_ptr<DecisionTarget> createInitialTarget(const goal_route_element_t& firstElement,
                                                        const goal_route_element_t& secondElement,
                                                        const hssh::GlobalLocation& location);
    void createTargetsForRemainingElements(const std::vector<goal_route_element_t>& elements,
                                           std::vector<std::shared_ptr<DecisionTarget>>& targets);
    std::shared_ptr<DecisionTarget> createInitialPathTarget(const hssh::GlobalPathSegment& segment,
                                                            int nextPlaceId,
                                                            const hssh::GlobalLocation& location);
    std::shared_ptr<DecisionTarget>
      createPlaceTarget(const hssh::LargeScaleStar& star, fragment_info_t entry, fragment_info_t exit);
    std::shared_ptr<DecisionTarget> createPathTarget(void);

    hssh::TopologicalGraph graph;   // graph in which to plan
    bool haveGraph;
    uint32_t graphId;   // Id of the map for which the graph represents

    GoalRoute route;
    DecisionTargetSequence sequence;
    uint32_t routeId;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_GOAL_GOAL_PLANNER_H
