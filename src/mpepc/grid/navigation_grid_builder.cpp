/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_grid_builder.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of NavigationGridBuilder.
*/

#include "mpepc/grid/navigation_grid_builder.h"
#include "mpepc/grid/navigation_grid.h"
#include "mpepc/grid/params.h"
#include "mpepc/cost/cost_map.h"
#include "core/point.h"
#include "core/pose.h"
#include "utils/algorithm_ext.h"
#include "utils/ray_tracing.h"
#include <functional>
#include <limits>
#include <queue>

namespace vulcan
{
namespace mpepc
{

namespace {
struct cell_node_t
{
    Point<int> cell;
    int32_t cost;

    cell_node_t(void) { }

    cell_node_t(Point<int> cell, int32_t cost)
    : cell(cell)
    , cost(cost)
    {
    }

    bool operator<(const cell_node_t& rhs) const
    {
        return cost < rhs.cost;
    }

    bool operator>(const cell_node_t& rhs) const
    {
        return cost > rhs.cost;
    }
};

using CellQueue = std::priority_queue<cell_node_t, std::vector<cell_node_t>, std::greater<cell_node_t>>;
}

struct NavigationGridBuilder::Impl
{
    std::vector<Point<int>> goalCells_;
    std::vector<Point<int>> targetCells_;

    int32_t maximumGridCost_ = 1000000000;
    int32_t maximumAllowedPropagatedCost_;

    CellQueue cellQueue_;
    navigation_grid_builder_params_t params_;

    bool buildGrid(const pose_t& start,
                   const pose_t& goal,
                   const std::vector<NavWaypoint>& intermediate,
                   const CostMap& costMap,
                   NavigationGrid& navGrid);

    void initializeGrid(const pose_t& goalPose,
                        const CostMap& costMap,
                        NavigationGrid& navGrid);

    void setGoalPose(const pose_t& goalPose, NavigationGrid& navGrid);
    bool setWaypoint(const Line<double>& boundary, const CostMap& costMap);
    void setRobotPose(const pose_t&  robotPose, const NavigationGrid& navGrid);
    void reassignGoalCellCosts(NavigationGrid& navGrid);

    // Propagate the wavefront from goal cells to target cells
    bool propagateWavefront(const CostMap& costMap, bool keepGoing, NavigationGrid& navGrid);

    void expandCellNode(const cell_node_t& node,
                        const CostMap& costMap,
                        NavigationGrid& navGrid);

    bool processCellNode(const cell_node_t& node,
                         NavigationGrid& navGrid);
};


NavigationGridBuilder::NavigationGridBuilder(const navigation_grid_builder_params_t& params)
: impl_(std::make_unique<NavigationGridBuilder::Impl>())
{
    impl_->params_ = params;
}


NavigationGridBuilder::~NavigationGridBuilder(void)
{
    // For std::unique_ptr
}


bool NavigationGridBuilder::buildGrid(const pose_t& start,
                                      const pose_t& goal,
                                      const CostMap& costMap,
                                      NavigationGrid& navGrid)
{
    std::vector<NavWaypoint> intermediate;
    return buildGrid(start, goal, intermediate, costMap, navGrid);
}


bool NavigationGridBuilder::buildGrid(const pose_t& start,
                                      const pose_t& goal,
                                      const std::vector<NavWaypoint>& intermediate,
                                      const CostMap& costMap,
                                      NavigationGrid& navGrid)
{
    return impl_->buildGrid(start, goal, intermediate, costMap, navGrid);
}


bool NavigationGridBuilder::Impl::buildGrid(const pose_t& start,
                                            const pose_t& goal,
                                            const std::vector<NavWaypoint>& intermediate,
                                            const CostMap& costMap,
                                            NavigationGrid& navGrid)
{
    // No need to rebuild the grid if the costmap and target pose are the same (this is to be used in static environment)
    if((navGrid.getId() == costMap.getId()) && (navGrid.getGoalPose() == goal))
    {
        return false;
    }

    initializeGrid(goal, costMap, navGrid);

    if(!navGrid.isPointInGrid(goal.toPoint()))
    {
        std::cerr << "WARNING: NavGridBuilder: Goal pose is not reachable in the grid: " << goal << '\n';
        return false;
    }

    if(!navGrid.isPointInGrid(start.toPoint()))
    {
        std::cerr << "WARNING: NavGridBuilder: Starting pose is not in the grid: " << start << '\n';
        return false;
    }

    // Start from the goal
    setGoalPose(goal, navGrid);

    // Plan to each of the intermediate waypoints
    for(auto wayIt = intermediate.rbegin(), wayEnd = intermediate.rend(); wayIt != wayEnd; ++wayIt)
    {
        if(setWaypoint(*wayIt, costMap))
        {
            // After reaching the waypoint, the next part of the search starts from where the previous search ended
            propagateWavefront(costMap, false, navGrid);
            goalCells_ = targetCells_;
            reassignGoalCellCosts(navGrid);
        }
    }

    // Perform the final search to reach the start pose
    setRobotPose(start, navGrid);
    // Allow the wavefront to propagate beyond the goal to make a better navigation function around the robot's
    // pose to give a proper progress term for drive-in-reverse behaviors.
    bool haveBuiltGrid = propagateWavefront(costMap, true, navGrid);
    return haveBuiltGrid;
}


void NavigationGridBuilder::Impl::initializeGrid(const pose_t& goalPose,
                                                 const CostMap& costMap,
                                                 NavigationGrid& navGrid)
{
    // set size and coordinates for the grid
    if((costMap.getWidthInCells() != navGrid.getWidthInCells())
       || (costMap.getHeightInCells() != navGrid.getHeightInCells()))
    {
        navGrid.setGridSizeInCells(costMap.getWidthInCells(), costMap.getHeightInCells());
    }

    navGrid.setBottomLeft(costMap.getBottomLeft());
    navGrid.setMetersPerCell(costMap.metersPerCell());

    // The timestamp corresponds to the most recent piece of information included in the NavigationGrid
    int64_t mostRecentTime = (goalPose.timestamp > costMap.getTimestamp()) ? goalPose.timestamp : costMap.getTimestamp();
    navGrid.setTimestamp(mostRecentTime);
    navGrid.setId(costMap.getId());

    // set initial value to some large number
    navGrid.resetCosts(maximumGridCost_);
    maximumAllowedPropagatedCost_ = maximumGridCost_;
}


void NavigationGridBuilder::Impl::setGoalPose(const pose_t& goalPose, NavigationGrid& navGrid)
{
    Point<float> goalPosition = goalPose.toPoint();
    Point<int> goalCell = navGrid.positionToCell(goalPosition);
    goalCells_.clear();
    goalCells_.push_back(goalCell);

    navGrid.setGoalPose(goalPose);
    navGrid.setGridCostNoCheck(goalCell, 0); // goal cell always has zero cost, and is the only cell that has zero cost.
}


bool NavigationGridBuilder::Impl::setWaypoint(const Line<double>& boundary, const CostMap& costMap)
{
    Line<double> cellEndpoints(utils::global_point_to_grid_cell(boundary.a, costMap),
                                     utils::global_point_to_grid_cell(boundary.b, costMap));

    // Extract cells along the boundary line
    std::vector<Point<int>> waypointCells;
    utils::find_cells_along_line(cellEndpoints, costMap, std::back_inserter(waypointCells));

    utils::erase_unique(waypointCells);

    // Cells outside the map can't be reached and must be assumed to be in collision for safety
    utils::erase_remove_if(waypointCells, [&costMap](auto cell) {
        return !costMap.isCellInGrid(cell);
    });

    // Any collision cells are unreachable, so must not be in the set of intermediate waypoint cells
    utils::erase_remove_if(waypointCells, [&costMap](auto cell) {
        return costMap(cell.x, cell.y) >= kMinCollisionCost;
    });

    if(waypointCells.empty())
    {
        return false;
    }
    else
    {
        targetCells_ = waypointCells;
        return true;
    }
}


void NavigationGridBuilder::Impl::setRobotPose(const pose_t& robotPose, const NavigationGrid& navGrid)
{
    targetCells_.clear();
    targetCells_.push_back(navGrid.positionToCell(robotPose.toPoint()));
}


void NavigationGridBuilder::Impl::reassignGoalCellCosts(NavigationGrid& navGrid)
{
    // Reassign the cost at this waypoint to be the maximum cost amongst the boundary cells
    // Doing so will make the whole boundary equal cost to get to if no obstacles are present, which
    // is more inline with gateway-based topological navigation

    auto assignedGoals = goalCells_;
    utils::erase_remove_if(assignedGoals, [&](auto cell) {
        return navGrid(cell.x, cell.y) == maximumGridCost_;
    });

    auto maxCellIt = std::max_element(assignedGoals.begin(), assignedGoals.end(), [&navGrid](auto lhs, auto rhs) {
        return navGrid(lhs.x, lhs.y) < navGrid(rhs.x, rhs.y);
    });

    if(maxCellIt != assignedGoals.end())
    {
        auto maxCost = navGrid(maxCellIt->x, maxCellIt->y);

        for(auto cell : assignedGoals)
        {
            navGrid(cell.x, cell.y) = maxCost;
        }
    }
}


bool NavigationGridBuilder::Impl::propagateWavefront(const CostMap& costMap, bool keepGoing, NavigationGrid& navGrid)
{
    maximumAllowedPropagatedCost_ = maximumGridCost_;

    cell_node_t node;
    // The wavefront starts at the goal cells
    cellQueue_ = CellQueue();       // clear out the previous queue
    for(auto& cell : goalCells_)
    {
        cellQueue_.push(cell_node_t(cell, navGrid.getGridCostNoCheck(cell)));
    }

    auto remainingTargets = targetCells_;
    bool reachedTarget = false;

    // While there's a target to reach and the queue hasn't emptied out, then the wavefront should keep propagating
    while(!cellQueue_.empty() && !remainingTargets.empty())
    {
        node = cellQueue_.top();
        cellQueue_.pop();

        // if the queue has reached the target cell, set maximum allowed propagation for the cost
        if(utils::contains(targetCells_, node.cell))
        {
            // set cost margin of 5m (hard coded)
            int32_t costMargin = 5.0 * navGrid.cellsPerMeter() * kStraightCellDist;

            if(maximumGridCost_ - costMargin < node.cost)
            {
                maximumAllowedPropagatedCost_ = maximumGridCost_;
            }
            else
            {
                maximumAllowedPropagatedCost_ = std::min(maximumAllowedPropagatedCost_, node.cost + costMargin);
            }

            reachedTarget = true;

            // If the wavefront should propagate beyond the target cells, then the target nodes need to be expanded
            if(keepGoing)
            {
                expandCellNode(node, costMap, navGrid);
            }
            // If not going to keep going after all targets found, then erase the targets as they are reached
            else
            {
                utils::erase_remove(remainingTargets, node.cell);
            }
        }
        // if the cost is smaller then max, keep searching
        else if((node.cost < maximumAllowedPropagatedCost_) && (node.cost <= navGrid(node.cell.x, node.cell.y)))
        {
            expandCellNode(node, costMap, navGrid);
        }
    }

    return reachedTarget;
}


void NavigationGridBuilder::Impl::expandCellNode(const cell_node_t& node,
                                                 const CostMap& costMap,
                                                 NavigationGrid& navGrid)
{
    for(int y = -1; y <= 1; ++y) // checking cells around the node to expand (8-way wavefront)
    {
        for(int x = -1; x <= 1; ++x)
        {
            // Don't process the same cell
            if((x != 0) || (y != 0))
            {
                Point<int> candidateCell(node.cell.x+x, node.cell.y+y);

                // do not expand into obstacles
                if(costMap.getValue(candidateCell.x, candidateCell.y) < kMinCollisionCost)
                {
                    // compute the cost, where the cost = parent cost + move cost + occupancy cost
                    int32_t moveCost = ((x == 0) || (y == 0)) ? kStraightCellDist : kDiagCellDist;
                    int32_t gridCost = moveCost + costMap.getValue(candidateCell.x, candidateCell.y);

                    assert(moveCost > 0);
                    assert(costMap.getValue(candidateCell.x, candidateCell.y) >= 0);

                    // add parent cost while preventing overflow
                    if(maximumGridCost_ - gridCost < node.cost)
                    {
                        gridCost = maximumGridCost_;
                    }
                    else
                    {
                        gridCost += node.cost;
                    }

                    // process node according to the cost
                    cell_node_t nodeToAdd(candidateCell, gridCost);
                    if(processCellNode(nodeToAdd, navGrid))
                    {
                        cellQueue_.push(nodeToAdd);
                    }
                }
            }
        }
    }
}


bool NavigationGridBuilder::Impl::processCellNode(const cell_node_t& node, NavigationGrid& navGrid)
{
    // process cell node if it is within the grid and if the proposed cost
    // is less than the currently encoded value in the grid
    if(navGrid.isCellInGrid(node.cell))
    {
        if(node.cost < navGrid.getGridCostNoCheck(node.cell))
        {
            if(node.cost < 0)
            {
                std::cerr << "ERROR: Cost below zero at " << node.cell << " cost:" << node.cost << " Old cost:" << navGrid.getGridCostNoCheck(node.cell) << '\n';
                assert(node.cost >= 0);
            }
            navGrid.setGridCostNoCheck(node.cell, node.cost);
            return true;
        }
    }

    return false;
}

} // namespace mpepc
} // namespace vulcan
