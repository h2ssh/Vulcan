/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_grid.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of NavigationGrid, a grid map of likely cost of travel from each cell to a goal position.
*/

#ifndef MPEPC_GRIDS_NAVIGATION_GRID_H
#define MPEPC_GRIDS_NAVIGATION_GRID_H

#include <mpepc/types.h>
#include <core/pose.h>
#include <utils/cell_grid.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{

namespace mpepc
{

/**
* NavigationGrid is a grid where each cell represents the likely cost of traveling from the cell
* to the closest goal position. The cost of a cell is the sum of the cost of traveling to the cell
* from the neighboring low-cost cell plus the cost of occupying the cell itself. The occupation
* cost is related to the closeness to static obstacles as determined by the ObstacleDistanceGrid.
*
* With the current setting, the cost is equivalent to metric Euclidean distance if the path is
* far from obstacles.
*/
class NavigationGrid : public utils::CellGrid<int32_t>
{
public:

    /**
    * Constructor for NavigationGrid.
    */
    NavigationGrid(void);

    // Methods for accessing the values in the grid
    float getCostToGo(const Point<float>& position) const; // returns cost-to-go at some metric position
    float getCostToGo(const Point<int>&   cell)     const; // returns cost-to-go at some grid cell location
    // No check is an unsafe, but faster method for accessing a cell, as it avoids bounds checks
    float getGridCostNoCheck(const Point<int>& cell) const { return getValueNoCheck(cell.x, cell.y); }

    // Methods for mutating the parameters
    void setGridCost(const Point<int>& cell, int32_t gridCost);
    void setGridCostNoCheck(const Point<int>& cell, int32_t gridCost);

    // maximum cost-to-go found in the entire grid
    float getMaxCostToGo(void) const;
    // returns if the position queried is within the grid with added margin in meters
    bool isPointInGrid(Point<float> position, float margin_m = 0.0) const;

    pose_t getGoalPose(void) const { return goalPose_; }

    // Methods to convert metric position to grid cell indices and vice versa
    Point<int>   positionToCell(const Point<float>& position) const;
    Point<float> cellToPosition(const Point<int>&   cell)     const;

    // Methods for accessing other parameters
    int64_t getTimestamp      (void) const { return timestamp_; }
    int32_t getId             (void) const { return id_; }
    int32_t getMaxGridCostToGo(void) const { return maxGridCostToGo_; }

    void setTimestamp(int64_t timestamp) { timestamp_ = timestamp; };
    void setId(int32_t id) { id_ = id; };
    void setMaxGridCostToGo(int32_t cost) { maxGridCostToGo_ = cost; }
    void setGoalPose(const pose_t& goalPose) { goalPose_ = goalPose; }

    /**
    * resetCosts resets all costs associated with the grid.
    *
    * \param    cost            Cost to assign to all cells
    */
    void resetCosts(int32_t cost);

private:

    int64_t timestamp_;
    int32_t id_;
    int32_t maxGridCostToGo_;

    pose_t goalPose_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void save(Archive& ar) const
    {
        ar (cereal::base_class<utils::CellGrid<int32_t>>(this),
            timestamp_,
            id_,
            maxGridCostToGo_,
            goalPose_);
    }

    template <class Archive>
    void load(Archive& ar)
    {
        ar (cereal::base_class<utils::CellGrid<int32_t>>(this),
            timestamp_,
            id_,
            maxGridCostToGo_,
            goalPose_);
    }
};

} // namespace mpepc
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(mpepc::NavigationGrid, ("DEBUG_MPEPC_NAVIGATION_GRID"))

#endif // MPEPC_GRIDS_NAVIGATION_GRID_H
