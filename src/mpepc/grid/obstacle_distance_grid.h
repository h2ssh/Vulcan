/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     obstacle_distance_grid.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of ObstacleDistanceGrid, a grid map of distance-to-the-nearest-static-obstacle.
*/

#ifndef MPEPC_GRIDS_OBSTACLE_DISTANCE_GRID_H
#define MPEPC_GRIDS_OBSTACLE_DISTANCE_GRID_H

#include <mpepc/types.h>
#include <utils/cell_grid.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }

namespace mpepc
{

/**
* ObstacleDistanceGrid is a grid map of distance-to-the-nearest-obstacle,
* where the obstacles are static (or unobserved) cells from a local perceptual map.
* Use getObstacleDistanceInterp() method to get the stored minimum metric distance
* to static obstacle in the map from any metric point in the map. 
*/
class ObstacleDistanceGrid : public utils::CellGrid<int32_t>
{
public:

    /**
    * Constructor for ObstacleDistanceGrid
    */
    ObstacleDistanceGrid(void);
    
    // Methods for accessing the values in the grid
    float getObstacleDistance(const Point<float>& position) const; // returns obstacle distance at some metric position
    float getObstacleDistance(const Point<int>&   cell)     const; // returns obstacle distance at some grid cell location
    // maximum clearance found in the entire grid
    float getMaxObstacleDistance(void) const { return (maxGridDist_ * metersPerCell()) / kStraightCellDist; }
    
    // No check is an unsafe, but faster method for accessing a cell, as it avoids bounds checks
    int32_t getGridDistNoCheck(const Point<int>& cell) const { return getValueNoCheck(cell.x, cell.y); }
    
    // Methods for mutating the parameters
    void setGridDist       (const Point<int>& cell, int32_t gridDist);
    void setGridDistNoCheck(const Point<int>& cell, int32_t gridDist);
    
    // A method for checking if a certin position is within a freespace, with specified margin in meters
    bool isPositionInFreeSpace(const Point<float> position, float margin_m = 0.0) const;
    
    // Methods to convert metric position to grid cell indices and vice versa
    Point<int>   positionToCell(const Point<float>& position) const;
    Point<float> cellToPosition(const Point<int>&   cell)     const;
    
    // Methods for accessing other parameters
    int64_t getTimestamp  (void) const { return timestamp_; }
    int32_t getId         (void) const { return id_; }
    int32_t getMaxGridDist(void) const { return maxGridDist_; }
    
    void setTimestamp  (int64_t  timestamp) { timestamp_ = timestamp; }
    void setId         (int32_t  id) { id_ = id; }
    void setMaxGridDist(int32_t cost) { maxGridDist_ = cost; }

private:
    
    int64_t timestamp_;
    int32_t id_;
    int32_t maxGridDist_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void save(Archive& ar) const
    {
        ar (cereal::base_class<utils::CellGrid<int32_t>>(this),
            timestamp_,
            id_,
            maxGridDist_);
    }
    
    template <class Archive>
    void load(Archive& ar)
    {
        ar (cereal::base_class<utils::CellGrid<int32_t>>(this),
            timestamp_,
            id_,
            maxGridDist_);
    }
};

} // namespace mpepc
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(mpepc::ObstacleDistanceGrid, ("DEBUG_MPEPC_DISTANCE_GRID"))

#endif // MPEPC_GRIDS_OBSTACLE_DISTANCE_GRID_H
