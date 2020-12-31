/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm.h
* \author   Collin Johnson
*
* Declaration of the OccupancyGrid.
*/

#ifndef HSSH_UTILS_METRICAL_OCCUPANCY_GRID_H
#define HSSH_UTILS_METRICAL_OCCUPANCY_GRID_H

#include "utils/cell_grid.h"
#include <cereal/access.hpp>
#include <cstdint>

namespace vulcan
{
namespace hssh
{

using cell_type_t = uint8_t;

const cell_type_t kUnobservedOccGridCell        = 0x80;
const cell_type_t kUnknownOccGridCell           = 0x01;
const cell_type_t kFreeOccGridCell              = 0x02;
const cell_type_t kOccupiedOccGridCell          = 0x04;
const cell_type_t kDynamicOccGridCell           = 0x08;
const cell_type_t kHazardOccGridCell            = 0x10;
const cell_type_t kQuasiStaticOccGridCell       = 0x20;
const cell_type_t kLimitedVisibilityOccGridCell = 0x40;

const cell_type_t kObstacleOccGridCell = kOccupiedOccGridCell | kLimitedVisibilityOccGridCell | kQuasiStaticOccGridCell;
const cell_type_t kUnsafeOccGridCell = kOccupiedOccGridCell | kHazardOccGridCell | kLimitedVisibilityOccGridCell | kQuasiStaticOccGridCell;
const cell_type_t kPermanentOccGridCell = kHazardOccGridCell | kLimitedVisibilityOccGridCell;


using SubGrid = utils::CellGrid<cell_type_t>;

/**
* OccupancyGrid is an occupancy grid implementation that maintains a
* cost value for each square. The cost can be used to represent any number
* of concepts that require a discretized grid of the robot's surround.
*
* In addition to occupancy information, each cell in the OccupancyGrid
* stores the a classification for the cell. The classifications provide data on
* the longer term behavior of a cell.
*
* The OccupancyGrid exists in a local reference frame. The parameters used
* to describe the grid are the coordinate of the bottom left cell, the
* number of meters per cell, and the width and height of the grid in cells.
*
* The LPM can have its reference frame changed. When the reference frame changes, the coordinates
* and map are adjusted accordingly. The current reference frame has a monotonically increasing
* index. The transform from the previous reference frame to the current reference frame
* is can be accessed for updating the coordinates of representations that use the LPM's
* reference frame.
*
* All operations in the OccupancyGrid are in grid coordinates. Helper functions
* are provided to convert between global and grid coordinates.
*/
class OccupancyGrid
{
public:

    /**
    * Default constructor for OccupancyGrid.
    *
    * Create an empty grid.
    */
    OccupancyGrid(void) { }

    /**
    * Constructor to create a new OccupancyGrid.
    *
    * \param    gridWidth           Width of the grid in cells
    * \param    gridHeight          Height of the grid in cells
    * \param    gridScale           Meters per grid cell
    * \param    globalCenter        Global center of the map
    * \param    occupiedCellCost    Threshold cost for a cell to be considered occupied or dynamic
    * \param    freeCellCost        Threshold cost for a cell to be considered free
    */
    OccupancyGrid(std::size_t               widthInCells,
                  std::size_t               heightInCells,
                  float                     cellsToMeters,
                  const Point<float>& globalCenter,
                  uint8_t                   occupiedCellCost,
                  uint8_t                   freeCellCost);

    // Copy and assignment for value semantics
    OccupancyGrid(const OccupancyGrid& gridToCopy) = default;
    OccupancyGrid(OccupancyGrid&&      toMove)     = default;
    OccupancyGrid& operator=(const OccupancyGrid& rhs) = default;
    OccupancyGrid& operator=(OccupancyGrid&&      rhs) = default;

    std::size_t getWidthInCells(void) const  { return costGrid.getWidthInCells(); }
    float       getWidthInMeters(void) const { return costGrid.getWidthInMeters(); }

    std::size_t getHeightInCells(void) const  { return costGrid.getHeightInCells(); }
    float       getHeightInMeters(void) const { return costGrid.getHeightInMeters(); }

    // The boundary is the physical space in the global reference frame currently encompassed by the LPM
    math::Rectangle<float> getBoundary(void)     const { return costGrid.getBoundary(); }
    Point<float>     getGlobalCenter(void) const { return costGrid.getGlobalCenter(); }
    Point<float>     getBottomLeft(void)   const { return costGrid.getBottomLeft(); }

    uint8_t getMaxCellCost(void) const;
    float   metersPerCell(void) const { return costGrid.metersPerCell(); }
    float   cellsPerMeter(void) const { return costGrid.cellsPerMeter(); }

    void setGridSizeInCells(std::size_t width, std::size_t height);
    void setBottomLeft(const Point<float>& newBottom);
    void setMetersPerCell(float gridScale);

    // Methods for modifying position of the grid

    /**
    * changeGlobalCenter moves the global center of the grid to the specified
    * position. Changing the center will shift the grid the appropriate number
    * of cells.
    *
    * \param    newGlobalCenter     New global center for the grid
    * \param    initialCost         Cost to assign to the new values that will appear in the grid
    */
    void changeGlobalCenter(const Point<float>& newGlobalCenter, uint8_t initialCost);

    /**
    * changeBoundary changes the part of the world the LPM represents.
    *
    * \param    boundary            New boundary for the LPM
    */
    void changeBoundary(const math::Rectangle<float>& boundary);

    /**
    * rotate rotates the OccupancyGrid by the specified number of radians.
    *
    * \param    radians             Number of radians to rotate the grid
    */
    void rotate(float radians);

    /**
    * reset resets the value of all cells in the LPM. Cell costs are set to maxCost/2. Cell types are
    * set to unobserved.
    */
    void reset(void);

    // Cell mutators
    bool isCellInGrid(const Point<int>& cell) const { return costGrid.isCellInGrid(cell);                     }

    void    setCost       (const Point<int>& cell, uint8_t cost);
    void    setCostNoCheck(const Point<int>& cell, uint8_t cost);

    uint8_t updateCost       (const Point<int>& cell, int8_t change);
    uint8_t updateCostNoCheck(const Point<int>& cell, int8_t change);

    void    setTypeNoCheck(const Point<int>& cell, cell_type_t type) { typeGrid.setValueNoCheck(cell.x, cell.y, type); }
    void    setType       (const Point<int>& cell, cell_type_t type) { typeGrid.setValue(cell.x, cell.y, type);        }

    // Cell observers
    uint8_t getCost       (const Point<int>& cell) const { return costGrid.getValue(cell.x, cell.y); }
    uint8_t getCostNoCheck(const Point<int>& cell) const { return costGrid.getValueNoCheck(cell.x, cell.y); }
    uint8_t getCostNoCheck(int x, int y)                 const { return costGrid.getValueNoCheck(x, y);           }

    cell_type_t getCellType       (const Point<int>& cell) const { return typeGrid.getValue(cell.x, cell.y);        }
    cell_type_t getCellTypeNoCheck(const Point<int>& cell) const { return typeGrid.getValueNoCheck(cell.x, cell.y); }
    cell_type_t getCellType       (int x, int y)                 const { return typeGrid.getValue(x, y);                  }
    cell_type_t getCellTypeNoCheck(int x, int y)                 const { return typeGrid.getValueNoCheck(x, y);           }

private:

    // Quickies for updating the cell type
    cell_type_t setCellType(const Point<int>& cell, uint8_t cost);
    cell_type_t setCellTypeNoCheck(const Point<int>& cell, uint8_t cost);
    cell_type_t determineCellType(uint8_t cost, const Point<int>& cell);
    bool        hasOccupiedNeighbor(const Point<int>& cell);

    SubGrid costGrid;
    SubGrid typeGrid;

    uint8_t occupiedCellCostThreshold;
    uint8_t freeCellCostThreshold;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & costGrid;
        ar & typeGrid;
        ar & occupiedCellCostThreshold;
        ar & freeCellCostThreshold;
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_UTILS_METRICAL_OCCUPANCY_GRID_H
