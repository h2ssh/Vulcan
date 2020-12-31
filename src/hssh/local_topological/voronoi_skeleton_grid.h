/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     voronoi_skeleton_grid.h
* \author   Collin Johnson
*
* Declaration of VoronoiSkeletonGrid.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_VORONOI_SKELETON_GRID_H
#define HSSH_LOCAL_TOPOLOGICAL_VORONOI_SKELETON_GRID_H

// #include "utils/tiled_cell_grid.h"
#include "hssh/types.h"
#include "utils/cell_grid.h"
#include "core/pose.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>

namespace vulcan
{
namespace hssh
{

using VoronoiDist = int32_t;
    
// SkeletonToSources are the source cells -- frontier or obstacle -- associated with a given skeleton cell
using SourceCells       = std::vector<cell_t>;
using SkeletonToSources = std::map<cell_t, SourceCells>;

// typedef utils::TiledCellGrid<uint16_t, 8> DistanceGrid;
// typedef utils::TiledCellGrid<uint8_t,  8> ClassificationGrid;

using DistanceGrid       = utils::CellGrid<VoronoiDist>;
using ClassificationGrid = utils::CellGrid<uint8_t>;

/**
* skeleton_cell_type_t defines the possible classifications for a given cell in a VoronoiSkeletonGrid.
* The overall type of a cell is a bitwise-or of various skeleton_cell_type_t, so a mask needs
* to be used to determine if a cell is of a given type.
*/
enum skeleton_cell_type_t
{
    SKELETON_CELL_UNKNOWN           = 0x01,
    SKELETON_CELL_FREE              = 0x02,
    SKELETON_CELL_OCCUPIED          = 0x04,
    SKELETON_CELL_FRONTIER          = 0x08,
    SKELETON_CELL_GATEWAY           = 0x10,
    SKELETON_CELL_SKELETON          = 0x20,
    SKELETON_CELL_REDUCED_SKELETON  = 0x40,
    SKELETON_CELL_TEMP              = 0x80    // a place holder that can be useful for a variety of processing algorithms
};

/**
* VoronoiSkeletonGrid is the metric representation of a place from which the symbolic representation
* of gateways, paths, and the small-scale star are extracted.
*
* The VoronoiSkeletonGrid contains two representations of space. One representation contains the distance
* to the nearest obstacle for each non-obstacle cell in the grid. The other representation is are
* classifications for each cell. A cell can have multiple classifications. The current
* classifications are:
*
*   - UNKNOWN  : the cell has not been observed long enough to be classified
*   - FREE     : the cell is in free space or contains a dynamic obstacle
*   - OCCUPIED : the cell contains a static obstacle
*   - FRONTIER : the cell is on the frontier of explored space, the boundary between free and unknown
*   - GATEWAY  : the cell is part of a gateway
*   - SKELETON : the cell is part of the skeleton/EVG
*   - REDUCED_SKELETON : the cell is part of the reduced skeleton/EVG
*/
class VoronoiSkeletonGrid
{
public:
    
    using CellConstIter            = std::vector<cell_t>::const_iterator;
    using SkeletonSourcesConstIter = SkeletonToSources::const_iterator;

    // Constructors

    /**
    * Default constructor for VoronoiSkeletonGrid.
    */
    VoronoiSkeletonGrid(void);

    /**
    * Constructor for VoronoiSkeletonGrid.
    */
    VoronoiSkeletonGrid(std::size_t width, std::size_t height, float scale);

    /**
    * Constructor for VoronoiSkeletonGrid.
    */
    VoronoiSkeletonGrid(const DistanceGrid& distanceGrid, const ClassificationGrid& classificationGrid);

    // Accessors
    int64_t       getTimestamp(void)       const { return timestamp; }
    int32_t       getReferenceIndex(void)  const { return frameIndex; }
    pose_t getFrameTransform(void)  const { return transformFromLastFrame; }
    VoronoiDist   getMaximumDistance(void) const { return maximumDistance; }

    Point<float> getGlobalCenter(void) const { return distanceGrid.getGlobalCenter(); }
    Point<float> getBottomLeft(void)   const { return distanceGrid.getBottomLeft(); }

    VoronoiDist getObstacleDistance(int x, int y) const { return distanceGrid.getValue(x, y); }
    uint8_t  getClassification  (int x, int y) const { return classificationGrid.getValue(x, y); }
    uint8_t  getClassification  (cell_t cell) const { return classificationGrid.getValue(cell.x, cell.y); }
    uint8_t  getClassificationNoCheck(int x, int y) const { return classificationGrid(x, y); }
    float    getMetricDistance  (int x, int y) const { return std::sqrt(distanceGrid.getValue(x, y)) * metersPerCell(); }
    SourceCells getSourceCells  (int x, int y) const;

    DistanceGrid&       getObstacleDistanceGrid(void) { return distanceGrid; }
    ClassificationGrid& getClassificationGrid(void)   { return classificationGrid; }

    const DistanceGrid&       getObstacleDistanceGrid(void) const { return distanceGrid; }
    const ClassificationGrid& getClassificationGrid(void) const   { return classificationGrid; }

    std::size_t getWidthInCells(void)          const { return distanceGrid.getWidthInCells(); }
    std::size_t getHeightInCells(void)         const { return distanceGrid.getHeightInCells(); }
    float       getWidthInMeters(void)  const { return distanceGrid.getWidthInMeters(); }
    float       getHeightInMeters(void) const { return distanceGrid.getHeightInMeters(); }
    float       metersPerCell(void)      const { return distanceGrid.metersPerCell(); }
    float       cellsPerMeter(void)      const { return distanceGrid.cellsPerMeter(); }

    // Access high-level Voronoi graph information
    const std::vector<cell_t>& getSkeletonCells(void)   const { return skeletonCells; }
    const std::vector<cell_t>& getFrontierCells(void)   const { return frontierCells; }
    const SkeletonToSources&   getSkeletonSources(void) const { return skeletonSources; }
    const std::vector<cell_t>& getJunctionsPoints(void) const { return junctions; }
    const std::vector<cell_t>& getDeadEnds(void)        const { return deadEnds; }
    const std::vector<cell_t>& getExitPoints(void)      const { return exitPoints; }
    
    /**
    * filterVoronoiInformation goes through the saved cells for exits, junctions, and dead ends
    * to eliminate any cells that don't have the desired classification.
    */
    void filterVoronoiInformation(uint8_t classificationMask);

    bool isCellInGrid(const cell_t& cell) const { return distanceGrid.isCellInGrid(cell); }
    bool isCellInGrid(int x, int y)       const { return distanceGrid.isCellInGrid(x, y); }

    // Mutators

    void setGridSize(std::size_t width, std::size_t height);
    void setScale(float scale);
    void setBottomLeft(const Point<float>& bottomLeft);

    void setObstacleDistance(int x, int y, VoronoiDist distance);
    void setClassification(int x, int y, uint8_t classification);
    void addClassification(int x, int y, skeleton_cell_type_t classification); // bitwise-or classification onto the cell classificatin
    void removeClassification(int x, int y, skeleton_cell_type_t classification); // keep everything but this classification 

    void setObstacleDistanceGrid(const DistanceGrid& grid)             { distanceGrid = grid; }
    void setClassificationGrid  (const ClassificationGrid& grid)       { classificationGrid = grid; }
    void setSkeletonAnchors     (const SkeletonToSources& anchors)    { skeletonSources = anchors; }
    void setJunctions           (const std::vector<cell_t>& junctions) { this->junctions = junctions; }

    void reset(VoronoiDist distance, uint8_t classification);

    void setTimestamp      (int64_t timestamp) { this->timestamp = timestamp; }
    void setReferenceIndex (int32_t index) { frameIndex = index; }
    void setFrameTransform (const pose_t& transform) { transformFromLastFrame = transform; }
    void setMaximumDistance(uint8_t maxDistance) { maximumDistance = maxDistance; }

    // Iterator access for the various stored types
    CellConstIter beginSkeletonCells(void) const { return skeletonCells.begin(); }
    CellConstIter endSkeletonCells(void)   const { return skeletonCells.end();   }
    
    CellConstIter beginFrontierCells(void) const { return frontierCells.begin(); }
    CellConstIter endFrontierCells(void)   const { return frontierCells.end();   }
    
    CellConstIter beginJunctions(void) const { return junctions.begin(); }
    CellConstIter endJunctions(void)   const { return junctions.end();   }
    
    CellConstIter beginExitPoints(void) const { return exitPoints.begin(); }
    CellConstIter endExitPoints(void)   const { return exitPoints.end();   }
    
    CellConstIter beginDeadEnds(void) const { return deadEnds.begin(); }
    CellConstIter endDeadEnds(void)   const { return deadEnds.end();   }
    
    SkeletonSourcesConstIter beginSkeletonSources(void) const { return skeletonSources.begin(); }
    SkeletonSourcesConstIter endSkeletonSources(void)   const { return skeletonSources.end();   }
    
    // Unsafe -- cell MUST be a skeleton cell!
    CellConstIter beginSourceCells(cell_t cell) const { return skeletonSources.at(cell).begin(); }
    CellConstIter endSourceCells(cell_t cell) const { return skeletonSources.at(cell).end(); }
    
private:

    friend class VoronoiSkeletonBuilder;
    friend class SkeletonBuilder;

    int64_t       timestamp;
    int32_t       frameIndex;
    pose_t transformFromLastFrame;
    VoronoiDist   maximumDistance;

    DistanceGrid       distanceGrid;
    ClassificationGrid classificationGrid;

    std::vector<cell_t> skeletonCells;
    std::vector<cell_t> frontierCells;
    SkeletonToSources   skeletonSources;
    std::vector<cell_t> junctions;
    std::vector<cell_t> deadEnds;
    std::vector<cell_t> exitPoints;
    
    // Serialization access
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & timestamp;
        ar & maximumDistance;
        ar & distanceGrid;
        ar & classificationGrid;
        ar & skeletonCells;
        ar & frontierCells;
        ar & skeletonSources;
        ar & junctions;
        ar & deadEnds;
        ar & exitPoints;
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(hssh::VoronoiSkeletonGrid, ("DEBUG_HSSH_VORONOI_SKELETON_GRAPH"))

#endif // HSSH_LOCAL_TOPOLOGICAL_VORONOI_SKELETON_GRID_H
