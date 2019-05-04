/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     obstacle_distance_grid_builder.h
* \author   Collin Johnson and Jong Jin Park
* 
* Definition of ObstacleDistanceGridBuilder.
*/

#include <mpepc/grid/obstacle_distance_grid_builder.h>
#include <mpepc/grid/obstacle_distance_grid.h>
#include <mpepc/grid/params.h>
#include <hssh/local_metric/lpm.h>
#include <core/point.h>
#include <queue>

namespace vulcan
{
namespace mpepc
{
    
struct cell_node_t
{
    Point<int> cell;
    int32_t gridDist;
    
    cell_node_t(void) { }
    
    cell_node_t(Point<int> cell, int32_t gridDist)
    : cell    (cell)
    , gridDist(gridDist)
    {
    }

    bool operator<(const cell_node_t& rhs) const { return gridDist > rhs.gridDist; }
};


/////   Impl   /////
struct ObstacleDistanceGridBuilder::Impl
{
    int32_t defaultGridDist_;
    std::priority_queue<cell_node_t> cellQueue_;
    obstacle_distance_grid_builder_params_t params_;
    
    
    bool buildGrid(const hssh::LocalPerceptualMap& lpm, ObstacleDistanceGrid& distGrid)
    {
        // No need to build the cost map if it is the same as the lpm
        if(lpm.getId() == distGrid.getId())
        {
            return false;
        }
        
        initializeGrid(lpm, distGrid);
        setObstacles(lpm, distGrid);
        computeDistances(distGrid);
        
        if(params_.shouldGrowObstacle)
        {
            reduceDistanceToObstacles(params_.growObstacleRadius, distGrid);
        }
        
        return true;
    }
    

    void initializeGrid(const hssh::LocalPerceptualMap& lpm, ObstacleDistanceGrid& distGrid)
    {
        // set size and coordinates for the grid
        if((lpm.getWidthInCells()  != distGrid.getWidthInCells())
            || (lpm.getHeightInCells() != distGrid.getHeightInCells()))
        {
            distGrid.setGridSizeInCells(lpm.getWidthInCells(), lpm.getHeightInCells());
        }
        distGrid.setBottomLeft(lpm.getBottomLeft());
        distGrid.setMetersPerCell(lpm.metersPerCell());
        
        // set timestamp and id
        distGrid.setTimestamp(lpm.getTimestamp());
        distGrid.setId(lpm.getId());
        
        defaultGridDist_ = params_.maxObstacleDistance * distGrid.cellsPerMeter() * kStraightCellDist;
    }
    
    
    void setObstacles(const hssh::LocalPerceptualMap& lpm, ObstacleDistanceGrid& distGrid)
    {
        Point<int> cell;
    
        for(cell.y = 0; cell.y < static_cast<int>(lpm.getHeightInCells()); ++cell.y)
        {
            for(cell.x = 0; cell.x < static_cast<int>(lpm.getWidthInCells()); ++cell.x)
            {
                hssh::cell_type_t type = lpm.getCellTypeNoCheck(cell);
                
                if(type & hssh::kUnsafeOccGridCell)
                {
                    distGrid.setGridDistNoCheck(cell, 0);  // all unsafe (i.e. occupied) cells gets marked with distance 0
                    cellQueue_.push(cell_node_t(cell, 0));
                }
                else if(params_.isUnobservedObstacle && (type == hssh::kUnobservedOccGridCell))
                {
                    distGrid.setGridDistNoCheck(cell, 0);
                }
                else
                {
                    distGrid.setGridDistNoCheck(cell, defaultGridDist_); // all safe cells gets initialized with the predetermined (large) distance value
                }
            }
        }
    }
    
    
    void computeDistances(ObstacleDistanceGrid& distGrid)
    {
        cell_node_t node;
    
        while(!cellQueue_.empty())
        {
            node = cellQueue_.top();
            expandCellNode(node, distGrid);
            cellQueue_.pop();
        }
    }
    
    
    void expandCellNode(const cell_node_t& node, ObstacleDistanceGrid& distGrid)
    {
        for(int i = -1; i <= 1; i++) // checking cells around the node to expand (8-way wavefront)
        {
            for(int j = -1; j <= 1; j++)
            {
                if(i == 0 && j == 0)
                {
                    // do nothing to itself
                }
                else
                {
                    Point<int> candidateCell(node.cell.x+i, node.cell.y+j);
                    
                    int gridDist;
                    if(i != j) // for horizontally and vertically adjacent cells
                    {
                        gridDist = node.gridDist + kStraightCellDist;
                    }
                    else // for diagnally adjacent cells
                    {
                        gridDist = node.gridDist + kDiagCellDist;
                    }
                    
                    cell_node_t nodeToAdd(candidateCell, gridDist);
                    if(processCellNode(nodeToAdd, distGrid))
                    {
                        cellQueue_.push(nodeToAdd);
                    }
                }
            }
        }
    }
    
    
    bool processCellNode(const cell_node_t& node, ObstacleDistanceGrid& distGrid)
    {
        // process cell node if it is within the grid and if the proposed distance
        // is less than the currently encoded value in the grid    
        if(distGrid.isCellInGrid(node.cell))
        {
            if(node.gridDist < distGrid.getGridDistNoCheck(node.cell))
            {
                distGrid.setGridDistNoCheck(node.cell, node.gridDist);
                return true;
            }
        }
        
        return false;
    }
    
    
    // forced under-estimation of the distance to static obstacles (i.e. growing walls);
    void reduceDistanceToObstacles(float radius, ObstacleDistanceGrid& distGrid)
    {
        int gridDistToReduce = radius * distGrid.cellsPerMeter() * kStraightCellDist;
    
        for(std::size_t j = 0; j < distGrid.getHeightInCells(); ++j)
        {
            for(std::size_t i = 0; i < distGrid.getWidthInCells(); ++i)
            {
                Point<int> cell(i,j);
                int originalGridDist = distGrid.getGridDistNoCheck(cell);
                int reducedGridDist  = (originalGridDist > gridDistToReduce) ? (originalGridDist - gridDistToReduce) : 0;
                
                distGrid.setGridDistNoCheck(cell, reducedGridDist);
            }
        }
    }
};


ObstacleDistanceGridBuilder::ObstacleDistanceGridBuilder(const obstacle_distance_grid_builder_params_t& params)
: impl_(std::make_unique<Impl>())
{
    impl_->defaultGridDist_ = 0;
    impl_->params_ = params;
}


ObstacleDistanceGridBuilder::~ObstacleDistanceGridBuilder(void)
{
    // For std::unique_ptr
}


bool ObstacleDistanceGridBuilder::buildGrid(const hssh::LocalPerceptualMap& lpm, ObstacleDistanceGrid& distGrid)
{
    return impl_->buildGrid(lpm, distGrid);
}


} // namespace mpepc
} // namespace vulcan
