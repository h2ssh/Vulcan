/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_walls.cpp
* \author   Collin Johnson
* 
* Definition of GlassWall and associated predict_glass_walls function.
*/

#include "hssh/metrical/mapping/glass_walls.h"
#include "hssh/metrical/mapping/glass_map_utils.h"
#include "hssh/metrical/glass_map.h"
#include "core/laser_scan.h"
#include "laser/line_extraction.h"
#include "laser/line_extractor_params.h"
#include "math/regression.h"
#include "math/clustering.h"
#include "utils/timestamp.h"
#include <boost/optional.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <iomanip>

namespace vulcan
{
namespace hssh
{

template <class Compare>  // (cell, cell, glassMap) -> bool
CellVector bfs_glass_map(cell_t start, const GlassMap& map, CellSet& visited, Compare comp);
std::vector<GlassWall> expand_glass_wall(cell_t start, const GlassMap& map, CellSet& visited);
CellToIntMap split_into_angle_clusters(const CellVector& cells, const GlassMap& map);
std::vector<GlassWall> split_wall(const GlassWall& wall, const GlassMap& map);
std::vector<GlassWall> merge_walls(const std::vector<GlassWall>& walls, const GlassMap& map);
double distance_between_walls(const GlassWall& lhs, const GlassWall& rhs);


////////////////////////////  GlassWall implementation  ////////////////////////////////////

GlassWall::GlassWall(const CellVector& cells, const GlassMap& map)
: cells_(cells.size())
{
    // Convert the cells to the corresponding global positions
    std::transform(cells.begin(), cells.end(), cells_.begin(), [&map](cell_t cell) {
        return utils::grid_point_to_global_point(cell, map);
    });
    
    computeWall(map);
}


GlassWall::GlassWall(const std::vector<GlassWall>& walls, const GlassMap& map)
{
    for(auto& w : walls)
    {
        boost::push_back(cells_, boost::as_array(w.cells_));
    }
    
    computeWall(map);
}


void GlassWall::computeWall(const GlassMap& map)
{
    // The wall line can be fit to these grid points associated with the wall
    wall_ = math::total_least_squares(cells_.begin(), cells_.end());
    
    // Sum the angle bins for all cells in the wall to find the normal
    std::vector<int> sumBins(map.numAngleBins(), 0);
    for(auto global : cells_)
    {
        auto c = utils::global_point_to_grid_cell_round(global, map);
        auto binBegin = map.beginBin(c.x, c.y);
        for(int n = 0; n < map.numAngleBins(); ++n)
        {
            sumBins[n] = *(binBegin + n);
        }
    }
    
    normal_ = weighted_angle_sum(sumBins.begin(), sumBins.end());
    
    double totalError = 0.0;
    for(auto& c : cells_)
    {
        totalError += distance_to_line_segment(c, wall_);
    }
    error_ = totalError / cells_.size();
    
    // TODO: Compute the predicted wall.
    
    std::cout << "Wall with " << cells_.size() << " cells at " << wall_ << " Normal:" << normal_ << " Error:" 
        << error_ << '\n';
}


////////////////////////////  predict_glass_walls implementation  ///////////////////////////////////?

std::vector<GlassWall> predict_glass_walls(const GlassMap& map)
{
    int64_t computationStartTimeUs = utils::system_time_us();

    auto flatActiveRegion = active_region_in_flat_map(map);
    
    const auto& flatGrid = map.flattenedMap();
    CellSet visited;
    std::vector<GlassWall> walls;

    // Any cells deemed to be 
    for(int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y)
    {
        for(int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x)
        {
            if((flatGrid.getCellTypeNoCheck(x, y) & kOccupiedOccGridCell)
                && (visited.find(cell_t(x, y)) == visited.end()))
            {
                auto newWalls = expand_glass_wall(cell_t(x, y), map, visited);
                boost::push_back(walls, boost::as_array(newWalls));
            }
        }
    }
    
    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout << "Wall prediction time: " << computationTimeUs << "us\n";
    return walls;
}


template <class Compare>
CellVector bfs_glass_map(cell_t start, const GlassMap& map, CellSet& visited, Compare comp)
{
    auto activeRegion = map.activeRegionInCells();
    auto flatActiveRegion = active_region_in_flat_map(map);
    auto glassToFlatOffset = map.glassToFlatMapOffset();
    
    const auto& flatGrid = map.flattenedMap();
    CellVector glassCells;
    
    // Use an 8-way search
    const int xDeltas[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
    const int yDeltas[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
    
    std::deque<cell_t> searchQueue;
    searchQueue.push_back(start);
    visited.insert(start);
    
    cell_t debugCell(553, 429);
    
    while(!searchQueue.empty())
    {
        auto cell = searchQueue.front();
        searchQueue.pop_front();
        
        auto angleCell = cell - glassToFlatOffset;
        glassCells.push_back(angleCell);
        
        bool debug = debugCell == angleCell;
        if(debug)
        {
            std::cout << "Debugging BFS for " << debugCell << " filled bins:";
            
            int binIndex = 0;
            for(auto b : boost::make_iterator_range(map.beginBin(angleCell.x, angleCell.y),
                map.endBin(angleCell.x, angleCell.y)))
            {
                if(b > 0)
                {
                    std::cout << binIndex << ' ';
                }
                ++binIndex;
            }
            std::cout << "Offset:" << glassToFlatOffset << " Glass origin:" << map.getBottomLeft() << " Flat origin:"
                << flatGrid.getBottomLeft();
        }
        
        for(int n = 0; n < 8; ++n)
        {
            if(debug)
            {
                std::cout << '\n';
            }
            Point<int> adjacentCell(cell.x + xDeltas[n], cell.y + yDeltas[n]);
            auto adjacentAngleCell = adjacentCell - glassToFlatOffset;
            
            if(debug)
            {
                std::cout << "Adj:" << adjacentAngleCell << " Visited:" << (visited.find(adjacentCell) != visited.end());
            }
            
            if(debug)
            {
                std::cout << " bins:";
                int binIndex = 0;
                for(auto b : boost::make_iterator_range(map.beginBin(adjacentAngleCell.x, adjacentAngleCell.y),
                    map.endBin(adjacentAngleCell.x, adjacentAngleCell.y)))
                {
                    if(b > 0)
                    {
                        std::cout << binIndex << ' ';
                    }
                    ++binIndex;
                }
            }
            
            // Ignore the neighbor if it has been visited already
            if(visited.find(adjacentCell) != visited.end())
            {
                continue;
            }
            
            assert(is_cell_in_region(adjacentCell, flatActiveRegion) == is_cell_in_region(adjacentAngleCell, activeRegion));
            
            if(debug)
            {
                std::cout << " in-region:" << is_cell_in_region(adjacentCell, flatActiveRegion);
            }
            
            // Ensure the cell is in the active region of the map
            if(!is_cell_in_region(adjacentCell, flatActiveRegion))
            {
                continue;
            }
            
            if(debug)
            {
                std::cout << " occ:" << std::hex << (int)flatGrid.getCellTypeNoCheck(adjacentCell) << ' ' 
                    << (int)(~flatGrid.getCellTypeNoCheck(adjacentCell) & kOccupiedOccGridCell) << std::dec
                    << " cost:" << (int)flatGrid.getCostNoCheck(adjacentCell.x, adjacentCell.y);
            }
            
            // Skip anything that isn't occupied because it isn't a confirmed piece of glass
            if(~flatGrid.getCellTypeNoCheck(adjacentCell) & kOccupiedOccGridCell)
            {
                continue;
            }
            
            // If the cells are also adjacent across angles, then add the cell to the queue and mark as part of
            // the occupied portion of the environment.
            if(debug)
            {
                std::cout << " angle:" << comp(angleCell, adjacentAngleCell);
            }
            if(comp(angleCell, adjacentAngleCell))
            {
                searchQueue.push_back(adjacentCell);
                visited.insert(adjacentCell);
            }
        }
    }
    
    return glassCells;
}


std::vector<GlassWall> expand_glass_wall(cell_t start, const GlassMap& map, CellSet& visited)
{
    // Find all angle-adjacent cells to start cell
    auto cells = bfs_glass_map(start, map, visited, [&map](cell_t lhs, cell_t rhs) {
        return map.haveOverlappingAngles(lhs.x, lhs.y, rhs.x, rhs.y);
    });
    
    assert(!cells.empty());
    
//     std::vector<GlassWall> walls;
//     walls.push_back(GlassWall(cells, map));
//     return walls;
    
    // Split those cells into clusters based on angle bins
    auto clusters = split_into_angle_clusters(cells, map);
    
    // Expand each cluster to only adjacent cells to create the appropriate wall
    std::vector<GlassWall> walls;
    CellSet visitedClusters;
    auto offset = map.glassToFlatMapOffset();
    
    for(auto c : cells)
    {
        auto offsetCell = c + offset;
        
        if(visitedClusters.find(offsetCell) != visitedClusters.end())
        {
            continue;
        }
        
        auto wallCells = bfs_glass_map(offsetCell, map, visitedClusters, [&clusters](cell_t lhs, cell_t rhs) {
            return clusters[lhs] == clusters[rhs];
        });
        
        if(wallCells.size() > 2)
        {
            GlassWall wall(wallCells, map);
            if(wall.error() > 0.1)
            {
                std::cout << "Splitting wall error was too high!\n";
                boost::push_back(walls, split_wall(wall, map));
            }
            else
            {
                walls.push_back(std::move(wall));
            }
        }
    }
    
    std::sort(walls.begin(), walls.end(), [](const GlassWall& lhs, const GlassWall& rhs) {
        return length(lhs.wall()) > length(rhs.wall());
    });
    
    return merge_walls(walls, map);
}


CellToIntMap split_into_angle_clusters(const CellVector& cells, const GlassMap& map)
{
    // Create the normals
    std::vector<double> normals(cells.size());
    std::transform(cells.begin(), cells.end(), normals.begin(), [&map](cell_t c) {
        return wrap_to_2pi(glass_cell_normal(c.x, c.y, map));
    });

    // Cluster them to see what happens
    auto clusters = math::dbscan_1d_linear(normals.begin(), normals.end(), 10.0 * M_PI / 180.0, 25);
    
    CellToIntMap cellCluster;
    for(std::size_t n = 0; n < cells.size(); ++n)
    {
        cellCluster[cells[n]] = clusters.assignedCluster[n];
    }
    
    return cellCluster;
}


std::vector<GlassWall> split_wall(const GlassWall& wall, const GlassMap& map)
{
    CellToIntMap cellToLine;
    
    // Create a fake laser scan from the laser data to try and do a line fitting.
    Point<float> mean;
    for(auto c : wall)
    {
        mean.x += c.x;
        mean.y += c.y;
    }
    mean.x /= wall.size();
    mean.y /= wall.size();
    
    // Do a radial sort of the cells
    std::vector<Point<float>> sorted(wall.begin(), wall.end());
    std::sort(sorted.begin(), sorted.end(), [mean](Point<float> lhs, Point<float> rhs) {
        return angle_to_point(mean, lhs) < angle_to_point(mean, rhs);
    });
    
    polar_laser_scan_t scan;
    scan.numRanges = sorted.size();
    for(auto c : sorted)
    {
        scan.ranges.push_back(distance_between_points(mean, c));
    }
    cartesian_laser_scan_t cartScan;
    cartScan.numPoints = sorted.size();
    cartScan.scanPoints = sorted;
    
    laser::quick_split_params_t params;
    params.clusterDistance     = 0.5;
    params.maxDistanceFromLine = 0.05;
    params.minPoints           = 5;
    
    std::vector<GlassWall> splitWalls;
    auto extracted = laser::quick_split(scan, cartScan, params);
    CellVector splitCells;
    for(int index = 0, endIndex = static_cast<int>(extracted.lines.size()); index < endIndex; ++index)
    {
        splitCells.clear();
        for(std::size_t n = 0; n < sorted.size(); ++n)
        {
            if(extracted.indices[n] == index)
            {
                splitCells.push_back(utils::global_point_to_grid_cell_round(sorted[n], map));
            }
        }
        splitWalls.push_back(GlassWall(splitCells, map));
    }
    
    return splitWalls;
}


std::vector<GlassWall> merge_walls(const std::vector<GlassWall>& walls, const GlassMap& map)
{
    std::vector<GlassWall> mergedWalls(walls);
    std::vector<GlassWall> toMerge;
    for(std::size_t n = 0; n < mergedWalls.size(); ++n)
    {
        const GlassWall& outer = mergedWalls[n];
        toMerge.push_back(outer);
        for(std::size_t i = n + 1; i < mergedWalls.size(); ++i)
        {
            const GlassWall& inner = mergedWalls[i];
            toMerge.push_back(inner);
            GlassWall merged(toMerge, map);
            if((outer.error() > merged.error() || inner.error() > merged.error()) 
                && (distance_between_walls(outer, inner) < 0.1))
            {
                std::cout << "Orig: " << outer.wall() << " , " << inner.wall() << " Errors:" << outer.error() 
                    << ',' << inner.error() << " Merged:" << merged.wall() << " Error:" << merged.error() << '\n';
                mergedWalls[n] = merged;
                mergedWalls.erase(mergedWalls.begin() + i);
                --i;
                toMerge.clear();
                toMerge.push_back(merged);
            }
            else
            {
                toMerge.pop_back();
            }
        }
        toMerge.clear();
    }
    
    return mergedWalls;
}


double distance_between_walls(const GlassWall& lhs, const GlassWall& rhs)
{
    double lhsA = distance_to_line_segment(lhs.wall().a, rhs.wall());
    double lhsB = distance_to_line_segment(lhs.wall().b, rhs.wall());
    double rhsA = distance_to_line_segment(rhs.wall().a, lhs.wall());
    double rhsB = distance_to_line_segment(rhs.wall().b, lhs.wall());
    
    return std::min(lhsA,
                    std::min(lhsB,
                             std::min(rhsA, rhsB)));
}

} // namespace hssh
} // namespace vulcan
