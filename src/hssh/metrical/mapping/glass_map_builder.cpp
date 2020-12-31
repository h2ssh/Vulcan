/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map_builder.cpp
* \author   Collin Johnson
*
* Definition of GlassMapBuilder adapted from Paul's reference implementation of the glass mapping.
*/

#include "hssh/metrical/mapping/glass_map_builder.h"
#include "hssh/metrical/mapping/glass_map_utils.h"
#include "hssh/metrical/mapping/mapping_params.h"
#include "utils/cell_grid.h"
#include "utils/ray_tracing.h"
#include "utils/timestamp.h"
#include "laser/moving_laser_scan.h"
#include "core/pose.h"
#include "math/coordinates.h"
#include "utils/algorithm_ext.h"
#include <array>
#include <stack>
#include <set>
#include <cstdlib>

namespace vulcan
{
namespace hssh
{

GlassMapBuilder::GlassMapBuilder(const lpm_params_t& mapParams, 
                                 const pose_t& currentPose,
                                 const glass_map_builder_params_t& glassParams)
: map_(glassParams.maxLaserRange,
       glassParams.numAngleBins,
       glassParams.hitThreshold,
       glassParams.missThreshold,
       mapParams.width,
       mapParams.height,
       mapParams.scale,
       currentPose.toPoint())
, angleBinPlusPi_(glassParams.numAngleBins)
, kMaxLaserRange_(glassParams.maxLaserRange)
, kUse180Bins_(glassParams.canSeeCellsFromBothSides)
, kShouldFilterDynamic_(glassParams.shouldFilterDynamic)
, kShouldFilterReflections_(glassParams.shouldFilterReflections)
{
    assert(glassParams.numAngleBins > 0);

    map_.reset();   // clear out any previous glass information when initializing
    
    // Store the bin on the opposite side (+180 degrees)
    for(int n = 0; n < glassParams.numAngleBins; ++n)
    {
        angleBinPlusPi_[n] = (n + (glassParams.numAngleBins / 2)) % glassParams.numAngleBins;
    }

    double radPerBin = 2.0 * M_PI / glassParams.numAngleBins;
    kMinVisibleBins_ = std::max(lrint(glassParams.minVisibleOccupiedRange / radPerBin), 1l);
    kMinHighlyVisibleBins_ = std::max(lrint(glassParams.minHighlyVisibleRange / radPerBin), 1l);

    // If the bins are 180 bins, then double the number of hits are needed because there are two hits per angle
    if(kUse180Bins_)
    {
        kMinVisibleBins_ *= 2;
        kMinHighlyVisibleBins_ *= 2;
    }
    
    std::cout << "INFO: GlassMapBuilder: Min visible bins:" << kMinVisibleBins_ << " Min highly-visible bins:"
        << kMinHighlyVisibleBins_ << '\n';
}


laser::ReflectedLaserScan GlassMapBuilder::detectReflections(const laser::MovingLaserScan& scan)
{
    CellVector rayCells;
    std::vector<laser::reflected_ray_t> reflectedRays;
    const auto& flatMap = map_.flattenedMap();
    
    for(auto& ray : scan)
    {
        // Ignore invalid rays
        if(ray.range < 0.0f)
        {
            continue;
        }
        
        rayCells.clear();
        // Find the cells along the ray that will be checked for reflections
        float range = std::min(ray.range, kMaxLaserRange_); // only consider up to the maximum range
        Point<double> rangeLimitedEnd(ray.position.x + (range * std::cos(ray.angle)),
                                            ray.position.y + (range * std::sin(ray.angle)));
        utils::find_cells_along_line(Line<double>(utils::global_point_to_grid_point(ray.position, map_), 
                                                        utils::global_point_to_grid_point(rangeLimitedEnd, map_)),
                                    map_,
                                    std::back_inserter(rayCells));
        
        laser::reflected_ray_t reflected;
        reflected.origin = ray.position;
        reflected.range = ray.range;
        reflected.angle = ray.angle;
        reflected.isReflected = false;
        reflected.distToReflection = ray.range;
        
        int angleBin = map_.angleToBin(wrap_to_2pi(ray.angle));
        bool confirmedReflection = false;
        
        // Search along the cells the ray passes through. If a cell is occupied, then it is the potential source of a
        // reflection. Until a free cell is encountered, we keep update the reflection hypothesis because the walls in
        // the map aren't perfectly one cell thick and thus, we don't want false positives where it's just a two-cell
        // thick bit of wall.
        for(auto cell : rayCells)
        {
            // The two grids don't have the same reference frame, so cajole them together here.
            auto global = utils::grid_point_to_global_point(cell, map_);
            auto flatCell = utils::global_point_to_grid_cell_round(global, flatMap);
            
            if(!confirmedReflection     // once a reflection is confirmed, all hits thereafter are reflections
                && (flatMap.getCellType(flatCell) & kOccupiedOccGridCell)) // if we encounter an occupied cell,
                                                                           // it might be the source of a reflection
            {
                // A reflection has occurred the ray passes through an occupied cell outside its visible range
                auto angleRange = angle_bins_to_angle_range(cell.x, cell.y, map_);
                if(!angleRange.contains(ray.angle))
                {
                    reflected.isReflected = true;
                    reflected.distToReflection = distance_between_points(reflected.origin, global);
                }
            }
            // If there was a reflection detected and now have hit a free cell, then we definitely passed through
            // an object
            else if(reflected.isReflected)
            {
                // For all reflections, mark this angle bin as being the result of a reflection
                confirmedReflection = true;
                
                if(kShouldFilterReflections_)
                {
                    map_.reflectedCell(cell.x, cell.y, angleBin);
                }
            }
        }
        
        reflectedRays.push_back(reflected);
    }
    
    return laser::ReflectedLaserScan(scan.laserId(), scan.timestamp(), reflectedRays);
}


void GlassMapBuilder::reset(void)
{
    // No special resetting needed
}


void GlassMapBuilder::boundaryChanged(const math::Rectangle<float>& boundary)
{
    // Nothing to do here. changeBoundary in the MapBuilder base class is sufficient
}


void GlassMapBuilder::update(const map_update_data_t& data)
{
    // Keep the map centered around the robot
    map_.recenterActiveRegion(data.pose.toPoint());

    // Process the laser scan once the map is centered
    // Don't use the scan position for centering the map because the laser positions are separated by enough that the
    // glass map will thrash back and forth between blocks, significantly slowing the system down.
    processScanPoints(data.scan);
}


void GlassMapBuilder::rotate(float radians)
{
    std::cerr << "STUB! GlassMapBuilder::rotate\n";
}


void GlassMapBuilder::processScanPoints(const laser::MovingLaserScan& scan)
{
    findScanBins(scan);

    for(std::size_t n = 0; n < scanBins_.size(); ++n)
    {
        angle_bin_range_t binRange = findAngleBinRange(n);

        auto hitCells = markHitCellsInRange(binRange, scan);
        markFreeCellsInRange(binRange, scan, hitCells);

        // Jump to the end of the current bin range
        n = binRange.endIndex - 1;
        // now the ++n will increment to the start of the next bin
    }

    map_.flattenActiveRegion(kMinVisibleBins_);

    if(kShouldFilterDynamic_)
    {
        map_.filterDynamicObjectsFromActiveRegion(kMinHighlyVisibleBins_);
    }
}


void GlassMapBuilder::findScanBins(const laser::MovingLaserScan& scan)
{
    scanBins_.clear();

    for(std::size_t n = 0; n < scan.size(); ++n)
    {
        const auto& ray = scan[n];

        // Skip invalid scans
        if(ray.range < 0.0f)
        {
            continue;
        }

        ray_bin_t rayBin;
        rayBin.bearing = wrap_to_2pi(angle_to_point(ray.position, ray.endpoint));
        rayBin.angleBin = map_.angleToBin(rayBin.bearing);
        rayBin.scanIndex = n;

        scanBins_.push_back(rayBin);
    }
}


GlassMapBuilder::angle_bin_range_t GlassMapBuilder::findAngleBinRange(std::size_t beginIndex)
{
    const int angleBin = scanBins_[beginIndex].angleBin;

    angle_bin_range_t range;
    range.beginIndex = beginIndex;
    range.endIndex = scanBins_.size();      // for the last bin, won't trigger the break condition
                                            // so start with the end including the rest of the scan

    // Find all rays that fall into the same angle bin within the scan
    for(std::size_t n = beginIndex + 1; n < scanBins_.size(); ++n)
    {
        if(scanBins_[n].angleBin != angleBin)
        {
            range.endIndex = n;
            break;
        }
    }

    return range;
}


CellVector GlassMapBuilder::markHitCellsInRange(angle_bin_range_t range, const laser::MovingLaserScan& scan)
{
    CellVector hitCells;

    // Mark all hit cells in the angle bin range
    for(std::size_t n = range.beginIndex; n < range.endIndex; ++n)
    {
        const auto& ray = scan[scanBins_[n].scanIndex];

        // Mark the hit if it was within range
        auto hitCell = utils::global_point_to_grid_cell(ray.endpoint, map_);
        if(ray.range < kMaxLaserRange_)
        {
            map_.observedCell(hitCell.x, hitCell.y, scanBins_[n].angleBin, ray.intensity);
            if(kUse180Bins_)
            {
                map_.observedCell(hitCell.x, hitCell.y, angleBinPlusPi_[scanBins_[n].angleBin], ray.intensity);
            }

            hitCells.push_back(hitCell);
        }
    }

    return hitCells;
}


void GlassMapBuilder::markFreeCellsInRange(angle_bin_range_t range,
                                           const laser::MovingLaserScan& scan,
                                           const CellVector& hitCells)
{
    for(std::size_t n = range.beginIndex; n < range.endIndex; ++n)
    {
        markFreeCellsAlongRay(scanBins_[n], scan, hitCells);
    }
}


void GlassMapBuilder::markFreeCellsAlongRay(const ray_bin_t& ray,
                                            const laser::MovingLaserScan& scan,
                                            const CellVector& hitCells)
{
    // Trace the free up to the endpoint or the edge of the max range
    auto laserCell = utils::global_point_to_grid_point(scan[ray.scanIndex].position, map_);

    float range = std::min(scan[ray.scanIndex].range, kMaxLaserRange_);
    int numSteps = range * map_.cellsPerMeter();
    float x = 0.0f;
    float y = 0.0f;
    const float deltaX = std::cos(ray.bearing);
    const float deltaY = std::sin(ray.bearing);

    cell_t cell;

    for(int n = 0; n <= numSteps; ++n)
    {
        cell.x = laserCell.x + x;
        cell.y = laserCell.y + y;

        // Only mark misses if there wasn't a hit in the same cell/bin on this update
        if(!utils::contains(hitCells, cell))
        {
            map_.missedCell(cell.x, cell.y, ray.angleBin);

            if(kUse180Bins_)
            {
                map_.missedCell(cell.x, cell.y, angleBinPlusPi_[ray.angleBin]);
            }
        }

        x += deltaX;
        y += deltaY;
    }
}

} // namespace hssh
} // namespace vulcan
