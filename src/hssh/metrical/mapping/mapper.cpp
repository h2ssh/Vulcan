/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mapper.cpp
* \author   Collin Johnson
*
* Definition of Mapper.
*/

#include "hssh/metrical/mapping/mapper.h"
#include "hssh/metrical/mapping/mapping_params.h"
#include "hssh/metrical/data.h"
#include "laser/moving_laser_scan.h"
#include "core/pose_distribution.h"
#include "core/velocity.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

// updateRadius is additional area by which to extend the update region when considering the union
math::Rectangle<float> rectangle_union(const math::Rectangle<float>& lpm,
                                       const math::Rectangle<float>& update,
                                       float updateRadius);
// bool is_glass_cell(std::size_t x, std::size_t y, const std::vector<const BeliefGrid*>& maps);



Mapper::Mapper(const mapper_params_t& params)
: rasterizer_(params.lpmParams.scale, params.rasterizerParams)
, lpmBuilder_(params.lpmParams, pose_t(0, 0, 0))
, lpm_(params.lpmParams, Point<float>(0.0f, 0.0f))
, boundary_(lpmBuilder_.getMap().getBoundary())
, mode_(MapperMode::expanding)
, shouldUpdateMap_(true)
, shouldBuildGlass_(params.shouldBuildGlassMap)
, tightTurnStartTime_(0)
, lastMergeTime_(0)
, params_(params)
{
    if(shouldBuildGlass_)
    {
        initializeGlassMapping();
    }
}


void Mapper::resetMap(const pose_t& pose)
{
    initializedLasers_.clear();
    lpmBuilder_.resetMap();
    if(glassBuilder_)
    {
        glassBuilder_->resetMap();
    }

    lpm_ = lpmBuilder_.getMap();

    math::Rectangle<float> newBoundary(Point<float>(pose.x - 5.0f, pose.y - 5.0f),
                                       Point<float>(pose.x + 5.0f, pose.y + 5.0f));

    updateBoundaryIfNeeded(newBoundary);

    mode_         = MapperMode::expanding;
    previousPose_ = pose;
}


void Mapper::setMap(const LocalPerceptualMap& lpm)
{
    lpmBuilder_.setMap(lpm);
    if(glassBuilder_)
    {
        glassBuilder_->resetMap();
        glassBuilder_->changeMapBoundary(lpm.getBoundary());
    }

    lpm_ = lpm;
    boundary_ = lpm.getBoundary();
    mode_     = MapperMode::expanding;
}


bool Mapper::updateMap(const pose_distribution_t& poseDistribution, const metric_slam_data_t& data)
{
    bool shouldUpdate = shouldMap(poseDistribution, data.velocity, data.laser.laserId);
    pose_t pose{poseDistribution};

    // Always update the lasers, even if not mapping so they show up in the correct spot in the visualization
    mappingLaser_ = params_.shouldUseMovingLaser ? laser::MovingLaserScan(data.laser, previousPose_, pose) :
        laser::MovingLaserScan(data.laser, pose, pose);

    if(shouldUpdate)
    {
        bool isInitialUpdate = initializeMapUpdate(pose, mappingLaser_);

        const map_update_data_t updateData(data.laser.timestamp,
                                           isInitialUpdate,
                                           pose,
                                           mappingLaser_,
                                           rasterizer_.getRaster());
        buildMaps(updateData);
    }
    else
    {
        std::cout << "\n\nNOT MAPPING\n\n";
    }

    // Update after, so if map was updated, that information is taken into account with the reflections
    if(glassBuilder_)
    {
        reflectedLaser_ = glassBuilder_->detectReflections(mappingLaser_);
    }

    previousPose_ = pose;
    lpm_.setTimestamp(data.endTime);

    return shouldUpdate;
}


void Mapper::updateMapTime(const metric_slam_data_t& sensorData)
{
    lpm_.setTimestamp(sensorData.endTime);
}


void Mapper::rotateMap(float radians)
{
    lpmBuilder_.rotateMap(radians);
}


void Mapper::truncateMap(const math::Rectangle<float>& boundary)
{
    updateBoundaryIfNeeded(boundary);
}


void Mapper::changeReferenceFrame(const pose_t& referenceFrame)
{
    lpm_.changeReferenceFrame(referenceFrame);
    lpmBuilder_.getMap().changeReferenceFrame(referenceFrame);

    // TODO: Need to generalized this so I can support changing the glass map too. Right now, I'll just reset the map
    if(glassBuilder_)
    {
        glassBuilder_->resetMap();
        glassBuilder_->changeMapBoundary(lpmBuilder_.getMap().getBoundary());
    }

    std::cout<<"Mapper: New reference frame:"<<referenceFrame<<" BL:"<<lpmBuilder_.getMap().getBottomLeft()<<'\n';
}


void Mapper::shouldBuildGlassMap(bool shouldBuild)
{
    shouldBuildGlass_ = shouldBuild;

    if(shouldBuildGlass_)
    {
        initializeGlassMapping();
    }
}


void Mapper::initializeGlassMapping(void)
{
    if(!glassBuilder_)
    {
        glassBuilder_.reset(new GlassMapBuilder(params_.lpmParams, pose_t(0, 0, 0), params_.glassBuilderParams));
    }

    glassBuilder_->changeMapBoundary(boundary_);
}


bool Mapper::shouldMap(const pose_distribution_t& pose, const velocity_t& velocity, int laserIndex)
{
    /*
    * The mapper will map if this is the first scan from a particular laser, or
    * if the localization is sufficiently good. And the robot isn't turning too fast.
    */

    if(!shouldUpdateMap_)
    {
        return false;
    }

    const int X_STD_INDEX     = 0;
    const int Y_STD_INDEX     = 1;
    const int THETA_STD_INDEX = 2;

    bool isLaserInitialized = initializedLasers_.find(laserIndex) != initializedLasers_.end();

    Matrix stddev = arma::sqrt(pose.uncertainty.getCovariance());

    bool isWellLocalized = (stddev(X_STD_INDEX,X_STD_INDEX)         < params_.maxMappingPositionStdDev) &&
                           (stddev(Y_STD_INDEX,Y_STD_INDEX)         < params_.maxMappingPositionStdDev) &&
                           (stddev(THETA_STD_INDEX,THETA_STD_INDEX) < params_.maxMappingOrientationStdDev);

    bool isTurningWide   = (std::abs(velocity.angular) < 0.1) || std::abs(velocity.linear / velocity.angular) >
        params_.minRadiusOfCurvature;

    if(!isTurningWide)
    {
        tightTurnStartTime_ = pose.timestamp;
    }

    if(!isWellLocalized)
    {
        std::cout << "NOT WELL-LOCALIZED!\n";
    }

    isTurningWide = (pose.timestamp - tightTurnStartTime_) > 500000;

    if(!isWellLocalized || !isTurningWide)
    {
        std::cout << "Lin:" << velocity.linear << " Ang:" << velocity.angular << " Curvature:" << velocity.linear / velocity.angular << '\n'
            << std::boolalpha << "Well localized? " << isWellLocalized << " Is turning wide? " << isTurningWide << '\n';
    }

    return !isLaserInitialized || (isWellLocalized && isTurningWide);
}


bool Mapper::initializeMapUpdate(const pose_t& currentPose, const laser::MovingLaserScan& scan)
{
    // When doing the update, check to see if the robot is actually in the LPM. If the robot isn't in the LPM for
    // some reason, then switch to EXPAND mode_, so the LPM will grow around the current location.
    if(!boundary_.contains(currentPose.toPoint()))
    {
        mode_ = MapperMode::expanding;
        initializedLasers_.clear();
    }

    bool isInitialUpdate = initializedLasers_.find(scan.laserId()) == initializedLasers_.end();

    rasterizer_.rasterizeScan(scan, boundary_.bottomLeft, isInitialUpdate);

    if(mode_ == MapperMode::expanding)
    {
        updateBoundaryIfNeeded(calculateExpandingBoundary(rasterizer_.getRaster(), currentPose));
    }

    initializedLasers_.insert(scan.laserId());

    if(isInitialUpdate)
    {
        previousPose_ = currentPose;
    }

    return isInitialUpdate;
}


void Mapper::updateBoundaryIfNeeded(const math::Rectangle<float>& newBoundary)
{
    if(newBoundary != boundary_)
    {
        lpm_.changeBoundary(newBoundary);
        lpmBuilder_.changeMapBoundary(newBoundary);

        if(shouldBuildGlass_)
        {
            glassBuilder_->changeMapBoundary(newBoundary);
        }

        // Use the LPM boundary_ because adjusting a Grid boundary_ will actually snap it to the nearest
        // grid cell, thereby ensuring the cells don't drift.
        boundary_ = lpmBuilder_.getMap().getBoundary();
    }
}


math::Rectangle<float> Mapper::calculateExpandingBoundary(const laser_scan_raster_t& raster, const pose_t& pose)
{
    // If the boundary_ has expanded, then reshape the LPM accordingly. Once the LPM is the correct, then the static
    // update can be applied to the grid
    math::Rectangle<float> mergedBoundary = rectangle_union(boundary_, raster.getBoundary(), params_.shiftRadius);

    if(mergedBoundary.width() > params_.maxMapWidthMeters)
    {
        mergedBoundary.bottomLeft.x  = std::max(mergedBoundary.bottomLeft.x, pose.x - params_.maxMapWidthMeters/2.0f);
        mergedBoundary.topRight.x    = std::min(mergedBoundary.topRight.x, pose.x + params_.maxMapWidthMeters/2.0f);
        mergedBoundary.bottomRight.x = mergedBoundary.topRight.x;
        mergedBoundary.topLeft.x     = mergedBoundary.bottomLeft.x;
    }

    if(mergedBoundary.height() > params_.maxMapHeightMeters)
    {
        mergedBoundary.bottomLeft.y  = std::max(mergedBoundary.bottomLeft.y, pose.y - params_.maxMapHeightMeters/2.0f);
        mergedBoundary.topRight.y    = std::min(mergedBoundary.topRight.y, pose.y + params_.maxMapHeightMeters/2.0f);
        mergedBoundary.bottomRight.y = mergedBoundary.bottomLeft.y;
        mergedBoundary.topLeft.y     = mergedBoundary.topRight.y;
    }

    return mergedBoundary;
}


void Mapper::buildMaps(const map_update_data_t& data)
{
    lpmBuilder_.updateMap(data);
    lpm_ = lpmBuilder_.getMap();

    if(shouldBuildGlass_)
    {
        assert(glassBuilder_);
        glassBuilder_->updateMap(data);
    }

    // If the glassBuilder_ exists, then glass mapping was performed at some point, so add to current LPM
    if(glassBuilder_)// && (utils::system_time_us() - lastMergeTime_ > 100000))
    {
        addGlassMapToLPM(glassBuilder_->getFlattenedMap());
        lastMergeTime_ = utils::system_time_us();
    }
}


void Mapper::addGlassMapToLPM(const LocalPerceptualMap& glassMap)
{
    assert(glassMap.getWidthInCells() >= lpm_.getWidthInCells());
    assert(glassMap.getHeightInCells() >= lpm_.getHeightInCells());
    assert(glassMap.getBottomLeft() == lpm_.getBottomLeft());

    utils::boundary_intersection_t intersection(lpm_.getBoundary(),
                                                glassMap.getBoundary(),
                                                lpm_.cellsPerMeter(),
                                                std::make_pair(lpm_.getWidthInCells(), lpm_.getHeightInCells()),
                                                std::make_pair(glassMap.getWidthInCells(), glassMap.getHeightInCells()));

    Point<int> lpmCell(intersection.gridStartCell);
    Point<int> glassCell(intersection.updateStartCell);

    for(std::size_t y = 0; y < intersection.updateHeight; ++y, ++lpmCell.y, ++glassCell.y)
    {
        lpmCell.x   = intersection.gridStartCell.x;
        glassCell.x = intersection.updateStartCell.x;

        for(std::size_t x = 0; x < intersection.updateWidth; ++x, ++lpmCell.x, ++glassCell.x)
        {
            cell_type_t type = lpm_.getCellTypeNoCheck(lpmCell);

            // If the cell is glass, mark it as limited visibility and not dynamic
            if((~type & kOccupiedOccGridCell) && (glassMap.getCellTypeNoCheck(glassCell) & kOccupiedOccGridCell))
            {
                type &= ~kDynamicOccGridCell;
                type |= kLimitedVisibilityOccGridCell | kOccupiedOccGridCell;

                lpm_.setCostNoCheck(lpmCell, glassMap.getCostNoCheck(glassCell));
                lpm_.setTypeNoCheck(lpmCell, type);
            }
        }
    }
}


math::Rectangle<float> rectangle_union(const math::Rectangle<float>& lpm,
                                       const math::Rectangle<float>& update,
                                       float                         updateRadius)
{
    // Default to having the same boundary_. Only extend it if the update falls outside the current LPM
    // Add the updateRadius onto the extended dimensions, so extending again occurs less.
    float topX    = lpm.topRight.x;
    float topY    = lpm.topRight.y;
    float bottomX = lpm.bottomLeft.x;
    float bottomY = lpm.bottomLeft.y;

    if(topX < update.topRight.x)
    {
        topX = update.topRight.x + updateRadius;
    }

    if(topY < update.topRight.y)
    {
        topY = update.topRight.y + updateRadius;
    }

    if(bottomX > update.bottomLeft.x)
    {
        bottomX = update.bottomLeft.x - updateRadius;
    }

    if(bottomY > update.bottomLeft.y)
    {
        bottomY = update.bottomLeft.y - updateRadius;
    }

    return math::Rectangle<float>(Point<float>(bottomX, bottomY), Point<float>(topX, topY));
}


// bool is_glass_cell(std::size_t x, std::size_t y, const std::vector<const BeliefGrid*>& maps)
// {
//     return std::find_if(maps.begin(),
//                         maps.end(),
//                         [x, y](const BeliefGrid* map)
//                         {
//                             return map->getValueNoCheck(x, y) > 3;
//                         }) != maps.end();
// }

} // namespace hssh
} // namespace vulcan
