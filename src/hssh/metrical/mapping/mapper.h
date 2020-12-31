/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     mapper.h
 * \author   Collin Johnson
 *
 * Declaration of Mapper, which is responsible for taking a pose and sensors values
 * to build the LPM.
 */

#ifndef HSSH_METRICAL_MAPPING_MAPPER_H
#define HSSH_METRICAL_MAPPING_MAPPER_H

#include "hssh/metrical/mapping/glass_map_builder.h"
#include "hssh/metrical/mapping/laser_scan_rasterizer.h"
#include "hssh/metrical/mapping/lpm_builder.h"
#include "laser/moving_laser_scan.h"
#include "laser/reflected_laser_scan.h"
#include <map>
#include <memory>
#include <set>

namespace vulcan
{
struct velocity_t;

namespace hssh
{

struct lpm_params_t;

/**
 * MapperMode specifies the different modes for building the LPM:
 *
 *   - scrolling : use a fixed-size LPM that follows the robot around
 *   - expanding : grow the LPM as the robot explores the local environment
 */
enum class MapperMode
{
    scrolling,
    expanding
};


/**
 * Mapper handles the mapping portion of the SLAM problem. The mapping system builds two maps to
 * form a hybrid mapping system that is necessary to model the more unusual features in the world,
 * like glass walls. These two maps are a traditional occupancy grid and a line map which models
 * the dominant line features of the world.
 *
 * The primary reason for the line map is to locate the transitory, occasionally visible features
 * of the world. By having a line representation, the angle of incidence between the laser ray
 * and the grid cell can be calculated. Knowing this angle allows the system to then calculate
 * how likely a cell is to be visible to the laser. Thus, cells unlikely to be visible will not
 * have their weights decrease much when marked seen as free space by the laser.
 */
class Mapper
{
public:
    /**
     * Constructor for Mapper.
     */
    Mapper(const mapper_params_t& params);

    // No value semantics
    Mapper(const Mapper& copy) = delete;
    void operator=(const Mapper& rhs) = delete;

    /**
     * resetMap resets the map with the robot at the specified pose.
     *
     * \param    pose            Pose at which the reset map should be centered
     */
    void resetMap(const pose_t& pose);

    /**
     * setMap changes the map currently being constructed.
     *
     * \param    lpm             New LPM to be used
     */
    void setMap(const LocalPerceptualMap& lpm);

    /**
     * updateMap updates the map with the most recent piece of sensor data, along with the pose
     * estimate for the robot at the time the data was captured.
     *
     * The updated maps can be accessed via the getLPM() and getGlassMap() methods.
     *
     * \return   True if the map was updated. False if the current robot state is such that mapping has a high
     *   likelihood of breaking the map, so the map was not updated.
     */
    bool updateMap(const pose_distribution_t& poseDistribution, const metric_slam_data_t& sensorData);

    /**
     * updateMapTime updates the current map timestamp. It is used when running in localization-only mode and no
     * full map updates are happening.
     */
    void updateMapTime(const metric_slam_data_t& sensorData);

    /**
     * rotateMap rotates the LPM around its center by the specified number of radians.
     *
     * \param    radians         Radians by which to rotate the map
     */
    void rotateMap(float radians);

    /**
     * truncateMap truncates the map so its contents fit within the specified bounding rectangle.
     *
     * \param    boundary        New boundary for the map
     */
    void truncateMap(const math::Rectangle<float>& boundary);

    /**
     * changeReferenceFrame changes the reference frame of the map to be centered with the origin (0, 0, 0)
     * at the provided pose.
     *
     * The map is rotated about the center and then the bottom left is translated to reflect the new origin.
     *
     * \param    referenceFrame      New reference frame for the map
     */
    void changeReferenceFrame(const pose_t& referenceFrame);

    /**
     * setMappingMode sets the mode to use for constructing the map.
     *
     * \param    mode                Mode to use
     */
    void setMappingMode(MapperMode mode) { mode_ = mode; }

    /**
     * shouldUpdateMap sets a flag indicating if the map should be updated during subsequent calls to updateMap.
     *
     * \param    shouldUpdate        Flag indicating if the map will be updated
     */
    void shouldUpdateMap(bool shouldUpdate) { shouldUpdateMap_ = shouldUpdate; }

    /**
     * shouldBuildGlassMap sets a flag indicating if the glass map should be built.
     *
     * \param    shouldBuild         Flag indicating if the glass mapping should be running or not
     */
    void shouldBuildGlassMap(bool shouldBuild);

    /**
     * shouldBuildHighResMap sets a flag indicating if the high-res map should be built and output instead of the normal
     * LPM.
     *
     * \param    shouldBuild         Flag toggling on/off high-res map mode
     */
    void shouldBuildHighResMap(bool shouldBuild) { shouldBuildHighRes_ = shouldBuild; }

    // Accessors for the fresh maps
    const LocalPerceptualMap& getLPM(void) const { return lpm_; }
    const GlassMap& getGlassMap(void) const { return glassBuilder_->getMap(); }
    const LocalPerceptualMap& getFlattenedMap(void) const { return glassBuilder_->getFlattenedMap(); }
    // Accessors for debugging information about the actual laser data used for the map update
    const laser::MovingLaserScan& getMappingLaser(void) const { return mappingLaser_; }
    const laser::ReflectedLaserScan& getReflectedLaser(void) const { return reflectedLaser_; }

private:
    LaserScanRasterizer rasterizer_;
    LPMBuilder lpmBuilder_;
    std::unique_ptr<GlassMapBuilder> glassBuilder_;

    LocalPerceptualMap lpm_;
    math::Rectangle<float> boundary_;
    pose_t previousPose_;   // Previous pose at which the map was updated

    laser::MovingLaserScan mappingLaser_;        // laser used for most recent map update
    laser::ReflectedLaserScan reflectedLaser_;   // laser with reflection info from the most recent update

    MapperMode mode_;
    bool shouldUpdateMap_;
    bool shouldBuildGlass_;
    bool shouldBuildHighRes_;
    std::set<int> initializedLasers_;   // Flag indicating which lasers been initialized since the latest reset

    int64_t tightTurnStartTime_;
    int64_t lastMergeTime_;

    mapper_params_t params_;

    void initializeGlassMapping(void);

    bool shouldMap(const pose_distribution_t& pose, const velocity_t& velocity, int laserIndex);
    bool initializeMapUpdate(const pose_t& currentPose, const laser::MovingLaserScan& scan);
    void updateBoundaryIfNeeded(const math::Rectangle<float>& newBoundary);

    math::Rectangle<float> calculateExpandingBoundary(const laser_scan_raster_t& raster, const pose_t& pose);

    void buildMaps(const map_update_data_t& data);
    void addGlassMapToLPM(const LocalPerceptualMap& glassMap);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_METRICAL_MAPPING_MAPPER_H
