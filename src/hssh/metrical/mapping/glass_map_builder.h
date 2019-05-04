/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     glass_map_builder.h
 * \author   Collin Johnson
 *
 * Declaration of GlassMapBuilder, the HSSH-specific version of Paul's glass mapping.
 */

#ifndef HSSH_LOCAL_METRIC_MAPPING_GLASS_MAP_BUILDER_H
#define HSSH_LOCAL_METRIC_MAPPING_GLASS_MAP_BUILDER_H

#include <hssh/metrical/mapping/map_builder.h>
#include <hssh/metrical/glass_map.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/types.h>
#include <laser/reflected_laser_scan.h>
#include <utils/discretized_angle_grid.h>
#include <map>

namespace vulcan
{
namespace hssh
{

struct lpm_params_t;
struct glass_map_builder_params_t;

/**
* GlassMapBuilder
*/
class GlassMapBuilder : public MapBuilder<GlassMap>
{
public:

    /**
    * Constructor for GlassMapBuilder.
    *
    * \param    mapParams       Parameters for configuring the map
    * \param    currentPose     Current pose of the robot, around which the map will initially be centered
    * \param    glassParams      Parameters for configuring the glass-building algorithm
    */
    GlassMapBuilder(const lpm_params_t& mapParams, 
                    const pose_t& currentPose, 
                    const glass_map_builder_params_t& glassParams);
    
    /**
    * detectReflections searches through the provided laser scan for reflections in the map. Any cells that a reflected
    * ray passes through are returned to the default unknown state, which has the useful effect of erasing mysterious
    * walls that can appear as a consequence of reflections.
    * 
    * \param    scan        Scan in which to find reflections
    * \return   Information on the reflections in the scan. Every valid ray (range > 0) in scan will be represented
    *   here. Rays in the scan with no reflections are simply marked as having no reflections, but are included in the
    *   returned scan.
    */
    laser::ReflectedLaserScan detectReflections(const laser::MovingLaserScan& scan);
    
    /**
    * getFlattenedMap retrieves the useable flattened map that represents the condensed version of the full 3D glass
    * map into a simpler 2D occupancy grid.
    * 
    * \return   Flattened LPM of the environment
    */
    const LocalPerceptualMap& getFlattenedMap() const { return map_.flattenedMap(); }

private:

    // ray_bin_t is the angle bin for a given ray in the laser scan
    struct ray_bin_t
    {
        double bearing;             // bearing of the ray
        int angleBin;               // angle bin the ray falls in
        std::size_t scanIndex;      // index of the ray in the MovingLaserScan
    };

    // Range is [beginIndex, endIndex)
    struct angle_bin_range_t
    {
        std::size_t beginIndex;
        std::size_t endIndex;
    };
    
    GlassMap map_;
    std::vector<ray_bin_t> scanBins_;
    std::vector<int> angleBinPlusPi_;
    float kMaxLaserRange_;
    bool kUse180Bins_;
    bool kShouldFilterDynamic_;
    bool kShouldFilterReflections_;
    int kMinVisibleBins_;
    int kMinHighlyVisibleBins_;
    uint16_t kMinGlassIntensity_;
    
    // MapBuilder interface
    void reset(void) override;
    void boundaryChanged(const math::Rectangle<float>& boundary) override;
    void update(const map_update_data_t& data) override;
    void rotate(float radians) override;
    
    GlassMap&       getMapInstance(void) override       { return map_; }
    const GlassMap& getMapInstance(void) const override { return map_; }

    // Internal helper methods for constructing the glass map
    void processScanPoints(const laser::MovingLaserScan& scan);
    void findScanBins(const laser::MovingLaserScan& scan);
    angle_bin_range_t findAngleBinRange(std::size_t beginIndex);
    CellVector markHitCellsInRange(angle_bin_range_t range, const laser::MovingLaserScan& scan);
    void markFreeCellsInRange(angle_bin_range_t range, const laser::MovingLaserScan& scan, const CellVector& hitCells);
    void markFreeCellsAlongRay(const ray_bin_t& ray, const laser::MovingLaserScan& scan, const CellVector& hitCells);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_METRIC_MAPPING_GLASS_MAP_BUILDER_H
