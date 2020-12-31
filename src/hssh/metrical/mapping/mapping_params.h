/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     mapping_params.h
 * \author   Collin Johnson
 *
 * Declaration of the params structs needed by the local_metric mapping subsystem.
 */

#ifndef HSSH_LOCAL_METRIC_MAPPING_MAPPING_PARAMS_H
#define HSSH_LOCAL_METRIC_MAPPING_MAPPING_PARAMS_H

#include <cassert>
#include <string>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace hssh
{

struct laser_scan_rasterizer_params_t
{
    float maxLaserDistance;
    uint16_t minLaserIntensity;

    int8_t occCellCostChange;
    int8_t freeCellCostChange;

    int8_t initialOccupiedCostChange;
    int8_t initialFreeCostChange;
};

struct lpm_params_t
{
    int width;
    int height;
    float scale;
    uint8_t maxCellCost;
    uint8_t occupiedCellCost;
    uint8_t freeCellCost;
};

struct glass_map_builder_params_t
{
    float maxLaserRange;
    int numAngleBins;
    bool shouldFilterDynamic;        // flag indicating if the dynamic object filter should be run
    bool shouldFilterReflections;    // flag indicating if reflections should be removed from the laser scan
    bool canSeeCellsFromBothSides;   // have the cells represent 180 degrees, not 360 degrees.
                                     // Implemented such that a hit at theta or theta + 180 results in changing the
                                     // count for the angle bin

    int hitThreshold;    // number of hits in a bin for it to get marked as a hit while building the map
    int missThreshold;   // number of misses in a bin for it to get marked as a miss while building the map

    double minVisibleOccupiedRange;   // [radians] minimum visible range for a cell to be considered occupied in the
                                      // flattened map
    double minHighlyVisibleRange;     // [radians] minimum visible range for a cell to be highly visible in the dynamic
                                      // object filter
    uint16_t minHighlyVisibleIntensity;   // [sensor-specific counts] minimum intensity reading for a cell to be
                                          // considered highly-visible glass
};

struct mapper_params_t
{
    std::string type;
    bool shouldBuildGlassMap;
    bool shouldUseMovingLaser;

    int maxMapWidthMeters;
    int maxMapHeightMeters;
    float shiftRadius;
    float placeBoundaryRadius;

    double maxMappingPositionStdDev;
    double maxMappingOrientationStdDev;
    double minRadiusOfCurvature;

    laser_scan_rasterizer_params_t rasterizerParams;
    lpm_params_t lpmParams;
    glass_map_builder_params_t glassBuilderParams;
};

/**
 * load_mapper_params loads the parameters for mapping subsystem of the local_metric layer of the HSSH.
 * In addition to the ConfigFile, the heading under which the parameters are located is provided.
 *
 * \param    config          ConfigFile containing the values parsed from a .cfg file
 * \param    heading         Heading under which the parameters are located
 * \return   mapper_params_t created from the config.
 */
mapper_params_t load_mapper_params(const utils::ConfigFile& config, const std::string& heading);

/**
 * load_lpm_params loads the parameter set for an LPM. In addition to
 * the ConfigFile, the heading under which the parameters are located is provided. It is likely
 * that LPMs will exist in multiple modules, so the heading of their parameters may
 * need to change.
 *
 * \param    config          File with the parameters
 * \param    heading         Heading under which the parameters are located
 * \return   Parameters for creating an LPM.
 */
lpm_params_t load_lpm_params(const utils::ConfigFile& config, const std::string& heading);

/**
 * load_rasterizer_params loads the parameter set for a rasterizer. In addition to
 * the ConfigFile, the heading under which the parameters are located is provided.
 *
 * \param    config          File with the parameters
 * \param    heading         Heading under which the parameters are located
 * \return   Parameters for creating a rasterizer.
 */
laser_scan_rasterizer_params_t load_rasterizer_params(const utils::ConfigFile& config, const std::string& heading);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_METRIC_MAPPING_MAPPING_PARAMS_H
