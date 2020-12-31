/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mapping_params.cpp
* \author   Collin Johnson
*
* Definition of load_ functions for the mapping system params structs.
*/

#include "utils/config_file.h"
#include "hssh/metrical/mapping/mapping_params.h"

namespace vulcan
{
namespace hssh
{
const std::string TYPE_KEY            ("mapper_kind");
const std::string BUILD_GLASS_KEY     ("should_build_glass_map");
const std::string USE_MOVING_LASER_KEY("use_moving_laser");
const std::string SHIFT_RADIUS_KEY    ("shift_radius_m");
const std::string PLACE_RADIUS_KEY    ("place_boundary_radius_m");
const std::string MAX_MAP_HEIGHT_KEY  ("max_map_height_m");
const std::string MAX_MAP_WIDTH_KEY   ("max_map_width_m");
const std::string MAX_POS_ERROR_KEY   ("max_position_error_std_dev");
const std::string MAX_ORIENT_ERROR_KEY("max_orientation_error_std_dev");
const std::string MIN_CURVATURE_KEY   ("min_radius_of_curvature");
const std::string BUILDER_GLASS_KEY   ("should_build_glass_map");

const std::string LPM_HEADING       ("LPMParameters");
const std::string GRID_WIDTH_KEY    ("grid_width");
const std::string GRID_HEIGHT_KEY   ("grid_height");
const std::string CELL_SCALE_KEY    ("cell_size");
const std::string MAX_COST_KEY      ("max_cell_cost");
const std::string OCC_CELL_COST_KEY ("occupied_cell_cost");
const std::string FREE_CELL_COST_KEY("free_cell_cost");

const std::string RASTERIZER_HEADING   ("LaserScanRasterizerParameters");
const std::string INITIAL_OCC_COST_KEY ("initial_occupied_cost_change");
const std::string INITIAL_FREE_COST_KEY("initial_free_cost_change");
const std::string OCC_COST_CHANGE_KEY  ("occupied_cell_cost_change");
const std::string FREE_COST_CHANGE_KEY ("free_cell_cost_change");
const std::string MAX_LASER_DIST_KEY   ("max_laser_distance_m");
const std::string MIN_INTENSITY_KEY    ("min_laser_intensity");

const std::string GLASS_BUILDER_HEADING("GlassMapBuilderParameters");
const std::string MAX_GLASS_RANGE_KEY  ("max_laser_range_m");
const std::string NUM_ANGLE_BINS_KEY   ("num_angle_bins");
const std::string FILTER_DYNAMIC_KEY   ("should_filter_dynamic_objects");
const std::string FILTER_REFLECTIONS_KEY("should_filter_reflections");
const std::string SEE_CELLS_BOTH_SIDES_KEY("can_see_cells_from_both_sides");
const std::string HIT_THRESH_KEY       ("hit_threshold");
const std::string MISS_THRESH_KEY      ("miss_threshold");
const std::string VISIBLE_OCC_RANGE_KEY("min_visible_occupied_range_degrees");
const std::string HIGH_VISIBLE_OCC_RANGE_KEY("min_highly_visible_range_degrees");
const std::string HIGH_VISIBLE_INTENSITY_KEY("min_glass_intensity");


laser_scan_rasterizer_params_t load_rasterizer_params   (const utils::ConfigFile& config);
glass_map_builder_params_t     load_glass_builder_params(const utils::ConfigFile& config);


mapper_params_t load_mapper_params(const utils::ConfigFile& config, const std::string& heading)
{
    mapper_params_t params;
    
    params.type                        = config.getValueAsString(heading, TYPE_KEY);
    params.shouldBuildGlassMap         = config.getValueAsBool(heading, BUILD_GLASS_KEY);
    params.shouldUseMovingLaser        = config.getValueAsBool(heading, USE_MOVING_LASER_KEY);
    params.maxMapHeightMeters          = config.getValueAsInt32(heading, MAX_MAP_HEIGHT_KEY);
    params.maxMapWidthMeters           = config.getValueAsInt32(heading, MAX_MAP_WIDTH_KEY);
    params.maxMappingPositionStdDev    = config.getValueAsDouble(heading, MAX_POS_ERROR_KEY);
    params.maxMappingOrientationStdDev = config.getValueAsDouble(heading, MAX_ORIENT_ERROR_KEY);
    params.minRadiusOfCurvature        = config.getValueAsDouble(heading, MIN_CURVATURE_KEY);
    params.shiftRadius                 = config.getValueAsFloat(heading, SHIFT_RADIUS_KEY);
    params.placeBoundaryRadius         = config.getValueAsFloat(heading, PLACE_RADIUS_KEY);
    params.shouldBuildGlassMap         = config.getValueAsBool(heading, BUILDER_GLASS_KEY);

    params.rasterizerParams   = load_rasterizer_params(config,RASTERIZER_HEADING);
    params.lpmParams          = load_lpm_params(config, LPM_HEADING);
    params.glassBuilderParams = load_glass_builder_params(config);

    assert(params.maxMapWidthMeters > params.lpmParams.scale);
    assert(params.maxMapHeightMeters > params.lpmParams.scale);
    assert(params.maxMappingPositionStdDev > 0.01);
    assert(params.maxMappingOrientationStdDev > 0.01);
    assert(params.minRadiusOfCurvature >= 0.0);
    assert(params.shiftRadius > 0.0f);
    assert(params.placeBoundaryRadius > 0.0f);

    return params;
}


lpm_params_t load_lpm_params(const utils::ConfigFile& config, const std::string& heading)
{
    lpm_params_t gridParams;

    gridParams.width            = config.getValueAsInt32(heading, GRID_WIDTH_KEY);
    gridParams.height           = config.getValueAsInt32(heading, GRID_HEIGHT_KEY);
    gridParams.scale            = config.getValueAsFloat(heading, CELL_SCALE_KEY);
    gridParams.maxCellCost      = config.getValueAsUInt8(heading, MAX_COST_KEY);
    gridParams.occupiedCellCost = config.getValueAsUInt8(heading, OCC_CELL_COST_KEY);
    gridParams.freeCellCost     = config.getValueAsUInt8(heading, FREE_CELL_COST_KEY);

    assert(gridParams.freeCellCost < gridParams.occupiedCellCost);
    assert(gridParams.scale > 0.0f);
    assert(gridParams.maxCellCost >= gridParams.occupiedCellCost);
    assert(gridParams.height > 0);
    assert(gridParams.width > 0);

    return gridParams;
}


laser_scan_rasterizer_params_t load_rasterizer_params(const utils::ConfigFile& config, const std::string& heading)
{
    laser_scan_rasterizer_params_t params;
    params.maxLaserDistance  = config.getValueAsFloat(heading, MAX_LASER_DIST_KEY);
    params.minLaserIntensity = config.getValueAsUInt16(heading, MIN_INTENSITY_KEY);

    params.occCellCostChange  = config.getValueAsInt8(heading, OCC_COST_CHANGE_KEY);
    params.freeCellCostChange = config.getValueAsInt8(heading, FREE_COST_CHANGE_KEY);

    params.initialOccupiedCostChange = config.getValueAsInt8(heading, INITIAL_OCC_COST_KEY);
    params.initialFreeCostChange     = config.getValueAsInt8(heading, INITIAL_FREE_COST_KEY);

    assert(params.maxLaserDistance > 0);
    assert(params.minLaserIntensity >= 0);
    assert(params.occCellCostChange > 0);
//     assert(params.freeCellCostChange > 0);
//     assert(params.initialOccupiedCostChange > 0);
//     assert(params.initialFreeCostChange > 0);

    return params;
}


glass_map_builder_params_t load_glass_builder_params(const utils::ConfigFile& config)
{
    glass_map_builder_params_t params;

    params.maxLaserRange = config.getValueAsFloat(GLASS_BUILDER_HEADING, MAX_GLASS_RANGE_KEY);
    params.numAngleBins = config.getValueAsInt32(GLASS_BUILDER_HEADING, NUM_ANGLE_BINS_KEY);
    params.shouldFilterDynamic = config.getValueAsBool(GLASS_BUILDER_HEADING, FILTER_DYNAMIC_KEY);
    params.shouldFilterReflections = config.getValueAsBool(GLASS_BUILDER_HEADING, FILTER_REFLECTIONS_KEY);
    params.canSeeCellsFromBothSides = config.getValueAsBool(GLASS_BUILDER_HEADING, SEE_CELLS_BOTH_SIDES_KEY);
    params.hitThreshold = config.getValueAsInt32(GLASS_BUILDER_HEADING, HIT_THRESH_KEY);
    params.missThreshold = config.getValueAsInt32(GLASS_BUILDER_HEADING, MISS_THRESH_KEY);
    params.minVisibleOccupiedRange = config.getValueAsDouble(GLASS_BUILDER_HEADING, VISIBLE_OCC_RANGE_KEY) * M_PI / 180.0;
    params.minHighlyVisibleRange = config.getValueAsDouble(GLASS_BUILDER_HEADING, HIGH_VISIBLE_OCC_RANGE_KEY) * M_PI / 180.0;
    params.minHighlyVisibleIntensity = config.getValueAsUInt16(GLASS_BUILDER_HEADING, HIGH_VISIBLE_INTENSITY_KEY);

    assert(params.maxLaserRange > 0.0f);

    params.hitThreshold = std::max(1, params.hitThreshold);
    params.missThreshold = std::min(-1, params.missThreshold);
    
    return params;
}

} // namespace hssh
} // namespace vulcan
