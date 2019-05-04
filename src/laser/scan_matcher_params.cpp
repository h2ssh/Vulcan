/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <utils/config_file.h>
#include <laser/scan_matcher_params.h>

namespace vulcan
{
namespace laser
{

const std::string SCAN_MATCHER_HEADING("ScanMatcherParameters");

const std::string FINE_GRID_WIDTH("fine_resolution_grid_width");
const std::string FINE_GRID_HEIGHT("fine_resolution_grid_height");
const std::string FINE_GRID_SCALE("fine_resolution_grid_scale");

const std::string COARSE_GRID_SCALE("coarse_resolution_grid_scale");

const std::string MIN_X_SEARCH_KEY("min_x_search_area");
const std::string MIN_Y_SEARCH_KEY("min_y_search_area");
const std::string MIN_THETA_SEARCH_KEY("min_theta_search_area");

const std::string MAX_X_SEARCH_KEY("max_x_search_area");
const std::string MAX_Y_SEARCH_KEY("max_y_search_area");
const std::string MAX_THETA_SEARCH_KEY("max_theta_search_area");

const std::string FINE_ANG_RES("fine_angular_resolution");
const std::string COARSE_ANG_RES("coarse_angular_resolution");

const std::string MIN_NUM_TRANSFORMS_KEY("min_num_transforms_to_consider");


scan_matcher_params_t load_scan_matcher_params(const utils::ConfigFile& config)
{
    scan_matcher_params_t params;

    params.fineResolutionGridWidth     = config.getValueAsUInt16(SCAN_MATCHER_HEADING, FINE_GRID_WIDTH);
    params.fineResolutionGridHeight    = config.getValueAsUInt16(SCAN_MATCHER_HEADING, FINE_GRID_HEIGHT);
    params.fineResolutionMetersPerCell = config.getValueAsFloat(SCAN_MATCHER_HEADING, FINE_GRID_SCALE);

    params.coarseResolutionMetersPerCell = config.getValueAsFloat(SCAN_MATCHER_HEADING, COARSE_GRID_SCALE);

    params.fineAngularSearchResolution   = config.getValueAsFloat(SCAN_MATCHER_HEADING, FINE_ANG_RES);
    params.coarseAngularSearchResolution = config.getValueAsFloat(SCAN_MATCHER_HEADING, COARSE_ANG_RES);

    params.minXSearchArea     = config.getValueAsFloat(SCAN_MATCHER_HEADING, MIN_X_SEARCH_KEY);
    params.minYSearchArea     = config.getValueAsFloat(SCAN_MATCHER_HEADING, MIN_Y_SEARCH_KEY);
    params.minThetaSearchArea = config.getValueAsFloat(SCAN_MATCHER_HEADING, MIN_THETA_SEARCH_KEY);

    params.maxXSearchArea     = config.getValueAsFloat(SCAN_MATCHER_HEADING, MAX_X_SEARCH_KEY);
    params.maxYSearchArea     = config.getValueAsFloat(SCAN_MATCHER_HEADING, MAX_Y_SEARCH_KEY);
    params.maxThetaSearchArea = config.getValueAsFloat(SCAN_MATCHER_HEADING, MAX_THETA_SEARCH_KEY);

    params.minNumTransformsToConsider = config.getValueAsInt16(SCAN_MATCHER_HEADING, MIN_NUM_TRANSFORMS_KEY);

    return params;
}

}
}
