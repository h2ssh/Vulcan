/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LASER_SCAN_MATCHER_PARAMS_H
#define LASER_SCAN_MATCHER_PARAMS_H

#include <cstdint>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace laser
{

struct scan_matcher_params_t
{
    uint16_t fineResolutionGridWidth;
    uint16_t fineResolutionGridHeight;
    float    fineResolutionMetersPerCell;
    float    coarseResolutionMetersPerCell;

    float    fineAngularSearchResolution;
    float    coarseAngularSearchResolution;

    float minXSearchArea;
    float minYSearchArea;
    float minThetaSearchArea;
    float maxXSearchArea;
    float maxYSearchArea;
    float maxThetaSearchArea;

    int minNumTransformsToConsider;
};


scan_matcher_params_t load_scan_matcher_params(const utils::ConfigFile& config);

}
}

#endif // LASER_SCAN_MATCHER_PARAMS_H
