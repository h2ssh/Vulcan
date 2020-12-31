/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef HSSH_LOCAL_METRIC_LOCAL_METRIC_PARAMS_H
#define HSSH_LOCAL_METRIC_LOCAL_METRIC_PARAMS_H

#include "hssh/local_metric/multifloor/params.h"
#include "hssh/metrical/localization/params.h"
#include "hssh/metrical/mapping/mapping_params.h"
#include "hssh/metrical/relocalization/params.h"
#include <stdint.h>
#include <string>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}

namespace hssh
{

struct local_metric_hssh_params_t
{
    float targetUpdateHz;   // target update rate in Hz

    int64_t mapTransmissionPeriodUs;   // time between full map transmissions (updates happen more frequently)
    bool shouldSendGlassMap;           // The glass map takes a lot of time to send, so don't do it very often.

    mapper_params_t mapperParams;
    mapper_params_t highResMapperParams;
    localizer_params_t localizationParams;
    metric_relocalizer_params_t relocalizationParams;
    multi_floor_mapper_params_t multiFloorParams;

    /**
     * Constructor for local_metric_hssh_params_t.
     *
     * \param    config          ConfigFile containing the parameters for the instance of local_metric_hssh
     */
    local_metric_hssh_params_t(const utils::ConfigFile& config);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_METRIC_LOCAL_METRIC_PARAMS_H
