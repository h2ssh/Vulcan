/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_metric/params.h"
#include "utils/config_file.h"

namespace vulcan
{
namespace hssh
{

const std::string LOCAL_METRIC_PARAMS_HEADING("LocalMetricHSSHParameters");
const std::string TARGET_HZ_KEY              ("target_update_hz");
const std::string MAP_TRANS_PERIOD_KEY       ("map_transmission_period_ms");
const std::string SEND_GLASS_KEY             ("should_send_glass_map");

const std::string MAPPER_HEADING             ("MapperParameters");
const std::string HIGH_RES_MAPPER_HEADING    ("HighResolutionMapperParameters");
const std::string HIGH_RES_LPM_HEADING       ("HighResolutionLPMParameters");
const std::string HIGH_RES_RASTERIZER_HEADING("HighResolutionLaserScanRasterizerParameters");

mapper_params_t load_high_res_mapper_params(const utils::ConfigFile& config);


local_metric_hssh_params_t::local_metric_hssh_params_t(const utils::ConfigFile& config)
: localizationParams(config)
, relocalizationParams(config)
{
    targetUpdateHz          = config.getValueAsFloat(LOCAL_METRIC_PARAMS_HEADING, TARGET_HZ_KEY);
    mapTransmissionPeriodUs = config.getValueAsInt32(LOCAL_METRIC_PARAMS_HEADING, MAP_TRANS_PERIOD_KEY) * 1000ll;
    shouldSendGlassMap      = config.getValueAsBool(LOCAL_METRIC_PARAMS_HEADING,  SEND_GLASS_KEY);
    
    mapperParams        = load_mapper_params(config, MAPPER_HEADING);
    highResMapperParams = load_high_res_mapper_params(config);
    multiFloorParams    = load_multi_floor_mapper_params(config);
}


mapper_params_t load_high_res_mapper_params(const utils::ConfigFile& config)
{
    mapper_params_t params = load_mapper_params(config, HIGH_RES_MAPPER_HEADING);
    
    params.lpmParams           = load_lpm_params(config, HIGH_RES_LPM_HEADING);
    params.rasterizerParams    = load_rasterizer_params(config, HIGH_RES_RASTERIZER_HEADING);            
    
    return params;
}

} // namespace hssh
} // namespace vulcan
