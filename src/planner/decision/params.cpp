/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Collin Johnson
*
* Definition of load_decision_planner_params and other various functions for parsing a ConfigFile
* into a usable set of parameters.
*/

#include <planner/decision/params.h>
#include <utils/config_file.h>

namespace vulcan
{
namespace planner
{

const std::string PLANNER_HEADING("DecisionPlannerParameters");
const std::string HANDLER_TYPE_KEY("handler_type");

const std::string HANDLER_CHAIN_HEADING("DecisionTargetHandlerChainParameters");
const std::string HANDLER_SEQUENCE_KEY ("handler_sequence");

decision_planner_params_t              load_planner_params      (const utils::ConfigFile& config);
decision_target_handler_params_t       load_handler_params      (const utils::ConfigFile& config);
decision_target_handler_chain_params_t load_handler_chain_params(const utils::ConfigFile& config);


decision_params_t load_decision_params(const utils::ConfigFile& config)
{
    decision_params_t params;

    params.plannerParams = load_planner_params(config);

    return params;
}


decision_planner_params_t load_planner_params(const utils::ConfigFile& config)
{
    decision_planner_params_t params;

    params.handlerType   = config.getValueAsString(PLANNER_HEADING, HANDLER_TYPE_KEY);
    params.handlerParams = load_handler_params(config);

    return params;
}


decision_target_handler_params_t load_handler_params(const utils::ConfigFile& config)
{
    decision_target_handler_params_t params;

    params.chainParams = load_handler_chain_params(config);

    return params;
}


decision_target_handler_chain_params_t load_handler_chain_params(const utils::ConfigFile& config)
{
    decision_target_handler_chain_params_t params;

    params.chainSequence = config.getValueAsString(HANDLER_CHAIN_HEADING, HANDLER_SEQUENCE_KEY);

    return params;
}

} // namespace planner
} // namespace vulcan
