/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
*
* Declaration of params structs for the decision planner and the load_decision_planner_params function
* for reading the parameters.
*/

#ifndef PLANNER_DECISION_PARAMS_H
#define PLANNER_DECISION_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace planner
{

struct decision_target_handler_chain_params_t
{
    std::string chainSequence;
};

struct decision_target_handler_params_t
{
    decision_target_handler_chain_params_t chainParams;
};

struct decision_planner_params_t
{
    std::string handlerType;

    decision_target_handler_params_t handlerParams;
};

struct decision_params_t
{
    decision_planner_params_t plannerParams;
};

/**
* load_decision_params loads the parameters for the decision planner from the provided
* config file.
*
* \param    config          Config file with the saved parameters
* \return   Parameters to use for the current instance of the decision planner.
*/
decision_params_t load_decision_params(const utils::ConfigFile& config);

}
}

#endif // PLANNER_DECISION_PARAMS_H
