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
* Declaration of the params structs for the Logical Interface UI and the load_logical_interface_params function.
*/

#ifndef UI_LOGICAL_PARAMS_H
#define UI_LOGICAL_PARAMS_H

#include <string>
#include <ui/common/ui_params.h>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace ui
{

struct logical_interface_experiment_params_t
{
    std::string experimentMap;
    std::string placeDescriptions;
    std::string experimentTasks;
    std::size_t numRandomTasks;
    std::string randomTasksFilename;
    std::string experimentMode;
    float       mixedModeGoalRatio;
};

struct logical_interface_params_t
{
    std::string setTopoMapMessageChannel;
    std::string targetSequenceChannel;
    std::string globalTargetChannel;

    ui_params_t                           uiParams;
    logical_interface_experiment_params_t experimentParams;
};

/**
* load_logical_interface_params loads the parameters for the logical interface.
*
* \param    config          ConfigFile with all the parameters
* \return   Parameters pulled from the config file.
*/
logical_interface_params_t load_logical_interface_params(const utils::ConfigFile& config);

}
}

#endif // UI_LOGICAL_PARAMS_H
