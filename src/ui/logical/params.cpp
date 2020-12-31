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
 * Definition of parsing functions for the params structs.
 */

#include "ui/logical/params.h"
#include "utils/config_file.h"

namespace vulcan
{
namespace ui
{

const std::string LOGICAL_FRAME_HEADING("LogicalInterfaceParameters");
const std::string SET_MAP_KEY("set_topological_map_output_channel");
const std::string LOCAL_SEQUENCE_KEY("local_topo_target_sequence_output_channel");
const std::string GLOBAL_TARGET_KEY("global_topo_target_output_channel");

const std::string EXPERIMENT_HEADING("LogicalInterfaceExperimentParameters");
const std::string EXP_MAP_KEY("experiment_map");
const std::string PLACE_DESC_KEY("place_descriptions");
const std::string EXP_TASKS_KEY("experiment_tasks");
const std::string NUM_TASKS_KEY("num_random_tasks");
const std::string TASKS_FILE_KEY("random_tasks_filename");
const std::string EXP_MODE_KEY("experiment_mode");
const std::string MIXED_RATIO_KEY("mixed_mode_goal_ratio");

logical_interface_experiment_params_t load_experiment_params(const utils::ConfigFile& config);


logical_interface_params_t load_logical_interface_params(const utils::ConfigFile& config)
{
    logical_interface_params_t params;

    params.setTopoMapMessageChannel = config.getValueAsString(LOGICAL_FRAME_HEADING, SET_MAP_KEY);
    params.targetSequenceChannel = config.getValueAsString(LOGICAL_FRAME_HEADING, LOCAL_SEQUENCE_KEY);
    params.globalTargetChannel = config.getValueAsString(LOGICAL_FRAME_HEADING, GLOBAL_TARGET_KEY);

    params.uiParams = load_ui_params(config);
    params.experimentParams = load_experiment_params(config);

    return params;
}


logical_interface_experiment_params_t load_experiment_params(const utils::ConfigFile& config)
{
    logical_interface_experiment_params_t params;

    params.experimentMap = config.getValueAsString(EXPERIMENT_HEADING, EXP_MAP_KEY);
    params.placeDescriptions = config.getValueAsString(EXPERIMENT_HEADING, PLACE_DESC_KEY);
    params.experimentTasks = config.getValueAsString(EXPERIMENT_HEADING, EXP_TASKS_KEY);
    params.numRandomTasks = config.getValueAsInt32(EXPERIMENT_HEADING, NUM_TASKS_KEY);
    params.randomTasksFilename = config.getValueAsString(EXPERIMENT_HEADING, TASKS_FILE_KEY);
    params.experimentMode = config.getValueAsString(EXPERIMENT_HEADING, EXP_MODE_KEY);
    params.mixedModeGoalRatio = config.getValueAsFloat(EXPERIMENT_HEADING, MIXED_RATIO_KEY);

    return params;
}

}   // namespace ui
}   // namespace vulcan
