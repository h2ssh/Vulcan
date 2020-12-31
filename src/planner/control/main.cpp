/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "planner/control/director.h"
#include "planner/control/planner.h"
#include "system/module.h"
#include "utils/command_line.h"
#include "utils/config_file.h"
#include <vector>


int main(int argc, char** argv)
{
    using namespace vulcan;

    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    commandLine.verify();

    utils::ConfigFile config(commandLine.configName());

    std::unique_ptr<planner::ControlPlanner> controlPlanner(new planner::ControlPlanner);
    std::unique_ptr<planner::ControlPlannerDirector> director(
      new planner::ControlPlannerDirector(std::move(controlPlanner)));

    system::Module<planner::ControlPlannerDirector> module(std::move(director), commandLine, config);

    module.run();

    return 0;
}
