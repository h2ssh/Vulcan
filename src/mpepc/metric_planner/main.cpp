/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     main.cpp
* \author   Collin Johnson
*/

#include <system/module.h>
#include <mpepc/metric_planner/director.h>
#include <utils/config_file.h>
#include <utils/command_line.h>
#include <vector>

using namespace vulcan;

int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});
    arguments.push_back({mpepc::kLessDebugInfoArgument, "Reduced amount of debug information output", true, ""});
    arguments.push_back({mpepc::kLocalTopoMapArgument, "Optional LocalTopoMap of the current environment", true, ""});
    arguments.push_back({mpepc::kMetricMapArgument, "Optional initial LPM of the current environment. "
        "Will be replaced once an LPM arrives from another module", true, ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    if(!commandLine.verify())
    {
        return 1;
    }

    utils::ConfigFile  config(commandLine.configName());

    system::Module<mpepc::MetricPlannerDirector> module(commandLine, config);

    module.run();

    return 0;
}
