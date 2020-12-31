/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "system/module.h"
#include "hssh/global_metric/director.h"
#include "utils/config_file.h"
#include "utils/command_line.h"
#include <vector>

using namespace vulcan;

int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, "local_metric_hssh.cfg"});
    arguments.push_back({hssh::kEmulateLPMArg, "Turn on or off Local Metric level emulation", true, ""});
    arguments.push_back({hssh::kUpdateRateArg, "Specify the update rate for the module (Hz) (optional, default = 20)",
                         true, "20"});
    arguments.push_back({hssh::kSavePosesArg, "Save generated poses to the specified file. Specify name if not using default.",
                         true, ""});
    arguments.push_back({hssh::kMapArg, "Map in which to localize. Optional, can specify via DebugUI. Also requires "
                         "initial-rect to be specified in order to relocalize right away.", true, ""});
    arguments.push_back({hssh::kInitialRectArg, "Optional initial bounding rect to relocalize in. Format: (bl_x,bl_y),"
                         "(tr_x,tr_y)", true, ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    commandLine.verify();

    utils::ConfigFile config(commandLine.configName());

    system::Module<hssh::GlobalMetricDirector> module(commandLine, config);

    module.run();

    return 0;
}
