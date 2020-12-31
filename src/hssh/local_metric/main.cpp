/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_metric/director.h"
#include "system/module.h"
#include "utils/command_line.h"
#include "utils/config_file.h"
#include <vector>

using namespace vulcan;

int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});
    arguments.push_back({hssh::kSavePosesArg,
                         "Save all LocalPoses to the provided file (if no file, then a default filename is used)",
                         true,
                         ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    if (!commandLine.verify()) {
        return 1;
    }

    utils::ConfigFile config(commandLine.configName());

    system::Module<hssh::LocalMetricDirector> module(commandLine, config);

    module.run();

    return 0;
}
