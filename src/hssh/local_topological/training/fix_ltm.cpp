/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_topological/training/area_labels.h"
#include "hssh/local_topological/params.h"
#include "utils/command_line.h"
#include "utils/config_file.h"
#include <iostream>


const std::string kDirectoryArg("directory");


int main(int argc, char** argv)
{
    using namespace vulcan;
    using namespace vulcan::hssh;

    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file for calculating local topo features", true, "local_topo_hssh.cfg"});
    arguments.push_back({kDirectoryArg, "Directory containing the maps to be fixed", false, ""});

    utils::CommandLine cmdLine(argc, argv, arguments);
    if(!cmdLine.verify())
    {
        return -1;
    }

    utils::ConfigFile config(cmdLine.configName());
    local_topology_params_t params(config);

    auto labels = load_labels_directory(cmdLine.argumentValue(kDirectoryArg), params);
    for(auto& building : labels.fullMaps)
    {
        for(auto& map : building.second)
        {
            reconstruct_ltm_from_labels(map);
        }
    }

    return 0;
}
