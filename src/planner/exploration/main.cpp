/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "planner/exploration/director.h"
#include "planner/exploration/map_explorer.h"
#include "system/module.h"
#include "utils/command_line.h"
#include "utils/config_file.h"
#include <vector>


const std::string kExplorerOption("explorer");
const std::string kMapOption("map");


int main(int argc, char** argv)
{
    using namespace vulcan;
    using namespace vulcan::planner;

    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});
    arguments.push_back({kExplorerOption, "The type of explorer to use. Current options: local_topo", true, ""});
    arguments.push_back({kMapOption,
                         "The map to be explored. Exact format depends on explorer. Currently, should be a .ltm file.",
                         true,
                         ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    if (!commandLine.verify()) {
        return -1;
    }

    utils::ConfigFile config(commandLine.configName());

    std::unique_ptr<MapExplorer> explorer =
      create_map_explorer(commandLine.argumentValue(kExplorerOption), commandLine.argumentValue(kMapOption), config);
    std::unique_ptr<ExplorationDirector> director(new ExplorationDirector(std::move(explorer)));

    system::Module<ExplorationDirector> module(std::move(director), commandLine, config);

    module.run();

    return 0;
}
