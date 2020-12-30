/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <system/module.h>
#include <hssh/local_topological/cmd_line.h>
#include <hssh/local_topological/director.h>
#include <utils/config_file.h>
#include <utils/command_line.h>
#include <utils/serialized_file_io.h>
#include <vector>

using namespace vulcan;

/**
* lpm_main is the main program for the LPM. It creates the Communicator and the Director, launches their
* threads and then relaxes.
*
* The primary task to be accomplished is creating the message subscriptions needed by the LPM. A
* template-based method is used for these subscriptions, so each message needs to be formulated
* individually to ensure the correct types are used.
*
* The command-line arguments for lpm are:
*
*   --config-file 'filename'        Name of the configuration file for the LPM
*
*/
int main(int argc, char** argv)
{
    using namespace vulcan::hssh;

    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file controlling the module behavior", true, ""});
    arguments.push_back({kMapNameArg, "Name of the map or building the robot is operating in.", true, ""});
    arguments.push_back({kDirectoryArg, "Directory where learned classifiers are stored.", true, "../data/classifiers/"});
    arguments.push_back({kSaveEventsArg, "Save all generated events on exit to the specified file", true, ""});
    arguments.push_back({kSaveMapArg, "Save the final LocalTopoMap to the specified file", true, ""});
    arguments.push_back({kOneShotArg, "Provide an LPM to process in a single-go. Need to also specify MapName and SaveMapArg.", true, ""});
    arguments.push_back({kConstraintLogProbArg, "Log-probability decrease of a failing constraint.", true, ""});
    arguments.push_back({kRepeatLogProbArg, "Log-probability decrease of a repeated network configuration.", true, ""});
    arguments.push_back({kMaxIterationsArg, "Maximum number of iterations to run MCMC sampling", true, ""});
    arguments.push_back({kSamplesPerIterArg, "Number of change samples to draw per iteration.", true, ""});

    utils::CommandLine commandLine(argc, argv, arguments);
    if(!commandLine.verify())
    {
        return 1;
    }

    utils::ConfigFile config(commandLine.configName());

    if(commandLine.argumentExists(kOneShotArg))
    {
        assert(commandLine.argumentExists(kSaveMapArg));
        assert(commandLine.argumentExists(kMapNameArg));

        LocalPerceptualMap lpm;
        utils::load_serializable_from_file(commandLine.argumentValue(kOneShotArg), lpm);

        LocalTopologyDirector dir(commandLine, config);
        auto ltm = dir.oneShotLabeling(lpm);
        if(!utils::save_serializable_to_file(commandLine.argumentValue(kSaveMapArg), ltm))
        {
            std::cerr << "ERROR: Failed to save LTM to " << commandLine.argumentValue(kSaveMapArg) << '\n';
            return 1;
        }

        // Store result in a file that script can read to determine quality of computed map
        std::ofstream out("ltm_result.txt");
        out << static_cast<int>(ltm.logProb() * 100) << std::endl;
        return 0;
    }
    else
    {
        system::Module<LocalTopologyDirector> module(commandLine, config);
        module.run();
    }

    return 0;
}
