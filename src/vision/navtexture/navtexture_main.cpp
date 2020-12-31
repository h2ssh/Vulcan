/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>

#include "system/module.h"
#include "utils/command_line.h"
#include "utils/config_file.h"

#include "vision/navtexture/navtexture_communicator.h"
#include "vision/navtexture/navtexture_director.h"
#include "vision/navtexture/navtexture_params.h"


const std::string HELP_SHORT("h");
const std::string HELP_LONG("help");
const std::string CONFIG("config-file");


void display_help_if_needed(const vulcan::utils::CommandLine& commandLine);


/**
 * navtexture_main launches the navigable texture classification module. The navigable texture
 * classifier works by segmenting an image and associating with each segments a texture. The
 * textures determined to be on the ground plane are then compared against the set of dynamic
 * objects in the world. Those objects moving on the ground plane are then used to help learn
 * classifiers for the navigable ground plane textures in the world
 *
 * The command-line arguments for navtexture are:
 *
 *   -h/--help                   Display the help message
 *   --config-file 'filename'    Location of the configuration file with the settings for navtexture
 *
 */
int main(int argc, char** argv)
{
    vulcan::utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    vulcan::utils::ConfigFile config(commandLine.argumentValue(CONFIG));

    vulcan::vision::navtexture_params_t params = vulcan::vision::load_navtexture_params(config);

    boost::shared_ptr<vulcan::vision::NavTextureCommunicator> communicator(
      new vulcan::vision::NavTextureCommunicator());
    boost::shared_ptr<vulcan::vision::NavTextureDirector> director(new vulcan::vision::NavTextureDirector(params));

    vulcan::system::Module<vulcan::vision::NavTextureCommunicator, vulcan::vision::NavTextureDirector> module(
      communicator,
      director);

    module.run();

    return 0;
}


void display_help_if_needed(const vulcan::utils::CommandLine& commandLine)
{
    bool help_needed = commandLine.argumentExists(HELP_SHORT) || commandLine.argumentExists(HELP_LONG)
      || !commandLine.argumentExists(CONFIG);

    if (help_needed) {
        std::cout << "The command-line arguments for navtexture are:\n"
                  << '\n'
                  << "-h/--help                   Display the help message\n"
                  << "--config-file 'filename'    Location of the configuration file with the settings for navtexture\n"
                  << std::endl;

        exit(1);
    }
}
