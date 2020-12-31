/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include "system/module_communicator.h"
#include "utils/command_line.h"
#include <cassert>
#include <ctime>
#include <iostream>
#include <random>
#include <string>

using namespace vulcan;

class MapSaver
{
public:
    MapSaver(const std::string& basename, double percentToSave) : basename_(basename), percentToSave_(percentToSave)
    {
        srand48(std::time(0));
        assert(percentToSave_ > 0.0);
    }

    void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
    {
        if (drand48() < percentToSave_ / 100.0) {
            std::ostringstream filename;
            filename << basename_ << '_' << numSaved_ << ".lpm";
            hssh::save_lpm_1_0(lpm, filename.str());
            ++numSaved_;

            std::cout << "Saved LPM to " << filename.str() << '\n';
        }
    }

private:
    std::string basename_;
    double percentToSave_;
    int numSaved_ = 0;
};


int main(int argc, char** argv)
{
    const std::string kPercent("percent");
    const std::string kBase("basename");

    utils::CommandLine cmdLine(
      argc,
      argv,
      {{kPercent, "Percent of maps to be saved", true, "1"}, {kBase, "Base of the filename for maps", false, "error"}});

    if (!cmdLine.verify()) {
        cmdLine.printHelp();
        return -1;
    }

    double percentToSave = std::atof(cmdLine.argumentValue(kPercent).c_str());

    MapSaver saver(cmdLine.argumentValue(kBase), percentToSave);
    system::ModuleCommunicator communicator;

    communicator.subscribeTo<hssh::LocalPerceptualMap>(&saver);

    while (true) {
        communicator.processIncoming();
        usleep(50000);
    }

    return 0;
}
