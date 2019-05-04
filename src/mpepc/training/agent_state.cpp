/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* @file
* @author   Collin Johnson
*
* Definition of utility functions for dealing with agent_state_t observations.
*/

#include <mpepc/training/agent_state.h>
#include <fstream>

namespace vulcan
{
namespace mpepc
{

void save_motion_observations(const std::string& filename, const MotionObsMap& obs)
{
    FILE* out = fopen(filename.c_str(), "a");

    for(auto& areaObs : obs)
    {
        for(auto& motion : areaObs.second)
        {
            fprintf(out, "%d %.2f %.2f %.2f %.2f %.2f\n", areaObs.first, motion.x, motion.y, motion.xVel, motion.yVel, motion.normDist);
        }
    }

    fclose(out);
}


MotionObsMap load_motion_observations(const std::string& filename)
{
    MotionObsMap observations;

    std::ifstream in(filename);
    std::string line;

    while(std::getline(in, line))
    {
        std::istringstream inStr(line);
        int areaId = 0;
        agent_state_t state;

        inStr >> areaId >> state.x >> state.y >> state.xVel >> state.yVel >> state.normDist;
        observations[areaId].push_back(state);
    }

    return observations;
}

} // namespace mpepc
} // namespace vulcan
