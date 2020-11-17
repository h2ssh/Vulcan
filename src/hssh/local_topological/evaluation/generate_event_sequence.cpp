/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     generate_event_sequence.cpp
* \author   Collin Johnson
*
* generate_event_sequence is a program to generate a sequence of LocalAreaEvents from a saved PoseTrace and a
* LocalTopoMap of the same environment. The program will save the sequence to a file for use with
* evaluate_incremental_labels.
*
* The command-line arguments for generate_event_sequence are:
*
*   Required:
*
*   --map 'filename'    : filename of the saved LocalTopoMap
*   --poses 'filename'  : filename of the stored PoseTrace
*   --events-out 'filename' : filename in which to store the generated files
*
*   Optional:
*
*   --config-file 'config' : location of config file for event detector (default = local_topo_hssh.cfg)
*/

#include <hssh/local_topological/event_detector.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/areas/serialization.h>
#include <hssh/local_metric/pose.h>
#include <utils/command_line.h>
#include <utils/config_file.h>
#include <utils/serialized_file_io.h>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <iostream>
#include <vector>

using namespace vulcan;
using namespace vulcan::hssh;


const std::string kMapArg("map");
const std::string kPosesArg("poses");
const std::string kEventsArg("events-out");


int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> args{
        {kMapArg, "filename of the saved LocalTopoMap", false, ""},
        {kPosesArg, "filename of the stored PoseTrace", false, ""},
        {kEventsArg, "filename in which to store the generated files", false, ""},
        {utils::kConfigFileArgument, "filename with the local topo config", true, "event_generator.cfg"}
    };

    utils::CommandLine cmdLine(argc, argv, args);
    if(!cmdLine.verify())
    {
        return -1;
    }

    LocalTopoMap map;          // Ground-truth map in which the robot is moving
    if(!utils::load_serializable_from_file(cmdLine.argumentValue(kMapArg), map))
    {
        std::cerr << "ERROR: Unable to load LocalTopoMap from " << cmdLine.argumentValue(kMapArg) << '\n';
        return -1;
    }

    utils::PoseTrace poses(cmdLine.argumentValue(kPosesArg));   // Poses from which to generate events
    if(poses.empty())
    {
        std::cerr << "ERROR: Pose trace was empty: " << cmdLine.argumentValue(kPosesArg) << '\n';
        return -1;
    }

    EventDetector detector(utils::ConfigFile(cmdLine.configName()));    // Event detector for finding events

    LocalAreaEventVec events;

    for(auto& p : poses)
    {
        detector.addPose(hssh::LocalPose(pose_distribution_t(p), 0));

        auto poseEvents = detector.detectEvents(map);

        boost::push_back(events, boost::as_array(poseEvents));

        for(auto& evt : poseEvents)
        {
            std::cout << evt->description() << '\n';
        }
    }

    std::cout << "INFO: Detected " << events.size() << " events.\n";

    if(!utils::save_serializable_to_file(cmdLine.argumentValue(kEventsArg), events))
    {
        std::cerr << "ERROR: Unable to save events to " << cmdLine.argumentValue(kEventsArg) << '\n';
        return -1;
    }
    else
    {
        std::cout << "INFO: Successfully saved events to " << cmdLine.argumentValue(kEventsArg) << '\n';
    }

    return 0;
}
