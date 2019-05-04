/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     event_log_runner.h
* \author   Collin Johnson
*
* Declaration of EventLogRunner.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_UTILS_EVENT_LOG_RUNNER_H
#define HSSH_GLOBAL_TOPOLOGICAL_UTILS_EVENT_LOG_RUNNER_H

#include <hssh/local_metric/pose.h>
#include <hssh/local_topological/event.h>
#include <vector>

namespace vulcan
{
namespace hssh
{

class GlobalTopoDirector;

/**
* EventLogRunner loads poses and events from stored logs. The logs can be either an AreaStabilityLog that contains
* LocalAreaEvents and LocalPoses, or they can be two text files, one containing the events and the other the LocalPose.
*
* To create the AreaStabilityLog, use the generate_stability_logs.sh script, which will perform the correct local and
* global localization, along with the detection of area transitions.
*
* The event runner replaces using a system::Module. The runner will feed data to the GlobalTopoDirector via its
* handleData methods using data read from the log files. The run method will keep going until it has passed all data
* to the director, at which point it returns and the final topological map will be constructed -- or subsequent events
* can be fed in.
*
* The event runner supports an interactive mode, where the user must hit enter to trigger the next local topo event
* to fire.
*/
class EventLogRunner
{
public:

    /**
    * Constructor for EventLogRunner.
    *
    * Create a log runner from a log that contains data logged by the stability logging script.
    *
    * \param    filename        Name of the stability log to load
    */
    explicit EventLogRunner(const std::string& filename);

    /**
    * Constructor for EventLogRunner.
    *
    * \param    eventName       Name of file containing the events
    * \param    poseName        Name of file containing the poses
    */
    EventLogRunner(const std::string& eventName, const std::string& poseName);

    /**
    * run runs the director through the contents of the loaded log file. At the end of the run, the director will
    * contain the final information about the global topological map described by the provided events.
    *
    * \param    director            Director for processing the events
    * \param    interactive     Flag indicating if runner should be in interactive mode.
    */
    void run(GlobalTopoDirector& director, bool interactive);

private:

    LocalAreaEventVec events_;
    std::vector<LocalPose> poses_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_UTILS_EVENT_LOG_RUNNER_H
