/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     script.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of MetricPlannerScript.
*/

#ifndef MPEPC_METRIC_PLANNER_SCRIPT_H
#define MPEPC_METRIC_PLANNER_SCRIPT_H

#include "mpepc/metric_planner/script/script_task.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{

namespace mpepc
{

class MetricPlannerTask;
class ObstacleDistanceGrid;

/**
* MetricPlannerScript describes a scripted sequence of tasks to be performed by
* the robot, one after another. The script allows a fixed demo to be run in a
* known environment.
*
* The script contains the already completed tasks, the active task, and the
* remaining tasks. The script can be constructed via a UI and then transmitted,
* or it can be loaded from a file.
*
*
* The file format for the script is:
*
*   <script>
*       <name> script name </name>
*   </script>
*
*   A collection of the following task descriptions:
*   <pose>
*       <seq_num> sequence number </seq_num>
*       <target> x y theta </target>
*   </pose>
*
*   <door>
*       <seq_num> sequence number </seq_num>
*       <entry> x y theta </entry>
*       <exit> x y theta </exit>
*   </door>
*
*   <elevator>
*       <seq_num> sequence number </seq_num>
*       <entry> x y theta </entry>
*       <inside> x y theta </inside>
*       <exit> x y theta </exit>
*   </elevator>
*
*/
class MetricPlannerScript
{
public:

    using TaskIter   = std::vector<ScriptTask>::const_iterator;
    using TargetIter = std::vector<pose_t>::const_iterator;

    /**
    * Default constructor for MetricPlannerScript.
    */
    MetricPlannerScript(void) { }

    /**
    * Constructor for MetricPlannerScript.
    *
    * Create a script by loading it from a file
    *
    * \param    filename        Load the script from the given file
    */
    MetricPlannerScript(const std::string& filename);

    /**
    * Constructor for MetricPlannerScript.
    *
    * Create a script from pre-constructed tasks. All tasks will be remaining.
    *
    * \param    name            Name of the script
    * \param    tasks           Tasks to be used in the script
    */
    MetricPlannerScript(const std::string& name, const std::vector<ScriptTask>& tasks);

    /**
    * save saves the remaining tasks in the script to the specified file using the file format described
    * in the class description.
    *
    * \param    filename        File in which to save the script
    * \return   True if saved successfully. False if the file couldn't be opened.
    */
    bool save(const std::string& filename) const;

    /**
    * getScriptName retrieves the name of the script.
    */
    std::string getScriptName(void) const { return name_; }

    /**
    * size retrieves the number of tasks in the script.
    */
    std::size_t size(void) const { return tasks_.size(); }

    /**
    * begin grabs the first task in the script.
    */
    TaskIter begin(void) const { return tasks_.begin(); }

    /**
    * end is the one-past the end for tasks in the script.
    */
    TaskIter end(void) const { return tasks_.end(); }

    bool isSafeToExecute(const ObstacleDistanceGrid& map);

    void loopThroughScript(void) { isLoopingThroughScript_ = !isLoopingThroughScript_; };

    bool isAtFinalTarget(void);

    bool shouldWaitForPrecondition(void) { return false; }; // TODO: Implement this!

    std::shared_ptr<MetricPlannerTask> getNextMetricPlannerTask(void);

private:

    std::string name_;
    std::vector<ScriptTask> tasks_;

    // for iterating through the script and targets within each script task
    bool       isActive_;
    bool       isLoopingThroughScript_;
    TaskIter   taskIt_;
    TargetIter targetIt_;
    std::vector<pose_t> currentTargets_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive &ar)
    {
        ar (name_,
            tasks_,
            isLoopingThroughScript_
        );
    }
};

} // mpepc
} // vulcan

DEFINE_SYSTEM_MESSAGE(mpepc::MetricPlannerScript, ("METRIC_PLANNER_SCRIPT"))

#endif // MPEPC_METRIC_PLANNER_SCRIPT_H
