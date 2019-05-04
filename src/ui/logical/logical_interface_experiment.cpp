/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logical_interface_experiment.cpp
* \author   Collin Johnson
*
* Definition of LogicalInterfaceExperiment.
*/

#include <ui/logical/logical_interface_experiment.h>
#include <hssh/global_topological/topological_map.h>
#include <hssh/utils/topological_map_io.h>
#include <utils/timestamp.h>
#include <fstream>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace ui
{

const std::string USE_RANDOM_GOALS("random");
const std::string RUN_DECISION    ("decision");
const std::string RUN_GOAL        ("goal");
const std::string RUN_ANY         ("any");
const std::string RUN_MIXED       ("mixed");


LogicalInterfaceExperiment::LogicalInterfaceExperiment(const logical_interface_experiment_params_t& params, const std::string& evaluationFilename)
    : activeTaskIndex(0)
    , evaluationFile(evaluationFilename)
    , params(params)
{
    loadExperimentMap(params.experimentMap);
    loadPlaceDescriptions(params.placeDescriptions);
}


LogicalInterfaceExperiment::~LogicalInterfaceExperiment(void)
{
}


bool LogicalInterfaceExperiment::hasNextTask(void) const
{
    return activeTaskIndex < tasks.size();
}


logical_experiment_task_t LogicalInterfaceExperiment::nextTask(void)
{
    if(activeTaskIndex < tasks.size())
    {
        taskState = WAIT_TO_START;

        return tasks[activeTaskIndex];
    }

    return logical_experiment_task_t();
}


bool LogicalInterfaceExperiment::isTaskComplete(const hssh::GlobalLocation& state) const
{
    return taskState == FINISHED_TASK;
}


void LogicalInterfaceExperiment::startedSelection(const logical_experiment_task_t& task)
{
//     assert(task.id == taskStats.id);

    activeStats().selectionStart = utils::system_time_us();

    taskState = STARTED_SELECTION;
}


void LogicalInterfaceExperiment::finishedSelection(const logical_experiment_task_t& task)
{
//     assert(task.id == taskStats.id);

    // There can be more than one selection per task, especially during the Decision phase
    activeStats().selectionFinish     = utils::system_time_us();
    activeStats().totalSelectionTime += activeStats().selectionFinish - activeStats().selectionStart;

    taskState = FINISHED_SELECTION;
}


void LogicalInterfaceExperiment::startedTask(const logical_experiment_task_t& task)
{
//     assert(task.id == taskStats.id);

    activeStats().taskStart = utils::system_time_us();

    taskState = STARTED_TASK;
}


void LogicalInterfaceExperiment::finishedTask(const logical_experiment_task_t& task)
{
//     assert(task.id == taskStats.id);

    activeStats().taskFinish    = utils::system_time_us();
    activeStats().totalTaskTime = activeStats().taskFinish - activeStats().taskStart;

    taskState = FINISHED_TASK;
    ++activeTaskIndex;
}


void LogicalInterfaceExperiment::loadExperimentMap(const std::string& file)
{
    map = hssh::TopologicalMapIO::load(file);

    auto places = map.getPlaces();

    for(auto placeIt = places.begin(), placeEnd = places.end(); placeIt != placeEnd; ++placeIt)
    {
        placeIds.push_back(placeIt->first);
    }
}


void LogicalInterfaceExperiment::loadPlaceDescriptions(const std::string& file)
{
    std::ifstream in(file);

    if(!in.is_open())
    {
        std::cerr<<"ERROR:LogicalInterfaceExperiment: Failed to open place description file: "<<file<<std::endl;
        assert(in.is_open());
    }

    std::string description;
    int         placeId = 0;

    while(!in.eof() && in.good())
    {
        in >> placeId >> description;

        descriptionToId.insert(std::make_pair(description, placeId));
        idToDescription.insert(std::make_pair(placeId, description));
    }
}


void LogicalInterfaceExperiment::validateDescriptions(void)
{
    // The validation goes through the place descriptions and the places in the map to see if there is a description for
    // each place. If a description isn't included for a particular place, then raise a warning, but otherwise continue
    // with execution of the program because not being able to select a particular place might be intentional

    for(auto idIt = placeIds.begin(), idEnd = placeIds.end(); idIt != idEnd; ++idIt)
    {
        if(idToDescription.find(*idIt) == idToDescription.end())
        {
            std::cerr<<"WARNING:LogicalInterfaceExperiment: Failed to find description for place "<<*idIt<<'\n';
        }
    }
}


void LogicalInterfaceExperiment::loadExperimentTasks(const std::string& file)
{
    std::ifstream in(file);

    if(!in.is_open())
    {
        std::cerr<<"ERROR:LogicalInterfaceExperiment: Failed to open experiment goal file: "<<file<<std::endl;
        assert(in.is_open());
    }

    int taskId = 0;
    int goalId = 0;
    int level  = 0;

    while(!in.eof() && in.good())
    {
        in >> goalId >> level;

        if(isValidTask(goalId, level))
        {
            tasks.push_back(logical_experiment_task_t(taskId++, goalId, idToDescription[goalId],static_cast<logical_task_level_t>(level)));
        }
        else
        {
            std::cerr<<"WARNING:LogicalInterfaceExperiment: Invalid task in "<<file<<':'<<goalId<<' '<<level<<'\n';
        }
    }
}


bool LogicalInterfaceExperiment::isValidTask(int goalId, int level)
{
    // A valid task has a goal that is in collection of place ids and has a valid level
    return (std::find(placeIds.begin(), placeIds.end(), goalId) != placeIds.end()) &&
           (level >= 0 && level <= 2);
}


void LogicalInterfaceExperiment::generateRandomGoals(void)
{
    int         taskId   = 0;
    std::size_t lastGoal = -1;

    while(tasks.size() < params.numRandomTasks)
    {
        std::size_t newGoal = drand48()*placeIds.size();

        if((newGoal != lastGoal) && (newGoal < placeIds.size()))
        {
            tasks.push_back(logical_experiment_task_t(taskId++, placeIds[newGoal], idToDescription[placeIds[newGoal]],newTaskLevel()));

            lastGoal = newGoal;
        }
    }

    saveRandomGoals();
}


logical_task_level_t LogicalInterfaceExperiment::newTaskLevel(void)
{
    logical_task_level_t level;

    if(params.experimentMode == RUN_ANY)
    {
        level = LOGICAL_ANY;
    }
    else if(params.experimentMode == RUN_DECISION)
    {
        level = LOGICAL_DECISION;
    }
    else if(params.experimentMode == RUN_GOAL)
    {
        level = LOGICAL_GOAL;
    }
    else if(params.experimentMode == USE_RANDOM_GOALS)
    {
        level = (drand48() < params.mixedModeGoalRatio) ? LOGICAL_GOAL : LOGICAL_DECISION;
    }
    else // unknown mode, so just use a default and throw out a warning
    {
        std::cerr<<"WARNING:LogicalInterfaceExperiment: Unknown experiment mode, "<<params.experimentMode<<", defaulting to LOGICAL_DECISION\n";
        level = LOGICAL_DECISION;
    }

    return level;
}


void LogicalInterfaceExperiment::saveRandomGoals(void)
{
    if(params.randomTasksFilename.empty())
    {
        return;
    }

    std::ofstream out(params.randomTasksFilename);

    if(!out.is_open())
    {
        std::cerr<<"ERROR:LogicalInterfaceExperiment:Unable to open random goal file, "<<params.randomTasksFilename<<". Not saving the generated goals.\n";
        return;
    }

    for(auto taskIt = tasks.begin(), taskEnd = tasks.end(); taskIt != taskEnd; ++taskIt)
    {
        out << taskIt->goalId << ' ' << static_cast<int>(taskIt->level) << '\n';
    }
}


void LogicalInterfaceExperiment::saveExperimentTaskStats(void)
{

}

} // namespace ui
} // namespace vulcan
