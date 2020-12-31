/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     logical_interface_experiment.h
 * \author   Collin Johnson
 *
 * Declaration of LogicalInterfaceExperiment.
 */

#ifndef UI_LOGICAL_LOGICAL_INTERFACE_EXPERIMENT_H
#define UI_LOGICAL_LOGICAL_INTERFACE_EXPERIMENT_H

#include "hssh/global_topological/topological_map.h"
#include "ui/logical/params.h"
#include <map>

namespace vulcan
{
namespace hssh
{
struct GlobalLocation;
}

namespace ui
{

/**
 * logical_task_level_t describes at which level of the logical interface a task
 * will be occurring.
 */
enum logical_task_level_t
{
    LOGICAL_DECISION = 0,
    LOGICAL_GOAL = 1,
    LOGICAL_ANY = 2   ///< No restrictions on the level to use. Leave to user.
};

/**
 * logical_experiment_task_t describes a task to be performed during an experiment. A
 * task consists of moving from the current place to a goal place using a specified
 * level of the logical interface.
 */
struct logical_experiment_task_t
{
    int id;                       ///< Identifier for the task -- keep tabs on where in the sequence the experiment is
    int goalId;                   ///< Id of the end place
    std::string description;      ///< Text description of the goal
    logical_task_level_t level;   ///< Level at which command should take place

    logical_experiment_task_t(void) : id(-1), goalId(-1), level(LOGICAL_ANY) { }

    logical_experiment_task_t(int id, int goalId, const std::string& description, logical_task_level_t level)
    : id(id)
    , goalId(goalId)
    , description(description)
    , level(level)
    {
    }
};

/**
 * LogicalInterfaceExperiment organizes and gathers statistics for an experiment conducted
 * using a physical implementation of the logical interface for navigating using the HSSH.
 *
 * An experiment for the logical interface consists of a series of goals to navigate to in
 * a fixed topological map. Each goal is assigned a task to complete, navigating using either
 * the Decision level or Goal level of the logical interface.
 *
 * For each task, the following information is recorded:
 *
 *   - start time for selecting the command
 *   - end time for selecting the command
 *   - start time for executing the command
 *   - finish time for executing the command
 *
 * While executing a command, queries to isTaskComplete() should be made. Once a task is complete,
 * nextTask() will retrieve the next experiment task to be executed.
 *
 * An experiment can either be setup to be a randomly selected number of goals or a predefined series loaded
 * from a file. The experiment file will be the sequential list of goals, which will be executed in the order
 * specified in the file:
 *
 *   goal_id level
 *   goal_id level
 *   etc...
 *
 * The map to be used for the experiment is in the format described in hssh/utils/topological_map_io.h.
 *
 * Along with the map file, the experiment needs a place description file. This file provides a text description
 * for each place in the map. This text description is intended to be more human-friendly when selecting places
 * in the map. The UI can access these descriptions via getPlaceDescriptions(). The file has a very simple format:
 *
 *   place_id description
 *   place_id description
 *   etc...
 *
 * Right now, the evaluation file is simple. It contains for each task:
 *
 *   goal_id level total_selection_time total_task_time
 *
 * NOTE: There will be more information attached in the future as more diagnostics are determined.
 *
 * The config file parameters for LogicalInterfaceExperiment are:
 *
 *   [LogicalInterfaceExperimentParameters]
 *   experiment_map        = filename of the map to use for the experiment
 *   place_descriptions    = filename with the placeId->string description file for the experiment map
 *   experiment_goals      = random or 'name of goal file'
 *   num_random_goals      = number of random goals to generate
 *   random_goal_filename  = file in which to save the generated goals for later analysis
 *   experiment_mode       = 'decision', 'goal', 'any', 'mixed'
 *   mixed_mode_goal_ratio = [0,1] in mix mode, number 0-1 is selected. All numbers below the ratio will be assigned
 * 'goal'
 *
 */
class LogicalInterfaceExperiment
{
public:
    /**
     * Constructor for LogicalInterfaceExperiment.
     *
     * \param    params                  Parameters for running the experiment
     * \param    evaluationFilename      Name of the file in which to save the evaluation results
     */
    LogicalInterfaceExperiment(const logical_interface_experiment_params_t& params,
                               const std::string& evaluationFilename);

    /**
     * Destructor for LogicalInterfaceExperiment.
     */
    ~LogicalInterfaceExperiment(void);

    /**
     * getExperimentMapFilename retrieves the filename of the map to be used for the experiment.
     */
    std::string getExperimentMapFilename(void) const { return params.experimentMap; }

    /**
     * getExperimentMap retrieves the map used for the experiment.
     */
    const hssh::TopologicalMap& getExperimentMap(void) const { return map; }

    /**
     * getPlaceDescriptions retrieves a mapping of description->place id.
     */
    const std::map<std::string, int>& getPlaceDescriptions(void) const { return descriptionToId; }

    /**
     * hasNextTask loads the next task in the experiment.
     *
     * \param    task            Task that was loaded (output)
     * \return   True if there was another task to be performed. False otherwise.
     */
    bool hasNextTask(void) const;

    /**
     * nextTask loads the next task in the experiment.
     *
     * \return   Next task. If there is no next task, the most recent task will be returned.
     */
    logical_experiment_task_t nextTask(void);

    /**
     * isTaskComplete checks to see if the current task is complete. The task is completed once
     * the goal is reached.
     *
     * \param    state           State of robot in topo map
     */
    bool isTaskComplete(const hssh::GlobalLocation& location) const;

    /**
     * startedSelection should be called when selection for the specified task is started.
     *
     * \param    task            Task for which selection has started
     */
    void startedSelection(const logical_experiment_task_t& task);

    /**
     * finishedSelection should be called when selection for the specified task has finished.
     *
     * \param    task            Task for which selection has finished
     */
    void finishedSelection(const logical_experiment_task_t& task);

    /**
     * startedTask should be called when the specified task is started.
     *
     * \param    task            Task which has started
     */
    void startedTask(const logical_experiment_task_t& task);

    /**
     * finishedTask should be called when the specified task has finished.
     *
     * \param    task            Task which has finished
     */
    void finishedTask(const logical_experiment_task_t& task);

    /**
     * saveExperimentTaskStats saves the statistics calculated for each task during execution of the experiment.
     */
    void saveExperimentTaskStats(void);

private:
    enum task_state_t
    {
        WAIT_TO_START,
        STARTED_SELECTION,
        FINISHED_SELECTION,
        STARTED_TASK,
        FINISHED_TASK
    };

    struct task_stats_t
    {
        int id;

        int64_t selectionStart;
        int64_t selectionFinish;
        int64_t totalSelectionTime;

        int64_t taskStart;
        int64_t taskFinish;
        int64_t totalTaskTime;

        task_stats_t(int id = -1) : id(id), totalSelectionTime(0), totalTaskTime(0) { }
    };

    void loadExperimentMap(const std::string& file);
    void loadPlaceDescriptions(const std::string& file);
    void validateDescriptions(void);
    void loadExperimentTasks(const std::string& file);
    bool isValidTask(int goalId, int level);
    void generateRandomGoals(void);
    logical_task_level_t newTaskLevel(void);
    void saveRandomGoals(void);

    task_stats_t& activeStats(void) { return stats[activeTaskIndex]; }

    std::vector<logical_experiment_task_t> tasks;

    std::size_t activeTaskIndex;
    task_state_t taskState;

    std::vector<task_stats_t> stats;

    hssh::TopologicalMap map;
    std::vector<int> placeIds;
    std::map<std::string, int> descriptionToId;
    std::map<int, std::string> idToDescription;

    std::string evaluationFile;
    logical_interface_experiment_params_t params;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_LOGICAL_LOGICAL_INTERFACE_EXPERIMENT_H
