/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \param    script.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of MetricPlannerScript.
 */

#include "mpepc/metric_planner/script/script.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "mpepc/metric_planner/script/script_task.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "utils/config_file_utils.h"
#include "utils/tagged_file.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>

// #define DEBUG_SCRIPT

namespace vulcan
{

namespace mpepc
{

const std::string SCRIPT_TAG("script");
const std::string SCRIPT_NAME_TAG("name");

const std::string SEQ_NUM_TAG("seq_num");

const std::string TASK_TAG("task");
const std::string TASK_NUM_TAG("num_poses");
const std::string TASK_POSE_TAG("pose");

// Types other than target kept around for legacy purposes, but they aren't really used
const std::string POSE_TAG("pose");
const std::string POSE_TARGET_TAG("target");

const std::string DOOR_TAG("door");
const std::string DOOR_ENTRY_TAG("entry");
const std::string DOOR_EXIT_TAG("exit");

const std::string ELEVATOR_TAG("elevator");
const std::string ELEVATOR_ENTRY_TAG("entry");
const std::string ELEVATOR_INSIDE_TAG("inside");
const std::string ELEVATOR_EXIT_TAG("exit");

using script_task_t = std::pair<int, ScriptTask>;

void write_script_info(const std::string& name, std::ofstream& out);
void write_task(const ScriptTask& task, int seqNum, std::ofstream& out);

std::string load_script_info(const utils::TaggedMap& info);
script_task_t load_task(const utils::TaggedMap& info);
script_task_t load_target(const utils::TaggedMap& info);
script_task_t load_door(const utils::TaggedMap& info);
script_task_t load_elevator(const utils::TaggedMap& info);

pose_t target_from_string(const std::string& string);


MetricPlannerScript::MetricPlannerScript(const std::string& filename)
{
    utils::TaggedFile tags(filename);

    auto script = tags.getNestedContents(SCRIPT_TAG);
    auto tasks = tags.getNestedContents(TASK_TAG);
    auto poses = tags.getNestedContents(POSE_TAG);
    auto doors = tags.getNestedContents(DOOR_TAG);
    auto elevators = tags.getNestedContents(ELEVATOR_TAG);

    name_ = load_script_info(script.front());

    // Load the tasks into a script keyed on the sequence number so I can iterate through tasks
    // and put them in tasks and they'll already be in the correct order
    std::map<int, ScriptTask> taskMap;

    for (auto task : tasks) {
        taskMap.insert(load_task(task));
    }

    for (auto pose : poses) {
        taskMap.insert(load_target(pose));
    }

    for (auto door : doors) {
        taskMap.insert(load_door(door));
    }

    for (auto elevator : elevators) {
        taskMap.insert(load_elevator(elevator));
    }

    for (auto& task : taskMap) {
        tasks_.push_back(task.second);
    }

    isActive_ = false;
    isLoopingThroughScript_ = true;
}


MetricPlannerScript::MetricPlannerScript(const std::string& name, const std::vector<ScriptTask>& tasks)
: name_(name)
, tasks_(tasks)
, isActive_(false)
, isLoopingThroughScript_(true)
{
}


bool MetricPlannerScript::isSafeToExecute(const ObstacleDistanceGrid& map)
{
    float kMinimumSafeDistance = 0.35;

    for (auto task : tasks_) {
        for (auto target : task.getTargets()) {
            Point<int> targetCell = map.positionToCell(target.toPoint());

            bool isCellInTheMap = map.isCellInGrid(targetCell);
            bool isCellAwayFromWalls = map.getObstacleDistance(targetCell) > kMinimumSafeDistance;

#ifdef DEBUG_SCRIPT
            std::cout << "\nDEBUG: NavigationScript: TargetCell: (" << targetCell.x << " , " << targetCell.y << ")"
                      << '\n';
            std::cout << "DEBUG: NavigationScript: Is the cell in the map?: (" << isCellInTheMap << ")" << '\n';
            std::cout << "DEBUG: NavigationScript: Is the cell away from walls?: (" << isCellAwayFromWalls << ")"
                      << '\n';
#endif

            if (!isCellInTheMap || !isCellAwayFromWalls) {
                return false;
            }
        }
    }

    return true;
}


bool MetricPlannerScript::isAtFinalTarget(void)
{
    if (isActive_ && !isLoopingThroughScript_) {
        //         assert(!tasks.empty());
        if (taskIt_ == tasks_.cend() - 1) {
            //             assert(!currentTargets.empty());
            if (targetIt_ == currentTargets_.cend() - 1) {
                isActive_ = false;
                return true;
            }
        }
    }

    return false;
}

std::shared_ptr<MetricPlannerTask> MetricPlannerScript::getNextMetricPlannerTask(void)
{
    // NOTE: the only type of metric planner task that can be generated from the
    // script is the navigation task (for now). If the script gets extended to
    // include other types, e.g. person pacing, then this needs to get extended
    // as well.
    if (!isActive_) {
        // initialize iterators and exit if first update
        assert(!tasks_.empty());   // check for non-empty container at creation
        taskIt_ = tasks_.cbegin();

        currentTargets_ = taskIt_->getTargets();
        assert(!currentTargets_.empty());   // check for non-empty container at creation
        targetIt_ = currentTargets_.cbegin();

        isActive_ = true;
    } else {
        pose_t previousTarget = *targetIt_;

        targetIt_++;   // forward step a target
        if (targetIt_ == currentTargets_.cend()) {
            taskIt_++;   // if at the end of the targets forward step a task
            if (taskIt_ == tasks_.cend()) {
                if (isLoopingThroughScript_) {
                    // re-initialize targets and tasks if in loop mode
                    taskIt_ = tasks_.cbegin();
                    currentTargets_ = taskIt_->getTargets();
                    targetIt_ = currentTargets_.cbegin();
                } else {
                    // return previous target and a warning sign if at the end of the vector
                    std::cout << "\nWarning!: MetricPlannerScript: No more task to be executed!\n";
                    return std::shared_ptr<MetricPlannerTask>(new NavigationTask(previousTarget.toPoint()));
                }
            } else   // handle next task in line
            {
                // renew current targets
                currentTargets_ = taskIt_->getTargets();
                assert(!currentTargets_.empty());
                targetIt_ = currentTargets_.cbegin();
            }
        }
    }

    // return a metric planner task generated from the target pose
    return std::shared_ptr<MetricPlannerTask>(new NavigationTask(targetIt_->toPoint()));
}


bool MetricPlannerScript::save(const std::string& filename) const
{
    std::ofstream out(filename);

    if (!out.is_open()) {
        return false;
    }

    write_script_info(name_, out);

    for (std::size_t n = 0; n < tasks_.size(); ++n) {
        write_task(tasks_[n], n, out);
    }

    return true;
}


void write_script_info(const std::string& name, std::ofstream& out)
{
    out << '<' << SCRIPT_TAG << ">\n"
        << "\t<" << SCRIPT_NAME_TAG << '>' << name << "</" << SCRIPT_NAME_TAG << ">\n"
        << "</" << SCRIPT_TAG << ">\n";
}


void write_task(const ScriptTask& task, int seqNum, std::ofstream& out)
{
    auto targets = task.getTargets();
    auto names = task.getTargetNames();

    assert(targets.size() == names.size());

    out << '<' << TASK_TAG << ">\n"
        << "\t<" << SEQ_NUM_TAG << '>' << seqNum << "</" << SEQ_NUM_TAG << ">\n"
        << "\t<" << TASK_NUM_TAG << '>' << targets.size() << "</" << TASK_NUM_TAG << ">\n";

    for (std::size_t n = 0; n < targets.size(); ++n) {
        auto& target = targets[n];
        out << "\t<" << TASK_POSE_TAG << '>' << n << ' ' << target.x << ' ' << target.y << ' ' << target.theta << ' '
            << names[n] << "</" << TASK_POSE_TAG << ">\n";
    }

    out << "</" << TASK_TAG << ">\n";
}


std::string load_script_info(const utils::TaggedMap& info)
{
    auto name = utils::TaggedFile::getTagValues(info, SCRIPT_NAME_TAG);

    assert(!name.empty());

    return name.front();
}


script_task_t load_task(const utils::TaggedMap& info)
{
    auto seqNumStr = utils::TaggedFile::getTagValues(info, SEQ_NUM_TAG);
    auto numTargetsStr = utils::TaggedFile::getTagValues(info, TASK_NUM_TAG);
    auto targets = utils::TaggedFile::getTagValues(info, TASK_POSE_TAG);

    int seqNum = std::stoi(seqNumStr.front());
    int numTargets = std::stoi(numTargetsStr.front());

    std::vector<pose_t> poses(numTargets);
    std::vector<std::string> names(numTargets);

    for (auto str : targets) {
        std::istringstream in(str);
        int idx = 0;
        in >> idx;
        in >> poses[idx].x >> poses[idx].y >> poses[idx].theta >> names[idx];
    }

    return std::make_pair(seqNum, ScriptTask(poses, names, "Task"));
}


script_task_t load_target(const utils::TaggedMap& info)
{
    auto num = utils::TaggedFile::getTagValues(info, SEQ_NUM_TAG);
    auto target = utils::TaggedFile::getTagValues(info, POSE_TARGET_TAG);

    for (auto tags : info) {
        std::cout << tags.first << ' ' << tags.second << '\n';
    }

    assert(!num.empty());
    assert(!target.empty());

    return std::make_pair(std::stoi(num.front()), ScriptTask(target_from_string(target.front()), "Pose"));
}


script_task_t load_door(const utils::TaggedMap& info)
{
    auto num = utils::TaggedFile::getTagValues(info, SEQ_NUM_TAG);
    auto entry = utils::TaggedFile::getTagValues(info, DOOR_ENTRY_TAG);
    auto exit = utils::TaggedFile::getTagValues(info, DOOR_EXIT_TAG);

    assert(!num.empty());
    assert(!entry.empty());
    assert(!exit.empty());

    std::vector<pose_t> poses;
    std::vector<std::string> names;

    poses.push_back(target_from_string(entry.front()));
    names.push_back("Entry");

    poses.push_back(target_from_string(exit.front()));
    names.push_back("Exit");

    return std::make_pair(std::stoi(num.front()), ScriptTask(poses, names, "Door"));
}


script_task_t load_elevator(const utils::TaggedMap& info)
{
    auto num = utils::TaggedFile::getTagValues(info, SEQ_NUM_TAG);
    auto entry = utils::TaggedFile::getTagValues(info, ELEVATOR_ENTRY_TAG);
    auto inside = utils::TaggedFile::getTagValues(info, ELEVATOR_INSIDE_TAG);
    auto exit = utils::TaggedFile::getTagValues(info, ELEVATOR_EXIT_TAG);

    assert(!num.empty());
    assert(!entry.empty());
    assert(!inside.empty());
    assert(!exit.empty());

    std::vector<pose_t> poses;
    std::vector<std::string> names;

    poses.push_back(target_from_string(entry.front()));
    names.push_back("Entry");

    poses.push_back(target_from_string(inside.front()));
    names.push_back("Inside");

    poses.push_back(target_from_string(exit.front()));
    names.push_back("Exit");

    return std::make_pair(std::stoi(num.front()), ScriptTask(poses, names, "Elevator"));
}


pose_t target_from_string(const std::string& string)
{
    auto split = utils::split_into_strings(string, ' ');

    assert(split.size() == 3);

    pose_t target;
    target.x = std::stof(split[0]);
    target.y = std::stof(split[1]);
    target.theta = std::stof(split[2]);

    return target;
}

}   // namespace mpepc
}   // namespace vulcan
