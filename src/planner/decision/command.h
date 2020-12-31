/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     command.h
 * \author   Collin Johnson
 *
 * Declaration of DecisionCommand.
 */

#ifndef PLANNER_DECISION_COMMAND_H
#define PLANNER_DECISION_COMMAND_H

#include <cereal/access.hpp>
#include <memory>
#include <vector>

namespace vulcan
{
namespace planner
{

class DecisionPlanner;
class DecisionTask;

/**
 * DecisionCommand issues a command to the DecisionPlanner. The command provides tasks for the DecisionPlanner to
 * execute. The command provides four options for issuing tasks:
 *
 *   1) Set     -- clear any existing tasks and execute only those provided by the command
 *   2) Append  -- perform these tasks after any existing tasks have executed
 *   3) Prepend -- perform these tasks before any existing tasks
 *   4) Clear   -- erase all tasks, i.e. cancel any actions being taken by the planner
 *
 * In most cases, the Set option should be used to ensure that the contents of the queue are exactly what you expect.
 * Multiple modules might use the Decision planner, so they might clobber each other. If only one module is known to be
 * controlling the Decision level, then Append/Prepend are good for user interaction because they allow for
 * incrementally navigating through the environment without the need to check the completion status of previously issued
 * commands.
 *
 * A DecisionCommand can only be issued once. The tasks it contains are moved into the DecisionPlanner.
 *
 * NOTE: Possible extension is to allow a priority system to exist for the commands being sent to the Decision planner.
 */
class DecisionCommand
{
public:
    using Tasks = std::vector<std::shared_ptr<DecisionTask>>;

    enum IssueMode
    {
        Set,
        Append,
        Prepend,
        Clear
    };

    /**
     * Constructor for DecisionCommand.
     *
     * If the mode is Clear, then the tasks won't be assigned. This is equivalent to using Set with no tasks.
     *
     * \param    source          Source of the command
     * \param    mode            Mode to use for issuing the command
     * \param    tasks           Tasks to be performed by the robot (optional, not needed for Clear, but for all others)
     */
    DecisionCommand(const std::string& source, IssueMode mode, const Tasks& tasks);

    /**
     * Default constructor for DecisionCommand.
     *
     * Creates a Clear command.
     *
     * \param    source          Source of the command
     */
    explicit DecisionCommand(const std::string& source);

    /**
     * issue issues the command to the provided DecisionPlanner.
     *
     * \param    planner         Planner instance to issue commands to
     */
    void issue(DecisionPlanner& planner);

private:
    std::string source_;
    IssueMode mode_;
    Tasks tasks_;

    // Serialization support
    friend class cereal::access;

    template <typename Archive>
    void serialize(Archive& ar)
    {
        ar& tasks_;
    }
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_DECISION_COMMAND_H
