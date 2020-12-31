/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface.h
* \author   Collin Johnson
* 
* Declaration of NavigationInterface.
*/

#ifndef PLANNER_INTERFACE_NAVIGATION_INTERFACE_H
#define PLANNER_INTERFACE_NAVIGATION_INTERFACE_H

#include "planner/interface/decision_interface.h"
#include "planner/interface/goal_interface.h"

namespace vulcan
{
namespace mpepc { struct metric_planner_status_message_t; }
namespace system { class SystemCommunicator; }
namespace planner
{

class DecisionAction;
class GoalAction;


/**
* NavigationInterface provides a high-level interface to navigating using the HSSH. The interface provides access to
* the Decision and Goal levels of the HSSH.
*
* The NavigationInterface coordinates the behavior of the Decision and Goal interfaces. The general behavior is:
*
*   - In any area, there is a local set of Decisions available to the robot, which it can choose from
*   - In any environment, there is a global set of Goals available to the robot, which it can choose from
*
* When navigating, the Decision interface takes predecent over the Goal interface. If the robot is driving to a Goal
* and a Decision command is given, then the Goal action is deferred. The user can either resume the Goal action later
* or issue a new Goal command as desired. The Goal action will not be resumed automatically.
*/
class NavigationInterface
{
public:

    /**
    * Constructor for NavigationInterface.
    *
    * \param    moduleName          Name of the module running the interface
    */
    NavigationInterface(std::string moduleName);

    /**
    * Destructor for NavigationInterface.
    */
    ~NavigationInterface(void);

    /**
    * hasFinishedAction checks if the current action being taken by the robot has been finished.
    */
    bool hasFinishedAction(const mpepc::metric_planner_status_message_t& status);

    //////////////// Decision navigation ////////////////

    /**
    * goToDecision commands the robot to go to the desired decision.
    *
    * \param    decision        Decision to go to
    * \param    robotPose       Current pose of the robot
    * \param    communicator    Communicator to use for getting to the decision
    */
    void goToDecision(const Decision& decision,
                      const pose_t& robotPose,
                      system::SystemCommunicator& communicator);

    /**
    * determineDecisions determines the decisions available to the robot in its current topological location. These
    * decisions are described generically as Decisions. The user should select among these decisions.
    *
    * If the LocalLocation hasn't changed, then no new decisions will be found.
    *
    * The determined decisions can be accessed via the iterators.
    *
    * \param    map         Local topological description of the nearby environment
    * \param    location    Location of the robot within this map
    * \param    pose        Pose of the robot in the LPM
    * \return   Number of new decisionsfound for the current location, which will be 0 if the location is unchanged from
    *           the previous update.
    */
    int determineDecisions(const hssh::LocalTopoMap& map,
                           const hssh::LocalLocation& location,
                           const pose_t& pose);

    // Iterate over available decisions
    DecisionInterface::DecisionIter beginDecisions(void) const { return decisionInterface_.begin(); }
    DecisionInterface::DecisionIter endDecisions(void) const { return decisionInterface_.end(); }
    std::size_t sizeDecisions(void) const { return decisionInterface_.size(); }
    bool emptyDecisions(void) const { return decisionInterface_.size() == 0; }

    //////////////// Goal navigation ////////////////

    /**
    * goToGoal commands the robot to go to the specified goal.
    *
    * \param    goal            Goal to go to
    * \param    topoMap         LocalTopoMap to use for creating the goal (optional, nullptr will use globalPose)
    * \param    communicator    Communicator to use for getting to the goal
    */
    void goToGoal(const Goal& goal,
                  const hssh::LocalTopoMap* topoMap,
                  system::SystemCommunicator& communicator);

    /**
    * goToGoal commands the robot to go to the specified goal.
    *
    * \param    goalName        Name of the goal to go to
    * \param    topoMap         LocalTopoMap to use for creating the goal (optional, nullptr will use globalPose)
    * \param    communicator    Communicator to use for getting to the goal
    * \return   True if a goal exists with the given name, so the command could be sent.
    */
    bool goToGoal(const std::string& goalName,
                  const hssh::LocalTopoMap* topoMap,
                  system::SystemCommunicator& communicator);

    /**
    * currentGoal retrieves the current goal that has been issued to the robot, if one has been assigned.
    */
    boost::optional<Goal> currentGoal(void) const;

    /**
    * hasDeferredGoal checks to see if a Goal action has been deferred while Decision-level commands have been
    * executing.
    */
    bool hasDeferredGoal(void) const;

    /**
    * resumeDeferredGoal resumes motion to the current deferred Goal.
    *
    * \pre  hasDeferredGoal == true
    * \param    communicator            Communicator to use for issuing the command
    */
    void resumeDeferredGoal(system::SystemCommunicator& communicator);

    /**
    * addGoalGlobalPose adds a new goal to take the robot to some global pose in the environment.
    *
    * \param    name            Name of the goal
    * \param    pose            Pose associated with the goal
    */
    void addGoalGlobalPose(const std::string& name, const pose_t& pose);

    /**
    * addGoalLocalArea adds a new goal to take the robot to some local area in the environment.
    *
    * \param    name            Name of the goal
    * \param    id              Id associated with the area
    */
    void addGoalLocalArea(const std::string& name, hssh::LocalArea::Id id);

    // Iterate over available goals
    GoalInterface::GoalIter beginGoals(void) const { return goalInterface_.begin(); }
    GoalInterface::GoalIter endGoals(void) const { return goalInterface_.end(); }
    std::size_t sizeGoals(void) const { return goalInterface_.size(); }
    bool emptyGoals(void) const { return goalInterface_.empty(); }

    /**
    * saveGoals saves the current goals to a file.
    *
    * \param    filename            Name of the file in which to save the goals
    * \return   True if the goals were saved successfully.
    */
    bool saveGoals(const std::string& filename);

    /**
    * loadGoals loads a set of goals from a file.
    *
    * \param    filename            Name of the file from which to load the goals
    * \return   True if the goals were loaded.
    */
    bool loadGoals(const std::string& filename);

private:

    std::string moduleName_;

    DecisionInterface decisionInterface_;
    GoalInterface goalInterface_;

    std::unique_ptr<DecisionAction> decisionAction_;
    std::unique_ptr<GoalAction> goalAction_;
    bool goalIsDeferred_;
};

} // namespace planner
} // namespace vulcan

#endif // PLANNER_INTERFACE_NAVIGATION_INTERFACE_H
