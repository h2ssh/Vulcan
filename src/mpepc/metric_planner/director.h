/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of MetricPlannerDirector.
*/

#ifndef METRIC_PLANNER_DIRECTOR_H
#define METRIC_PLANNER_DIRECTOR_H

// system and utils
#include "system/director.h"
#include "utils/locked_double_buffer.h"
#include "utils/locked_queue.h"
#include "utils/locked_bool.h"
#include "utils/condition_variable.h"
#include "utils/mutex.h"
#include <memory>

// metric planner inputs and outputs
#include "hssh/local_metric/lpm.h"
#include "hssh/local_topological/local_topo_map.h"
#include "core/motion_state.h"
#include "tracker/dynamic_object_collection.h"
#include "mpepc/metric_planner/messages.h"
#include "mpepc/motion_controller/task/target.h" // the planner can only send the motion target task (for now at least)
#include "mpepc/motion_controller/messages.h"

// task and script processors
#include "mpepc/metric_planner/task/task.h"
#include "mpepc/metric_planner/script/script.h"

// other data processors and paramsters
#include "mpepc/grid/obstacle_distance_grid.h"
#include "mpepc/grid/obstacle_distance_grid_builder.h"
#include "mpepc/grid/visibility_analysis.h"
#include "mpepc/simulator/dynamic_object_filter.h"
#include "mpepc/simulator/dynamic_object_simulator.h"
#include "mpepc/trajectory/trajectory_planner.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "mpepc/metric_planner/params.h"
#include "mpepc/metric_planner/progress_checker.h"

#include "lcmtypes/mpepc/metric_pose_target_t.h"

// NOTE: For motion planning and generation in local metric spaces (non-topological spaces) there are
//       three classes of motion representation at three distinct time scales:
//       (1) Path, or an equivalent task manifold: This is valid for the entire lifetime of the task,
//           where this 'lifetime' can extend to infinity.
//       (2) Trajectory: This can be a time-stamped sequence of robot states for a finite time horizon,
//           or some abstraction of that. Represents an immediate motion in the workspace of the robot.
//       (3) Motor command: This can be a joystick command to the robot or equivalent velocity command,
//           which is typcally updated at > 50Hz.
//
//       In our setting, the metric planner is responsible for finding an optimal motion target, an abstraction of
//       a realizable, time-optimal trajectory that is safe and comfortable. This involves generating a relevant
//       task manifold as intermediate process.
//
//       Then the motion_controller takes this motion target, or trajectory instruction, and convert that to the
//       low-level command that is actually transmitted to the physical robot. This includes adding on-line feedback
//       commands to compensate for events that the metric planner have failed to anticipate, such as small bump
//       on the ground.

// NOTE: !!IMPORTANT!!
//       The planning takes a non trivial amount of time to process data, which includes the map and the states of
//       the robot and all dynamic objects around the robot. This processing time introduces delay which needs to
//       be compensated for reliable real time motion generation. We do this by enforcing a fixed update interval
//       and accurately predicting the state of the robot and other dynamic objects to the next update cycle.

namespace vulcan
{
namespace mpepc
{

const std::string kLessDebugInfoArgument("less-debug-info");
const std::string kLocalTopoMapArgument("topo-map");
const std::string kMetricMapArgument("map");

/**
* MetricPlannerDirector processes incoming data and requests. Using these data,
* the director orchestrates the calculation of a trajectory for the robot to follow.
* The trajectory is represented by a motion target, which is issued to the motion
* controller module that is responsible for generating motor commands for the robot.
* The director is also responsible for enforcing a regular update period.
*/
class MetricPlannerDirector : public system::Director
{
public:

    /**
    * Constructor for MetricPlannerDirector.
    */
    MetricPlannerDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    /**
    * Destructor for MetricPlannerDirector.
    */
    ~MetricPlannerDirector(void);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& producer) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& transmitter) override;
    void shutdown (system::ModuleCommunicator& transmitter) override;

    // data subscriptions
    void handleData(const hssh::LocalPerceptualMap&            lpm,            const std::string& channel);
    void handleData(const hssh::LocalTopoMap&                  ltm,            const std::string& channel);
    void handleData(const tracker::DynamicObjectCollection& trackedObjects,    const std::string& channel);
    void handleData(const motion_state_t&               robotState,     const std::string& channel);
    void handleData(const std::shared_ptr<MetricPlannerTask>&  task,           const std::string& channel);
    void handleData(const MetricPlannerScript&                 script,         const std::string& channel);
    void handleData(const metric_planner_command_message_t&    commandMessage, const std::string& channel);
    void handleData(const vulcan_lcm::metric_pose_target_t&    target,         const std::string& channel);

private:

    // methods
    void setMetricPlannerTiming(void);
    void updateTaskEnvironment(void);
    void processCommandMessages(void);
    bool isSafeToPlan(void);
    void handleScript(void);
    bool assignNewTask(void);
    void runPlanners(void);
    void enforceMetricPlannerTiming(void);
    void transmitCalculatedOutput(system::ModuleCommunicator& transmitter);

    bool haveEssentialData(void) const { return haveGrid_ && haveRobotState_; }; // metric planner will start only when it has the essential pieces of data
    void handleCommandMessage(const metric_planner_command_message_t& message);

    void openDataLog(void);
    void logData(void);

    // params structure for the metric planner
    metric_planner_params_t params_;

    // TODO: Load these from configuration files rather than hard-coding
    std::vector<laser_configuration_t> laserConfigs_;

    // input data buffers
    utils::LockedDoubleBuffer<hssh::LocalPerceptualMap>         lpmBuffer_;
    utils::LockedDoubleBuffer<hssh::LocalTopoMap>               topoBuffer_;
    utils::LockedDoubleBuffer<tracker::DynamicObjectCollection> trackedObjectsBuffer_;
    utils::LockedDoubleBuffer<motion_state_t>            robotStateBuffer_;
    utils::LockedQueue<metric_planner_command_message_t>        messageQueue_;

    // data indicators
    utils::LockedBool haveGrid_;       // LPM has been received at least once
    utils::LockedBool haveLTM_;         // local topo map has been received at least once
    utils::LockedBool haveRobotState_; // robot state has been received at least once
    utils::LockedBool inDynamicEnvironment_;    // flag indicating if a DynamicObjectCollection has ever been received
                                                // if so, then require tracked objects to not go stale
                                                // if not, then the safety check for tracked objects is bypassed

    // data handlers
    utils::ConditionVariable dataTrigger_;

    // script and task handling
    utils::Mutex scriptLock_; // lock for the received script
    utils::Mutex taskLock_;   // lock for the received task

    utils::LockedBool haveNewScript_;
    utils::LockedBool haveNewTask_;
    utils::LockedBool usingLCMTask_;        // flag indicating using task from LCM, thus send LCM version of planner status
    utils::LockedBool usingScript_;         // is a script currently controlling the robot?

    std::shared_ptr<MetricPlannerScript> receivedScript_;
    std::shared_ptr<MetricPlannerScript> activeScript_;

    // received task is a simple metric planner task.
    // when it becomes active, it converts to a much richer task manifold.
    std::shared_ptr<MetricPlannerTask> receivedTask_;
    std::shared_ptr<MetricPlannerTask> activeTask_;
    std::unique_ptr<TaskManifold>      activeManifold_;

    // obstacles in the map. both static and dynamic
    bool haveNewObstacleDistanceGrid_;
    ObstacleDistanceGridBuilder obstacleDistanceGridBuilder_;
    ObstacleDistanceGrid        obstacleDistanceGrid_;
    VisibilityAnalysis          visibility_;
    DynamicObjectFilter         objectFilter_;
    DynamicObjectSimulator      objectSimulator_;

    TrajectoryPlanner           trajectoryPlanner_;

    // logging and progress checking
    std::ofstream   dataLog;
    ProgressChecker progressChecker;

    // metric planner timing
    int64_t previousPlanReleaseTimeUs_;
    int64_t updateStartTimeUs_;
    int64_t planReleaseTimeUs_;
    int64_t averagePlanningTimeUs_; // average time the planner is taking
    int64_t lastGridUpdateTimeUs_;
    int64_t lastTransmissionTimeForObstacleDistanceGridUs_;

    // planner status indicators
    bool isPaused_; // Is the planner paused (by external command)? If so the planner is idle, but it retains all the data.
    bool isWaitingForPrecondition_; // if the script is waiting for some precondition before switching to the next task.
    metric_planner_status_message_t plannerStatus_;

    // dynamic object simulator output
    std::vector<dynamic_object_trajectory_t> dynObjects_;

    // trajectory planner output
    bool shouldOutputLessDebugInfo_; // debug information is always produced, but this option allows you to discard all evaluated trajectories but the final output.
    trajectory_planner_output_t     trajectoryPlannerOutput_;
    trajectory_planner_debug_info_t trajectoryPlannerDebugInfo_;

    // commands to the motion controller
    bool haveMotionControllerTask_;
    bool haveMotionControllerCommandMessage_;

    motion_target_t                       previousMotionTarget_;
    motion_target_t                       nextMotionTarget_;
    std::shared_ptr<MotionControllerTask> motionControllerTask_;
    motion_controller_command_message_t   motionControllerCommandMessage_;
};

} // mpepc
} // vulcan

#endif // METRIC_PLANNER_DIRECTOR_H
