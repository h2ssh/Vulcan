/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     director.cpp
 * \author   Jong Jin Park and Collin Johnson
 *
 * Definition of MetricPlannerDirector.
 */

#include "mpepc/metric_planner/director.h"
#include "hssh/local_topological/areas/serialization.h"
#include "lcmtypes/mpepc/metric_pose_target_status_t.h"
#include "mpepc/manifold/task_manifold.h"
#include "mpepc/metric_planner/task/serialization.h"
#include "system/module_communicator.h"
#include "tracker/objects/serialization.h"
#include "utils/command_line.h"
#include "utils/serialized_file_io.h"
#include "utils/timestamp.h"
#include <iostream>
#include <time.h>

// #define DEBUG_DIRECTOR
// #define DEBUG_DIRECTOR_TIMING
// #define DEBUG_GRID_BUILDING
// #define LOG_DATA

namespace vulcan
{

namespace mpepc
{

MetricPlannerDirector::MetricPlannerDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
: params_(load_metric_planner_params(config))
, haveGrid_(false)
, haveLTM_(false)
, haveRobotState_(false)
, inDynamicEnvironment_(false)
, dataTrigger_(false)
, haveNewScript_(false)
, haveNewTask_(false)
, usingLCMTask_(false)
, haveNewObstacleDistanceGrid_(false)
, obstacleDistanceGridBuilder_(params_.obstacleDistanceGridBuilderParams)
, obstacleDistanceGrid_()
, objectFilter_(params_.dynamicObjectFilterParams)
, objectSimulator_(params_.dynamicObjectSimulatorParams)
, trajectoryPlanner_(params_.trajectoryPlannerParams)
, previousPlanReleaseTimeUs_(0)
, updateStartTimeUs_(0)
, planReleaseTimeUs_(0)
, averagePlanningTimeUs_(0)
, lastGridUpdateTimeUs_(0)
, lastTransmissionTimeForObstacleDistanceGridUs_(0)
, isPaused_(false)
, isWaitingForPrecondition_(false)
, shouldOutputLessDebugInfo_(commandLine.argumentExists(kLessDebugInfoArgument))
, haveMotionControllerTask_(false)
, haveMotionControllerCommandMessage_(false)
, previousMotionTarget_()
, nextMotionTarget_()
{
    plannerStatus_.taskId = MetricPlannerTask::kInvalidId;
    plannerStatus_.status = metric_planner_status_t::IDLE;

    // synchronize simulator time steps
    objectSimulator_.setTimeStep(params_.simulatorTimeStep);
    objectSimulator_.setTimeLength(params_.trajectoryTimeLength);
    trajectoryPlanner_.setTimeStep(params_.simulatorTimeStep);
    trajectoryPlanner_.setTimeLength(params_.trajectoryTimeLength);

    if (commandLine.argumentExists(kLocalTopoMapArgument)) {
        hssh::LocalTopoMap map;
        if (utils::load_serializable_from_file(commandLine.argumentValue(kLocalTopoMapArgument), map)) {
            topoBuffer_.write(map);
            haveLTM_ = true;
        }
    }

    if (commandLine.argumentExists(kMetricMapArgument)) {
        std::string lpmName = commandLine.argumentValue(kMetricMapArgument);

        hssh::LocalPerceptualMap map;
        if (utils::load_serializable_from_file(lpmName, map)) {
            lpmBuffer_.write(map);
            haveGrid_ = true;
        } else {
            std::cerr << "ERROR: MetricPlannerDirector: Failed to load requested LPM: " << lpmName << '\n';
        }
    }

    // Manually create the lasers from the config files
    //     struct laser_configuration_t
    //     {
    //         pose_t offset;               ///< Offset of the laser in the robot frame
    //         utils::ray_trace_range_t range;     ///< Range of angles to scan in the laser frame
    //     };

    laser_configuration_t frontConfig;
    frontConfig.offset.x = 0.44;
    frontConfig.offset.y = 0.0;
    frontConfig.offset.theta = 0.0;
    frontConfig.range = utils::ray_trace_range_t(-135.0 * M_PI / 180.0, std::size_t(1080 / 4), M_PI / 180.0);
    laserConfigs_.push_back(frontConfig);

    //     laser_configuration_t frontConfig;
    //     frontConfig.offset.x = 0.44;
    //     frontConfig.offset.y = -0.19;
    //     frontConfig.offset.theta = -0.595;
    //     frontConfig.range = utils::ray_trace_range_t((-135. + 11.25) * M_PI / 180.0, std::size_t(955 / 4), M_PI /
    //     180.0);
    //
    //     laser_configuration_t backConfig;
    //     backConfig.offset.x = -0.39;
    //     backConfig.offset.y = 0.27;
    //     backConfig.offset.theta = 2.55;
    //     // Taken from front_laser_scan_producer.cfg  first and last valid index parameters
    //     backConfig.range = utils::ray_trace_range_t((-135. + 5.0) * M_PI / 180.0, std::size_t(1040 / 4), M_PI /
    //     180.0);
    //
    //     laserConfigs_.push_back(frontConfig);
    //     laserConfigs_.push_back(backConfig);

    // initialize log
#ifdef LOG_DATA
    openDataLog();
#endif
}


MetricPlannerDirector::~MetricPlannerDirector(void)
{
    // For std::unique_ptr
}

//// data subscriptions ////////////////////////////////////////////////////////
void MetricPlannerDirector::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    lpmBuffer_.write(lpm);

    haveGrid_ = true;
    dataTrigger_.setPredicate(haveEssentialData());
    dataTrigger_.broadcast();
}


void MetricPlannerDirector::handleData(const hssh::LocalTopoMap& topoMap, const std::string& channel)
{
    topoBuffer_.write(topoMap);

    haveLTM_ = true;
    dataTrigger_.setPredicate(haveEssentialData());
    dataTrigger_.broadcast();
}


void MetricPlannerDirector::handleData(const tracker::DynamicObjectCollection& trackedObjects,
                                       const std::string& channel)
{
    inDynamicEnvironment_ = true;
    trackedObjectsBuffer_.write(trackedObjects);
}


void MetricPlannerDirector::handleData(const motion_state_t& motionState, const std::string& channel)
{
    robotStateBuffer_.write(motionState);

    haveRobotState_ = true;
    dataTrigger_.setPredicate(haveEssentialData());
    dataTrigger_.broadcast();
}


void MetricPlannerDirector::handleData(const std::shared_ptr<MetricPlannerTask>& task, const std::string& channel)
{
    {
        utils::AutoMutex autoLock(taskLock_);
        receivedTask_ = task;

        // if there is a script and a task arriving at the same time ignore the script.
        haveNewTask_ = true;
        haveNewScript_ = false;
        usingLCMTask_ = false;
        usingScript_ = false;
        std::cout << "INFO: MetricPlanner: New task received.\n";
    }

    // activate the planner as soon as the task arrives, but only if there is relevant data to work with.
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void MetricPlannerDirector::handleData(const MetricPlannerScript& script, const std::string& channel)
{
    // Create scope to cause the lock to release as soon as access to the targets stops.
    if (!haveNewTask_)   // if there is a script and a task arriving at the same time ignore the script.
    {
        utils::AutoMutex autoLock(scriptLock_);

        receivedScript_ = std::make_shared<MetricPlannerScript>(script);
        haveNewScript_ = true;
        usingLCMTask_ = false;
        std::cout << "INFO: MetricPlanner: New script received.\n";
    }

    // activate the planner as soon as the script arrives
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void MetricPlannerDirector::handleData(const metric_planner_command_message_t& message, const std::string& channel)
{
    messageQueue_.push(message);

    std::cout << "INFO: MetricPlanner: New command message received.\n";

    // Wake up the planner if a new message arrives because it might be to stop planning, which needs to happen ASAP
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void MetricPlannerDirector::handleData(const vulcan_lcm::metric_pose_target_t& target, const std::string& channel)
{
    if (target.is_position) {
        position_t position(target.x, target.y);
        auto task = std::make_shared<NavigationTask>(position, target.id);
        handleData(task, channel);
    } else {
        pose_t pose(target.x, target.y, target.theta);
        auto task = std::make_shared<NavigationTask>(pose, target.id);
        handleData(task, channel);
    }

    usingLCMTask_ = true;   // assign after handling to override any writes in the normal handling methods
}


//// system::Director Interface ////////////////////////////////////////////////
void MetricPlannerDirector::subscribeToData(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalPerceptualMap>(this);
    producer.subscribeTo<hssh::LocalTopoMap>(this);
    producer.subscribeTo<tracker::DynamicObjectCollection>(this);
    producer.subscribeTo<motion_state_t>(this);
    producer.subscribeTo<std::shared_ptr<MetricPlannerTask>>(this);
    producer.subscribeTo<MetricPlannerScript>(this);
    producer.subscribeTo<metric_planner_command_message_t>(this);
    producer.subscribeTo<vulcan_lcm::metric_pose_target_t>(this);
}


system::TriggerStatus MetricPlannerDirector::waitForTrigger(void)
{
    // wait if other data producers are not working, i.e. no state or no task.
    if (dataTrigger_.timedWait(100)) {
        return system::TriggerStatus::not_ready;
    }

    // switch buffer only if the data has arrived - ensure the read buffer
    // contains the most recent data.
    if (lpmBuffer_.hasData()) {
        lpmBuffer_.swapBuffers();
    }

    if (topoBuffer_.hasData()) {
        topoBuffer_.swapBuffers();
    }

    if (trackedObjectsBuffer_.hasData()) {
        trackedObjectsBuffer_.swapBuffers();
    }

    if (robotStateBuffer_.hasData()) {
        robotStateBuffer_.swapBuffers();
    }

    // unpause planner if new task has arrived
    if (haveNewTask_ || haveNewScript_) {
        isPaused_ = false;
    }

    std::cout << "\nINFO: MetricPlanner: Data received.\n";

    return system::TriggerStatus::ready;
}


void MetricPlannerDirector::shutdown(system::ModuleCommunicator& transmitter)
{
    // cancel any existing motion controller target at shutdown for safety.
    motionControllerCommandMessage_.command = motion_controller_command_t::MOTION_CONTROLLER_CANCEL;
    motionControllerCommandMessage_.timestamp = utils::system_time_us();

#ifdef LOG_DATA
    // stop logging
    dataLog.close();
#endif

    transmitter.sendMessage(motionControllerCommandMessage_);
}


system::UpdateStatus MetricPlannerDirector::runUpdate(system::ModuleCommunicator& transmitter)
{
    // setup timing and update the task environment with the static map and dynamic object info
    setMetricPlannerTiming();
    updateTaskEnvironment();

    // process any command message received.
    processCommandMessages();

    // initialize debug info
    trajectoryPlannerDebugInfo_.clear();

    plannerStatus_.taskId = activeTask_ ? activeTask_->id() : MetricPlannerTask::kInvalidId;

    // path and trajectory planning
    if (isPaused_) {
        plannerStatus_.status = metric_planner_status_t::PAUSED;
        std::cout << "\nINFO: MetricPlanner: Planner is paused.\n";
    } else if (!isSafeToPlan()) {
        // if not safe to plan, then send out command to stop robot and produce an error message.
        nextMotionTarget_ = motion_target_t(robotStateBuffer_.read().pose, approach_direction_t::FORWARD, 0.0);
        haveMotionControllerTask_ = true;
        ;

        plannerStatus_.status = metric_planner_status_t::FAILURE_UNABLE_TO_PROGRESS;
        std::cout << "\nWARNING!: MetricPlanner: Not safe to proceed! Stopping until recovery...\n";
    } else {
        // task handling
        handleScript();   // generate a task from the script if applicable, and setup a state machine for the task

        // attempt to assign a new task if one is available
        if (haveNewTask_) {
            // If can't assign the task, indicate which task has failed and ensure the correct status is set
            if (!assignNewTask()) {
                plannerStatus_.taskId = receivedTask_->id();
                plannerStatus_.status = FAILURE_CANNOT_ASSIGN_TASK;
            } else {
                runPlanners();
            }
        } else {
            runPlanners();   // process the task via path and trajectory planners and set various planner status
        }
    }

    // reset data trigger for intputs. Don't run if no new piece of data (or a task) has come in.
    dataTrigger_.setPredicate(false);

    // enforce timing and send outputs
    enforceMetricPlannerTiming();
    transmitCalculatedOutput(transmitter);

    // metric_planner runs forever
    return system::UpdateStatus::running;
}


//// methods for running update ////////////////////////////////////////////////
// setting and enforcing update timing
void MetricPlannerDirector::setMetricPlannerTiming(void)
{
    // recall last plan release time
    previousPlanReleaseTimeUs_ = planReleaseTimeUs_;

    // difference between current time and previous plan release time
    updateStartTimeUs_ = utils::system_time_us();
    int64_t timeSinceLastPlanReleaseUs = updateStartTimeUs_ - previousPlanReleaseTimeUs_;

#ifdef DEBUG_DIRECTOR_TIMING
    std::cout << "DEBUG: Director: planner previously executed at (ms): " << previousPlanReleaseTimeUs_ / 1000 << '\n';
#endif

    if ((timeSinceLastPlanReleaseUs > 0) && (timeSinceLastPlanReleaseUs < params_.plannerUpdatePeriodUs)) {
        // use previous planner update time when the system time makes sense
        planReleaseTimeUs_ = previousPlanReleaseTimeUs_ + params_.plannerUpdatePeriodUs;
    } else {
        // use relative time with respect to data if this is the first time the planner is running
        // or when there is a larger difference between the data.timestamp and the system_time_us()
        // this allows the metric planner to run with logged data.
        planReleaseTimeUs_ = robotStateBuffer_.read().timestamp + params_.plannerUpdatePeriodUs;
    }

    std::cout << "INFO: MetricPlanner: time since the last planner execution (ms): "
              << timeSinceLastPlanReleaseUs / 1000 << "\n";

#ifdef DEBUG_DIRECTOR_TIMING
    int64_t lpmTimestampUs = lpmBuffer_.read().getTimestamp();
    int64_t stateTimestampUs = robotStateBuffer_.read().timestamp;

    std::cout << "DEBUG: Director: curent time       (ms): " << updateStartTimeUs_ / 1000 << '\n';
    std::cout << "DEBUG: Director: LPM timestamp     (ms): " << lpmTimestampUs / 1000
              << ", relative (ms): " << (lpmTimestampUs - planReleaseTimeUs_) / 1000 << '\n';
    std::cout << "DEBUG: Director: state timestamp   (ms): " << stateTimestampUs / 1000
              << ", relative (ms): " << (stateTimestampUs - planReleaseTimeUs_) / 1000 << '\n';

    if (!trackedObjectsBuffer_.read().empty()) {
        int64_t objectTimestampUs = (*trackedObjectsBuffer_.read().begin())->timeLastSeen();
        std::cout << "DEBUG: Director: object timestamp  (ms): " << objectTimestampUs / 1000
                  << ", relative (ms): " << (objectTimestampUs - planReleaseTimeUs_) / 1000 << '\n';
    }

    std::cout << "DEBUG: Director: planner will execute next at   (ms): " << planReleaseTimeUs_ / 1000 << '\n';
#endif
}


void MetricPlannerDirector::enforceMetricPlannerTiming(void)
{
    // timing info for this planning cycle
    int64_t updateFinishTimeUs = utils::system_time_us();   // time after all planner computation
    int64_t timeElapsedUs = updateFinishTimeUs - updateStartTimeUs_;

    // timing from the previous planning cycle
    int64_t timeSincePreviousOutputUs = utils::system_time_us() - previousPlanReleaseTimeUs_;
    if (abs(timeSincePreviousOutputUs)
        > utils::sec_to_usec(10))   // detect if data is coming from a log by looking at the time differnce
    {
        timeSincePreviousOutputUs =
          timeElapsedUs;   // if data is coming from a log then just use computation time for update
    }

    // compute average planning time with exponential moving average, alpha*observation + (1-alpha)*previous.
    // alpha = 0.8 will set the weight of a sample to < 0.03 after ~30 observations.
    averagePlanningTimeUs_ =
      (averagePlanningTimeUs_ == 0) ? timeElapsedUs : 0.2 * timeElapsedUs + 0.8 * averagePlanningTimeUs_;

    // display planning time info
    std::cout << "INFO: MetricPlanner: Total planning time for this cycle (ms): " << timeElapsedUs / 1000 << '\n';
    std::cout << "INFO: MetricPlanner: Time since previous plan output    (ms): " << timeSincePreviousOutputUs / 1000
              << '\n';

    std::cout << "INFO: MetricPlanner: Average planning time (ms): " << averagePlanningTimeUs_ / 1000 << '\n';
    if (averagePlanningTimeUs_ > params_.plannerUpdatePeriodUs) {
        std::cout << "\nWARNING!: MetricPlanner: Average planning time is longer than the specified update interval!"
                  << '\n';
    }

    int64_t sleepTime = params_.plannerUpdatePeriodUs - timeSincePreviousOutputUs;
    if (sleepTime > 0)   // sleep only when the planning happend quicker than required.
    {
        std::cout << "INFO: MetricPlanner: Sleeping for " << sleepTime / 1000 << " ms until the next planning cycle."
                  << '\n';
        usleep(sleepTime);
    } else   // carry on planning even when the specified fixed interval cannot be met, but do churn out a warning.
    {
        std::cout
          << "\nWARNING!: MetricPlanner: Planning took longer than the specified update interval for this cycle!"
          << '\n';
    }
}


// setting up task environment - obstacle map and object trajectories
void MetricPlannerDirector::updateTaskEnvironment(void)
{
    // build obstacle ditance grid and simulate dynamic objects up to estimation time length

    // Build grid-based interpretaion of the environment.
    // The grids are rebuilt at roughly at regular interval if the robot is stationary, and update more often as robot
    // moves faster (e.g. with coefficient = 1.0 and robot linear velocity at 1 m/s, the grids are updated twice as
    // fast), and only if there has been a change in the original lpm.
    int64_t adaptiveGridRebuildIntervalUs = params_.gridRebuildIntervalUs
      / (1.0 + 1.0 * robotStateBuffer_.read().velocity.linear * robotStateBuffer_.read().velocity.linear);

    // attempt updating costMap if a stored lpm is loaded (lpm timestamp equals zero), or the grid has never been
    // updated before (lastGridUpdateTimeUs equals zero), or enough time has passed since the last update.
    if (lpmBuffer_.read().getTimestamp() == 0 || lastGridUpdateTimeUs_ == 0
        || (utils::system_time_us() - lastGridUpdateTimeUs_) > adaptiveGridRebuildIntervalUs) {

#ifdef DEBUG_GRID_BUILDING
        int64_t ticUs = utils::system_time_us();
#endif
        // buildCostMap will look at the ID's and decide if the costMap should be built. If we already have a costMap
        // generated from the given lpm (indicated by same ID) then the map will not be updated.
        haveNewObstacleDistanceGrid_ = obstacleDistanceGridBuilder_.buildGrid(
          lpmBuffer_.read(),
          obstacleDistanceGrid_);   // builder will not update the map if the lpm did not change

#ifdef DEBUG_GRID_BUILDING
        int64_t tocUs = utils::system_time_us();

        if (haveNewObstacleDistanceGrid_) {
            std::cout << "INFO: SensorDataProcessor: Costmap updated:\n"
                      << "    LPM ID: " << lpmBuffer_.read().getId()
                      << "  Costmap ID: " << obstacleDistanceGrid_.getId()
                      << "  Processing time (ms): " << (tocUs - ticUs) / 1000 << '\n';
        }
#endif

        lastGridUpdateTimeUs_ = utils::system_time_us();
    }

    dynObjects_ = objectFilter_.filterObjects(trackedObjectsBuffer_.read(),
                                              robotStateBuffer_.read(),
                                              obstacleDistanceGrid_,
                                              planReleaseTimeUs_);
    objectSimulator_.estimateObjectTrajectories(dynObjects_,
                                                robotStateBuffer_.read(),
                                                obstacleDistanceGrid_,
                                                planReleaseTimeUs_);

    // if this was the first update then setup previous motion target.
    // I can't do this at construction  since it requires the current robot pose.
    if (previousPlanReleaseTimeUs_ == 0) {
        // default controller target on top of the robot with a modest velocity gain
        previousMotionTarget_ = motion_target_t(robotStateBuffer_.read().pose, approach_direction_t::FORWARD, 0.75);
    }
}


// processing and handling command messages
void MetricPlannerDirector::processCommandMessages(void)
{
    // Take the messages in the queue and throw them in a vector so I can sort
    // based on the time stamps to ensure they are processed in the order of creation.
    // This code relies on the fact that only this thread shrinks the queue.
    // The communication thread can grow the queue as it wishes without affecting this code.
    std::vector<metric_planner_command_message_t> messages;

    while (!messageQueue_.empty()) {
        messages.push_back(messageQueue_.front());
        messageQueue_.pop();
    }

    // Sort the messages to ensure they are processed in the order in which they were created, not the arrival order
    // which could potentially be different
    std::sort(messages.begin(),
              messages.end(),
              [](const metric_planner_command_message_t& lhs, const metric_planner_command_message_t& rhs) {
                  return lhs.timestamp < rhs.timestamp;
              });

    for (const auto& message : messages) {
        handleCommandMessage(message);
    }
}

void MetricPlannerDirector::handleCommandMessage(const metric_planner_command_message_t& message)
{
    std::cout << "INFO: MetricPlanner: Received a command message." << '\n';

    switch (message.command) {
    case PAUSE:
        isPaused_ = true;
        std::cout << "INFO: MetricPlanner: Paused." << '\n';

        motionControllerCommandMessage_.command = motion_controller_command_t::MOTION_CONTROLLER_PAUSE;
        motionControllerCommandMessage_.timestamp = planReleaseTimeUs_;
        haveMotionControllerCommandMessage_ = true;

        break;

    case RESUME:
        isPaused_ = false;
        std::cout << "INFO: MetricPlanner: Resumed." << '\n';

        motionControllerCommandMessage_.command = motion_controller_command_t::MOTION_CONTROLLER_RESUME;
        motionControllerCommandMessage_.timestamp = planReleaseTimeUs_;
        haveMotionControllerCommandMessage_ = true;

        break;

    case CANCEL:   // intentional fall-through (hopefully temporary)
    case SCRIPT_STOP:
        haveNewScript_ = false;
        haveNewTask_ = false;
        usingScript_ = false;
        std::cout << "INFO: MetricPlanner: Cancelled all tasks." << '\n';

        activeScript_.reset();
        activeTask_.reset();
        progressChecker.reset();

        motionControllerCommandMessage_.command = motion_controller_command_t::MOTION_CONTROLLER_CANCEL;
        motionControllerCommandMessage_.timestamp = planReleaseTimeUs_;
        haveMotionControllerCommandMessage_ = true;

        break;

    case UPDATE_WAYPOINTS:
        // TODO: Fill this in. Doing this properly will require substantial change to how the script and tasks are
        // handled currently.

        break;

    case SCRIPT_NEXT_TASK:
        if (activeScript_) {
            utils::AutoMutex autoLock(taskLock_);

            receivedTask_.reset();
            receivedTask_ = activeScript_->getNextMetricPlannerTask();

            haveNewTask_ = true;
            isPaused_ = false;

            std::cout << "INFO: MetricPlanner: Switched to the next avaoialbe task in the script." << '\n';
        } else {
            std::cout << "INFO: MetricPlanner: Do not have an active script!" << '\n';
        }

        break;

    case SCRIPT_LOOP:
        std::cout << "INFO: MetricPlanner: No active script." << '\n';
        if (activeScript_) {
            activeScript_->loopThroughScript();
            std::cout << "INFO: MetricPlanner: Switching the script looping behavior." << '\n';
        }

        break;

    case SET_MODE_STOP_AT_WAYPOINTS:
        // TODO: Fill this in. Currently stops at all waypoints.

        break;

    case SET_MODE_USE_RRT_STAR:
        // TODO: Activate this after implementing RRT star.
        // params_.taskManifoldBuilderParams.navigationTaskManifoldParams.shouldBuildRRT =
        // static_cast<bool>(message.intValue);

        break;

    case SET_MODE_QUASI_STATIC_OBJECTS:
        objectSimulator_.setModeQuasiStatic(static_cast<bool>(message.intValue));

        break;

    case SET_MODE_SIMPLE_POSE_FOLLOWING:
        trajectoryPlanner_.setModePoseFollowing(static_cast<bool>(message.intValue));

        break;
    }
}


// simple check on preconditions
bool MetricPlannerDirector::isSafeToPlan(void)
{
    bool isSafe = true;

    // check for localization failure, and cancel planning if any of the uncertainty is very large
    Matrix poseUncertainty = robotStateBuffer_.read().poseDistribution.uncertainty.getCovariance();
    Vector uncertaintyEig = arma::eig_sym(poseUncertainty.submat(0, 0, 1, 1));
    double maxPositionStd = sqrt(uncertaintyEig(1));   // eigenvalues are in ascending order
    if (maxPositionStd > params_.maxSafePositionStd) {
        isSafe = false;
        std::cout << "\nWARNING!: MetricPlanner: Position uncertainty too large!\n";
    }

    if (poseUncertainty(2, 2) > params_.maxSafeHeadingStd) {
        isSafe = false;
        std::cout << "\nWARNING! Metric planner: Heading uncertianty too large!\n";
    }

    // check for stale data, which may be due to system failure
    if (planReleaseTimeUs_ - robotStateBuffer_.read().timestamp > params_.plannerUpdatePeriodUs + 100000) {
        isSafe = false;   // do not plan with stale state estimation
        std::cout << "\nWARNING!: MetricPlanner: Robot state too old! : State timestamp (ms): "
                  << robotStateBuffer_.read().timestamp / 1000 << "\n";
    }

    // NOTE: Only check for dynamic objects if they have ever been received. This allows for running metric_planner
    // with the object_tracker turned off. Though not having it turned on is generally a bad approach.
    if (inDynamicEnvironment_
        && (planReleaseTimeUs_ - trackedObjectsBuffer_.read().timestamp() > 2 * params_.plannerUpdatePeriodUs)) {
        isSafe = false;
        int64_t overTime =
          (planReleaseTimeUs_ - trackedObjectsBuffer_.read().timestamp() - 2 * params_.plannerUpdatePeriodUs);
        std::cout << "\nWARNING!: MetricPlanner: Tracked object state too old!: Object timestamp (ms): "
                  << trackedObjectsBuffer_.read().timestamp() / 1000 << " Over time by " << (overTime / 1000) << "ms\n";
    }

    // check if the pose is inside the map boundary
    if (!lpmBuffer_.read().isPoseInGrid(robotStateBuffer_.read().pose)) {
        isSafe = false;
        std::cout << "\nWARNING!: MetricPlanner: Current robot pose is not within the map!.\n";
    }

    return isSafe;
}


// hadling received script and task
void MetricPlannerDirector::handleScript(void)
{
    // handle action script from user input, if it exists
    if (haveNewScript_) {
        utils::AutoMutex autoLock(scriptLock_);

        if (receivedScript_->isSafeToExecute(
              obstacleDistanceGrid_))   // the script is valid only if it is compatible to the current map
        {
            activeScript_.reset();
            activeScript_ = receivedScript_;   // take the ownership of the received script.
            std::cout << "\nINFO: MetricPlanner: New script is safe to execute. Initiating the script...\n";

            progressChecker.reset();   // start evaluating current progress
        } else {
            activeScript_.reset();
            std::cout << "\nWarning!: MetricPlanner: Unable to execute the received script. Script cancelled.\n";
            plannerStatus_.status = metric_planner_status_t::FAILURE_CANNOT_ASSIGN_TASK;
        }

        haveNewScript_ = false;
        usingScript_ = true;
    }

    if (activeScript_) {
        progressChecker.run(robotStateBuffer_.read());

        if (progressChecker.robotIsNotProgressingForSomeTime()) {
            activeScript_.reset();

            std::cout << "\nWarning!: MetricPlanner: Unable to progress to complete the plan. Cancelling Script.\n";
            plannerStatus_.status = metric_planner_status_t::FAILURE_UNABLE_TO_PROGRESS;
        }
    }

    // if only the script is active and the task is inactive, then extract a new task from the script
    if (activeScript_ && usingScript_) {
        // set conditions for the state machine for metric planner tasks
        if (activeScript_->isAtFinalTarget()) {
            // if the task is completed (!activeTask_) and it was the final one then we're done.
            activeScript_.reset();
            std::cout << "INFO: MeticPlanner: End of the script reached. Task complete.\n";

            progressChecker.reset();   // finish checking progress

            // write to file to trigger manipulation process
            std::ofstream begin_file("begin.txt", std::ios::out | std::ios::app);
            std::cout << "*********** All targets done, now writing to file\n";
            exit(0);
        } else if (activeScript_->shouldWaitForPrecondition()) {
            // this is going to be useful for door and elevator tasks.
            isWaitingForPrecondition_ = true;
            std::cout
              << "INFO: MetricPlanner: Script is waiting until the precondition for the next task is satisfied.\n";
        } else if (!activeTask_ || (activeTask_ && activeTask_->isComplete(robotStateBuffer_.read()))) {
            // otherwise switch to the next task
            utils::AutoMutex autoLock(taskLock_);

            receivedTask_.reset();
            receivedTask_ = activeScript_->getNextMetricPlannerTask();   // no check for individual tasks here.
            haveNewTask_ = true;
            std::cout << "INFO: MetricPlanner: Switching to the next task in the script.\n";
        }
    }
}


bool MetricPlannerDirector::assignNewTask(void)
{
    // if have new task, either from the UI or the script,
    if (haveNewTask_) {
        utils::AutoMutex autoLock(taskLock_);

        receivedTask_->setTaskParameters(params_.taskParams, params_.taskManifoldBuilderParams);

        if (receivedTask_->isSafeToExecute(obstacleDistanceGrid_)) {
            activeTask_ = receivedTask_;
            activeManifold_ = activeTask_->createTaskManifold();
            plannerStatus_.status = metric_planner_status_t::ACTIVE_NORMAL;
            std::cout << "\nINFO: MetricPlanner: New task is safe to execute. Initiating the task...\n";
        } else {
            std::cout << "\nWarning!: MetricPlanner: Cannot assign the received task.\n";
            plannerStatus_.status = metric_planner_status_t::FAILURE_CANNOT_ASSIGN_TASK;
            plannerStatus_.taskId = receivedTask_->id();

            // reset the active task to ensure it stops running after the failure to assign a new task
            activeTask_.reset();
        }

        haveNewTask_ = false;
    }

    return plannerStatus_.status != metric_planner_status_t::FAILURE_CANNOT_ASSIGN_TASK;
}


// run the planner
void MetricPlannerDirector::runPlanners(void)
{
    // state machine for a given task
    if (!activeTask_) {
        // if no task, then the planner is idle.
        plannerStatus_.status = metric_planner_status_t::IDLE;
        std::cout << "INFO: MetricPlanner: Planner does not have an active task and is idle.\n";
    } else if (activeTask_->isComplete(robotStateBuffer_.read())) {
        plannerStatus_.status =
          metric_planner_status_t::SUCCESS_REACHED_POSE;   // this is the only task type that can be completed-for now.

        // if the task is complete, then reset the task and stop the controller.
        activeTask_.reset();
        motionControllerCommandMessage_.command = motion_controller_command_t::MOTION_CONTROLLER_CANCEL;
        motionControllerCommandMessage_.timestamp = planReleaseTimeUs_;
        haveMotionControllerCommandMessage_ = true;

        std::cout << "INFO: MeticPlanner: Task complete.\n";
    } else if (!activeTask_->isSafeToExecute(obstacleDistanceGrid_)) {
        // task is not safe to execute! this may be temporary.
        // TODO: decide what to do here exactly. Should I be pausing or canceling?
        plannerStatus_.status = metric_planner_status_t::FAILURE_UNABLE_TO_PROGRESS;
        std::cout << "\nWarning!: MetricPlanner: Unable to execute the received task.\n";

        motionControllerCommandMessage_.command = motion_controller_command_t::MOTION_CONTROLLER_CANCEL;
        motionControllerCommandMessage_.timestamp = planReleaseTimeUs_;
        haveMotionControllerCommandMessage_ = true;
    } else if (isWaitingForPrecondition_) {
        plannerStatus_.status = metric_planner_status_t::ACTIVE_SPECIAL;
        // script is doing its work. don't do anything here.
    } else {
        // if there is an active task, and the task is not complete, and if it is safe to execute the task,
        // and if there is no special behavior to run, then go ahead with the planning.

        planning_environment_t env;
        env.robotState = robotStateBuffer_.read();
        env.robotRadius = 0.32;   // radius of inscribed circle for rectangle model of robot --- TODO: Grab from config
        env.lpm = &lpmBuffer_.read();
        env.ltm = haveLTM_ ? &topoBuffer_.read() : nullptr;
        env.dists = &obstacleDistanceGrid_;

        if (laserConfigs_.size() > 1) {
            visibility_ =
              VisibilityAnalysis(env.robotState.pose, laserConfigs_, *env.lpm, trackedObjectsBuffer_.read());
        } else if (!laserConfigs_.empty()) {
            visibility_ =
              VisibilityAnalysis(env.robotState.pose, laserConfigs_.front(), *env.lpm, trackedObjectsBuffer_.read());
        }
        env.visibility = &visibility_;

        activeManifold_->update(env, dynObjects_);

        auto manifoldObjects = activeManifold_->createTaskSpecificObjects(env);
        if (!manifoldObjects.empty()) {
            objectSimulator_.estimateObjectTrajectories(manifoldObjects,
                                                        env.robotState,
                                                        obstacleDistanceGrid_,
                                                        planReleaseTimeUs_);
            dynObjects_.insert(dynObjects_.end(), manifoldObjects.begin(), manifoldObjects.end());
        }

        // if so run the trajectory planner
        trajectoryPlannerOutput_ = trajectoryPlanner_.run(*activeManifold_,
                                                          env.robotState,
                                                          obstacleDistanceGrid_,
                                                          dynObjects_,
                                                          previousMotionTarget_,
                                                          planReleaseTimeUs_,
                                                          trajectoryPlannerDebugInfo_);

        // set next motion target. if the output is not good the motion target is already set to stopping motoin.
        nextMotionTarget_ = trajectoryPlannerOutput_.motionTarget;
        haveMotionControllerTask_ = true;
        ;

        // set planner status according to the output.
        if (trajectoryPlannerOutput_.isGood) {
            plannerStatus_.status =
              metric_planner_status_t::ACTIVE_NORMAL;   // this should account for (ideally) ALL normal situation.
        } else {
            plannerStatus_.status =
              metric_planner_status_t::FAILURE_CANNOT_FIND_SOLUTION;   // indicate failure case otherwise.
        }
    }

#ifdef LOG_DATA
    logData();
#endif
}


// transmit output
void MetricPlannerDirector::transmitCalculatedOutput(system::ModuleCommunicator& transmitter)
{
    // motoin controller task and messages
    if (haveMotionControllerTask_)   // handle controller tasks first! Other large pieces of data might take more time
                                     // to communicate
    {
        // package the result for output comsumer
        int64_t motionTargetTaskTimeoutUs =
          10 * params_.plannerUpdatePeriodUs;   // set timeout length for graceful slowdown.
        // the motion controller will send stop commands if the motion target task has timed out.
        motionControllerTask_ = std::shared_ptr<MotionControllerTask>(
          new MotionTargetTask(nextMotionTarget_, motionTargetTaskTimeoutUs, planReleaseTimeUs_));
        transmitter.sendMessage(motionControllerTask_);

        // reset flag and store the result for the next update
        haveMotionControllerTask_ = false;
        previousMotionTarget_ = nextMotionTarget_;
    }

    if (haveMotionControllerCommandMessage_) {
        transmitter.sendMessage(motionControllerCommandMessage_);
        haveMotionControllerCommandMessage_ = false;
    }

    // always send the status of the planner
    plannerStatus_.timestamp = utils::system_time_us();
    transmitter.sendMessage(plannerStatus_);

    // If using an LCM task, then the LCM version of the status needs to be sent out
    if (usingLCMTask_ && (plannerStatus_.taskId != MetricPlannerTask::kInvalidId)) {
        vulcan_lcm::metric_pose_target_status_t status;
        status.timestamp = utils::system_time_us();
        status.target_id = plannerStatus_.taskId;

        switch (plannerStatus_.status) {
        case metric_planner_status_t::ACTIVE_NORMAL:
            status.status = vulcan_lcm::metric_pose_target_status_t::IN_PROGRESS;
            break;

        case metric_planner_status_t::SUCCESS_REACHED_POSE:
            status.status = vulcan_lcm::metric_pose_target_status_t::REACHED_TARGET;
            break;

        case metric_planner_status_t::FAILURE_CANNOT_ASSIGN_TASK:
            status.status = vulcan_lcm::metric_pose_target_status_t::FAILED_TARGET_CANNOT_BE_ASSIGNED;
            break;

        case metric_planner_status_t::FAILURE_CANNOT_FIND_SOLUTION:
            status.status = vulcan_lcm::metric_pose_target_status_t::FAILED_CANNOT_FIND_SOLUTION;
            break;

        case metric_planner_status_t::FAILURE_UNABLE_TO_PROGRESS:
            status.status = vulcan_lcm::metric_pose_target_status_t::FAILED_ROBOT_IS_STUCK;
            break;

        // If not doing anything, or any other message that isn't handled, the robot isn't doing anything.
        case metric_planner_status_t::IDLE:
        default:
            status.status = vulcan_lcm::metric_pose_target_status_t::IDLE;
        }

        // Still performing the task until we have success
        usingLCMTask_ = (plannerStatus_.status == metric_planner_status_t::ACTIVE_NORMAL)
          || (plannerStatus_.status == metric_planner_status_t::IDLE);

        transmitter.sendMessage(status);
    }

    // estimated dynamic object trajectories (will send empty vector if no object)
    //     transmitter.sendMessage(objectSimulator_.getDebugInfo());

    // always send the visibility analysis
    transmitter.sendMessage(visibility_);

    std::vector<dynamic_object_trajectory_debug_info_t> dynObjDebug;
    for (auto& obj : dynObjects_) {
        dynObjDebug.emplace_back(obj);
    }

    transmitter.sendMessage(dynObjDebug);

    if (params_.shouldDebugDynamicObjects)   // also report full data on dynamic objects if specified
    {
        transmitter.sendMessage(dynObjects_);
    }

    // trajectory planner debug info (will send empty one if there is no info)
    if (shouldOutputLessDebugInfo_) {
        // clears all trajecoty evaluation info.
        trajectoryPlannerDebugInfo_.numTrajectories = 0;
        trajectoryPlannerDebugInfo_.trajectories.clear();
        trajectoryPlannerDebugInfo_.coarseOptimizerOutput = robot_trajectory_debug_info_t();
        trajectoryPlannerDebugInfo_.localOptimizerOutput = robot_trajectory_debug_info_t();
    }

    transmitter.sendMessage(trajectoryPlannerDebugInfo_);

    // static obstacle info
    if (haveNewObstacleDistanceGrid_ && params_.shouldDebugGrids) {
        // limit max update rate to half a second
        if (utils::system_time_us() - lastTransmissionTimeForObstacleDistanceGridUs_ > 500000) {
            transmitter.sendMessage(obstacleDistanceGrid_);
            lastTransmissionTimeForObstacleDistanceGridUs_ = utils::system_time_us();
        }
    }

    // navigation function (and possibly rrt and more later)
    if (activeManifold_) {
        activeManifold_->sendDebugInfo(transmitter);
    }
}


void MetricPlannerDirector::openDataLog(void)
{
    if (!dataLog.is_open()) {
        std::cout << '\n' << "Opening log file..." << '\n';

        // set name using current time
        time_t systemTime;
        struct tm* timeInfo;

        time(&systemTime);
        timeInfo = localtime(&systemTime);

        std::ostringstream filename;
        filename << "planner_data_" << timeInfo->tm_mon << timeInfo->tm_mday << timeInfo->tm_hour << timeInfo->tm_min
                 << ".log";

        dataLog.open(filename.str());
    }
}

void MetricPlannerDirector::logData(void)
{
    if (haveMotionControllerTask_) {
        dataLog << updateStartTimeUs_
                << ' '
                //     <<trajectoryPlannerDebugInfo_.clearanceToStaticObs<<' '
                //     <<trajectoryPlannerDebugInfo_.clearanceToDynObs<<' '
                << robotStateBuffer_.read().timestamp
                << ' '
                //     <<robotStateBuffer_.read().pose.x<<' '
                //     <<robotStateBuffer_.read().pose.y<<' '
                //     <<robotStateBuffer_.read().pose.theta<<' '
                << robotStateBuffer_.read().velocity.linear << ' ' << robotStateBuffer_.read().velocity.angular
                << ' '
                //     <<robotStateBuffer_.read().acceleration.linear<<' '
                //     <<robotStateBuffer_.read().acceleration.angular<<' '
                << robotStateBuffer_.read().differentialWheels.rightWheel.speed << ' '
                << robotStateBuffer_.read().differentialWheels.leftWheel.speed << ' ' << nextMotionTarget_.velocityGain
                << std::endl;
    }
}

}   // namespace mpepc
}   // namespace vulcan
