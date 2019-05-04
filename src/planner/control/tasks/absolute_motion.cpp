/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     absolute_motion.cpp
* \author   Collin Johnson
*
* Definition of AbsoluteMotionTask.
*/

#include <planner/control/tasks/absolute_motion.h>
#include <planner/control/state.h>
#include <mpepc/metric_planner/task/navigation.h>
#include <mpepc/metric_planner/task/rotate.h>
#include <mpepc/metric_planner/task/wait.h>
#include <utils/ray_tracing.h>
#include <utils/timestamp.h>
#include <cassert>

namespace vulcan
{
namespace planner
{

pose_t select_straight_ahead_pose(const pose_t& startPose, const ControlState& state);


AbsoluteMotionTask::AbsoluteMotionTask(AbsoluteMotion motion, double speed, int32_t id, ControlTask* parent)
: ControlTask(id, parent)
, motion_(motion)
, speed_(speed)
, sentTask_(false)
{
    if(motion_ == AbsoluteMotion::turn_left)
    {
        outgoingTask_ = std::make_shared<mpepc::RotateTask>(mpepc::RotationMode::turn_left);
    }
    else if(motion_ == AbsoluteMotion::turn_right)
    {
        outgoingTask_ = std::make_shared<mpepc::RotateTask>(mpepc::RotationMode::turn_right);
    }
    else if(motion_ == AbsoluteMotion::stop)
    {
        outgoingTask_ = std::make_shared<mpepc::WaitTask>();
    }
}


void AbsoluteMotionTask::activate(const ControlState& state)
{
    // Activation only has meaning for the go straight task, as it needs to find the pose to command. Other tasks
    // all handle their business in the constructor
    
    if(motion_ != AbsoluteMotion::go_straight)
    {
        return;
    }

    startPose_ = state.pose.pose();
    targetPose_ = select_straight_ahead_pose(startPose_, state);

    outgoingTask_ = std::make_shared<mpepc::NavigationTask>(targetPose_);
}


bool AbsoluteMotionTask::hasSubTask(void) const
{
    // There are no subtasks that need to be executed.
    return false;
}


int AbsoluteMotionTask::pushSubTasks(ControlTaskStack& stack)
{
    // As there are never subtasks, nothing needs to be pushed
    return 0;
}


ControlTaskResult AbsoluteMotionTask::doExecute(const ControlState& state)
{
    // If close to the goal, then try to find a new goal
    if((motion_ == AbsoluteMotion::go_straight)
        && (distance_between_points(state.pose.pose().toPoint(), targetPose_.toPoint()) < 1.0))
    {
        pose_t newTarget = select_straight_ahead_pose(startPose_, state);

        std::cout << "Searching for new target. New pose:" << state.pose.pose() << " Start pose:" << startPose_ 
            << " Old target:" << targetPose_ << " New target:" << newTarget << '\n';
        // If a new target exists, then go to the new pose
        if(newTarget != targetPose_)
        {
            std::cout << "Found new target!\n";
            targetPose_ = newTarget;
            outgoingTask_ = std::make_shared<mpepc::NavigationTask>(targetPose_);
            sentTask_ = false;
        }
    }

    if(sentTask_)
    {
        return { taskStatus(state), nullptr };
    }

    sentTask_ = true;
    return { taskStatus(state), outgoingTask_ };
}


ControlTaskStatus AbsoluteMotionTask::taskStatus(const ControlState& state) const
{
    ControlTaskProgress progress = ControlTaskProgress::failed; // Failure by default, unless otherwise indicated
    
    // Fallback into this state as the default via value_or()
    if(!state.metricPlannerStatus.is_initialized())
    {
        progress = ControlTaskProgress::executing;
    }
    else if(state.metricPlannerStatus.get().status == mpepc::ACTIVE_NORMAL)
    {
        progress = ControlTaskProgress::executing;
    }
    // Guaranteed to be a value at this point
    else if((state.metricPlannerStatus.get().status == mpepc::IDLE) 
        || (state.metricPlannerStatus.get().status == mpepc::PAUSED))
    {
        progress = ControlTaskProgress::waiting;
    }
    else if(state.metricPlannerStatus.get().status == mpepc::SUCCESS_REACHED_POSE)
    {
        progress = ControlTaskProgress::completed;
    }
    
    // If success, send out the success result
    if(progress != ControlTaskProgress::failed)
    {
        return ControlTaskStatus(utils::system_time_us(), id(), progress);
    }
    
    // Otherwise, search for why the error occurred
    ControlTaskError error;
    
    if((state.metricPlannerStatus.get().status == mpepc::FAILURE_CANNOT_FIND_SOLUTION)
        || (state.metricPlannerStatus.get().status == mpepc::FAILURE_UNABLE_TO_PROGRESS))
    {
        error = ControlTaskError::target_unreachable;
    }
    else // if(state.metricPlannerStatus.get().status == mpepc::FAILURE_CANNOT_ASSIGN_TASK)
    {
        error = ControlTaskError::target_invalid;
    }
    
    return ControlTaskStatus(utils::system_time_us(), id(), error);
}


void AbsoluteMotionTask::childIsDone(ControlTask* child, ControlTaskProgress progress)
{
    // There is no child, so if this ever gets executed, assert failure
    std::cerr << "ERROR! AbsoluteMotionTask has no child tasks!\n";
    assert(false);
}


pose_t select_straight_ahead_pose(const pose_t& startPose, const ControlState& state)
{
    const double kTargetDistance = 20.0;
    const double kRobotFrontLength = 0.55;
    const double kRobotWidth = 0.35;

    double maxDist = 0.0;
    double maxHeading = 0.0;
    
    for(float heading = -0.15f; heading <= 0.15f; heading += 0.01f)
    {
        Point<double> frontRightCorner(kRobotFrontLength, -kRobotWidth);
        frontRightCorner = homogeneous_transform(frontRightCorner,
                                                    state.pose.pose().x,
                                                    state.pose.pose().y,
                                                    state.pose.pose().theta + heading);

        Point<double> frontLeftCorner(kRobotFrontLength, kRobotWidth);
        frontLeftCorner = homogeneous_transform(frontLeftCorner,
                                                    state.pose.pose().x,
                                                    state.pose.pose().y,
                                                    state.pose.pose().theta + heading);

        // Find the pose to command the robot to by tracing a ray through the LPM until it hits a wall or the edge of the
        // map. Backup from the collision by the front length of the robot to keep the goal from being inside the wall and
        // thus unreachable by the robot and causing a planner failure
        auto leftEndCell = utils::trace_ray_until_condition(utils::global_point_to_grid_point(frontLeftCorner, *state.map),
                                                            startPose.theta,
                                                            kTargetDistance,
                                                            *state.map,
                                                            [](const hssh::LocalPerceptualMap& lpm, Point<int> cell) {
                                                                return lpm.getCellType(cell.x, cell.y) & (hssh::kUnsafeOccGridCell | hssh::kUnobservedOccGridCell);
        });

        auto rightEndCell = utils::trace_ray_until_condition(utils::global_point_to_grid_point(frontRightCorner, *state.map),
                                                            startPose.theta,
                                                            kTargetDistance,
                                                            *state.map,
                                                            [](const hssh::LocalPerceptualMap& lpm, Point<int> cell) {
            return lpm.getCellType(cell.x, cell.y) & (hssh::kUnsafeOccGridCell | hssh::kUnobservedOccGridCell);
        });

        auto leftEndPoint = utils::grid_point_to_global_point(leftEndCell, *state.map);
        auto rightEndPoint = utils::grid_point_to_global_point(rightEndCell, *state.map);

        auto startPoint = frontLeftCorner;
        auto endPoint = leftEndPoint;

        // Take the minimum of the two distances as the safe distance to travel
        if(distance_between_points(leftEndPoint, frontLeftCorner) >
            distance_between_points(rightEndPoint, frontRightCorner))
        {
            startPoint = frontRightCorner;
            endPoint = rightEndPoint;
        }

        double forwardDist = std::max(distance_between_points(startPoint, endPoint) - (kRobotFrontLength / 4.0), 0.0);
        if((forwardDist * (1.0 - std::abs(heading))) > (maxDist * (1.0 - std::abs(maxHeading))))
        {
            maxDist = forwardDist;
            maxHeading = heading;
        }
    }

    pose_t pose(state.pose.pose().x + maxDist * std::cos(startPose.theta + maxHeading),
                       state.pose.pose().y + maxDist * std::sin(startPose.theta + maxHeading),
                       startPose.theta + maxHeading);

    std::cout << "Selected heading: " << maxHeading << " Dist:" << maxDist << '\n'
        << "Selected pose:" << pose << '\n';

    return pose;
}

} // namespace planner
} // namespace vulcan
