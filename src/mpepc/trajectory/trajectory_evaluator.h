/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     trajectory_evaluator.h
* \author   Jong Jin Park
*
* Declaration of TrajectoryEvaluator for evaluating the trajectory over a task manifold.
*/

#ifndef MPEPC_TRAJECTORY_EVALUATOR_H
#define MPEPC_TRAJECTORY_EVALUATOR_H

#include <mpepc/trajectory/params.h>
#include <mpepc/collision/robot_collision_model.h>
#include <mpepc/simulator/dynamic_object_trajectory.h>
#include <mpepc/metric_planner/task/task.h>
#include <string>
#include <memory>

namespace vulcan
{
struct pose_t;
struct velocity_t;
struct acceleration_t;

namespace robot { struct collision_model_params_t; }

namespace mpepc
{

// robot_trajectory_info_t stores both the trajectory to be evaluated (input)
// and the evaluation results (output).
struct robot_trajectory_info_t;

/**
* TrajectoryEvaluator calculates an expected cost of a trajectory. The notion of
* expected cost was first defined in "Robot Navigation with Model Predictive
* Equilibrium Point Control", by Park, Johnson and Kuipers, IROS 2012.
*
* The cost is a direct sum of expected progress, expected cost of collision and expected cost of action.
* (1) To compute expected progress, piecewise progress is calculated at each simulated step (or at some sub-sampled
*     steps for speed, as long as they are close enough so collision checking is valid.) The progress at each segment
*     is then mutiplied with the survivability, the probability that the trajectory at that step is going to be
*     collision free, to compute expected progress.
* (2) Expected cost of collision is computed similarly. The raw cost of collision is multiplied with the probability of
*     collision and we get expected cost of collision.
* (3) Expected cost of action is equal to raw cost of action, which is defined as a quadratic cost of velocities
*     along the simulated trajectory in the current implementation.
*
* For the expected progress, we can write
*
* expected_progress(X) = Sigma_(k=1)^(k=N) {survivability(x(k))*progress(k)} + angle_heuristic(x(N))
*
* where x(k) is a robot pose at time k in [0, ... ,N], along a trajectory X = {x(0) ... x(N)} to be evaluated,
* with N*timestep = T, the time horizon.
*
* We define the survivability(x(k)) as the approximated probability of the robot pose x(k) being
* collision free, and the progress(k) is the reduction in the cost-to-go toward the goal at that trajectory segment,
* e.g. NF(x(k)) - NF(x(k-1)) in the case of navigation function. angle_heuristic(x(N)) is an additional heuristic
* which promotes alignment of the final pose to the gradient of the navigation function, which of course is only valid
* if we are computing progress over the navigation function.
*
* Note that without the survivability term, which can be understood as a state-dependent, time-varying weight,
* the expected progress reduces to simple progress. which is
*
* progress(X) = NF(x(0)) - NF(x(N)) + angle_heuristic(x(N)).
*
* So the expected progress is the simple progress shown above multiplied by the time-varying probability weights at each segment.
* What does this mean? Usually, a cost function in an optimization problem is constructed as a constant-weighted sum of
* (often conflicting) sub-objectives. Then almost invariably determination of those weights becomes the real problem,
* as ill-chosen weights could lead to multiple local minima, unsmooth cost surface, etc, which results in undesirable behaviors.
* But when I have tried something like that, it was very difficult to find a set of constant weights that gave good results
* across wide range of situations.
*
* One of the key contribution of this formulation is that now we can compute those weights on-line, as a function of states
* along the simulated trajectory, in such a way that it is very easy to combine different objectives and create a nice, smooth
* cost surface that is easy to optimize over. Experimentally, the difference in robot behaviors generated from the simple progress
* with constan weights to the expected progress with probabilistic weights was very readily observable.
*
*
* UPDATE: When using the non-NF progress based on the polar coordinate, this angle_heuristic is not used anymore.
*
*
* ADDED: Now we also add negative dwell time at the goal to the progress and change the name of the variable from 'progress' to 'reward'.
* Functionally, this motivates the robot to move to the goal quickly and stay there. So what we have now, looks like:
*
* expected_reward(X) = Sigma_(k=1)^(k=N) {survivability(x(k)) * [progress(k) - dwellTime(k)] }
*
*/

class TrajectoryEvaluator
{
public:

    /**
    * constructor for TrajectoryEvaluator.
    */
    TrajectoryEvaluator(const trajectory_evaluator_params_t& params, const robot::collision_model_params_t& robotParams);

    /**
    * destructor for TrajectoryEvaluator.
    */
    ~TrajectoryEvaluator(void) {};

    /**
    * setTaskEnvironment links task and environment data to the evaluator via pointers, giving
    * the essential information to the evaluator.
    *
    * \param    taskManifold                Task and progress toward task completion
    * \param    obstacleDistanceGrid        Grid map of distance to the nearest static obstacle
    * \param    estimatedObjectTrajectories Estimated trajectories of dynamic objects
    */
    void setTaskEnvironment(const TaskManifold&                             taskManifold,
                            const ObstacleDistanceGrid&                     obstacleDistanceGrid,
                            const std::vector<dynamic_object_trajectory_t>& estimatedObjectTrajectories);

    /**
    * getClearanceToStaticObstacles does what it expects to do, from the given robot state.
    *
    * \param    robotState                  State of the robot
    * \internal obstacleDistanceGrid, a grid map of distance to the nearest static obstacle.
    * \return   clearance to the nearest static obstacle from robot boundary
    */
    float getClearanceToStaticObstacles(const motion_state_t& robotState);

    /**
    * getClearanceToDynamicObjects returns the distance to the nearest dynamic object,
    * from the given robot state and initial (observed) states of tracked dynamic objects.
    *
    * \param    robotState                  State of the robot
    * \internal trackedObject in estimatedObjecTrajectories.
    * \return   clearance to the nearest dynamic obect from robot boundary
    */
    float getClearanceToDynamicObjects (const motion_state_t& robotState);

    /**
    * evaluateTrajectory evaluates the cost of a given trajectory. Low cost
    * trajectories are better than high cost trajectories. This method requires
    * the data to be correctly setup. It is user's responsibility to correctly
    * setup the data.
    *
    * \param    robotTrajectory     Trajectory to be evaluated. (pass by reference)
    * \internal uses task manifold, obstacle distance grid, esimated object
    *           trajectories, and robot collision model.
    * \return   Cost of the trajectory.Lower cost are better.
    *           Also populates the input struct with evaluation results.
    */
    double evaluateTrajectory(robot_trajectory_info_t& robotTrajectory);


private:

    /**
    * piecewiseCollisionProbabiltiy evaluates a trajectory segment, represented
    * by its end pose, and returns the estimated probability of the collision
    * against static structure or a dynamic object.
    *
    * \param    endPose               Final pose of the robot in the trajectory segment
    * \param    velocities            Final velocity of the robot in the trajectory segment
    * \param    objectState           Position (and velocity) of the object of interest
    * \param    objectRadius          Radius of the object of interest (we assume circular object for now)
    * \param    timestep              Duration of the trajectory segment
    * \param    uncertaintySigma      (Output) Sigma value for the positional uncertainty at the previous pose
    * \param    distance              (Output) Euclidean distance to the nearest object
    * \return   Probability of collision over the trajectory segment specifed by the endPose
    */
    double piecewiseCollisionProbability(const pose_t&          endPose,
                                         const velocity_t&      velocities,
                                         float                         timestep,
                                         double*                       robotUncertaintySigma,
                                         double*                       distance = 0);

    double piecewiseCollisionProbability(const pose_t&          endPose,
                                         const velocity_t&      velocities,
                                         double                        robotUncertaintySigma,
                                         const dynamic_object_state_t& objectState,
                                         float                         objectRadius,
                                         float                         timestep,
                                         double*                       objectUncertaintySigma,
                                         double*                       distance = 0);

    /**
    * piecewiseActionCost evaluates a trajectory segment using its velocity and acceleration, and returns the estimated cost of action along the
    * trajectory segment.
    *
    * \param    velocity        Robot velocity over the trajectory segment
    * \param    acceleration    Robot acceleration over the trajectory segment
    * \param    timestep        Duration of the trajectory segment
    * \return   Cost of action over the trajectory segment
    */
    double piecewiseActionCost(const velocity_t&     velocity,
                               const acceleration_t& acceleration,
                               float                        timestep);

    /**
    * piecewiseCollisionCost evaluates a trajectory segment, and returns the estimated cost of collision assuming that the trajectory
    * segment is indeed in collision.
    *
    * \param    endState        Final state of the robot in the trajectory segment
    * \param    timestep        Duration of the trajectory segment
    * \return   Cost of collision over the trajectory segment
    */
    double piecewiseCollisionCost(const motion_state_t& endState, float timestep);


    /**
    * socialDistanceCost evaluates a trajectory segment, and returns the estimated cost of staying too close to a pedestrian.
    *
    * \param    endPose         Final Pose of the robot in the trajectory segment
    * \param    objectState     Position (and velocity) of the object of interest
    * \param    objectRadius    Radius of the object of interest (we assume circular object for now)
    * \param    timestep        Duration of the trajectory segment
    * \return   Cost of being in the proximity of a dynamic object
    */
    double socialDistanceCost(const pose_t& endPose, const dynamic_object_state_t& objectState, float objectRadius, float timestep);


    // NOTE: paths and trajectories are different! The key difference is that for the path, time is not a relevant quantity,
    //       while for a trajectory it is a defining quantity. In discrete setting, a trajectory can be described as a time-stamped
    //       series of poses, but a path is a series of poses that does not require the time stamp.
    //       This distinction is important as path-related quantities are invariant under velocity transformation, e.g. path length
    //       remains constant regardless of the velocity of the travel, but the action cost explicitly depend on them.
    /**
    * integratePathLength integrates the length of traveled along the path resulting from the evaluated trajectory.
    */
    void integratePathLength(double& pathLength, float linearVelocity, float timestep);

    /**
    * integratePathCost integrates the distance-related cost of travel along the path resulting from the evaluated trajectory.
    */
    void integratePathCost  (double& pathCost, const velocity_t& velocity, float timestep);

    // robot collision model
    std::unique_ptr<RobotCollisionModel> robot_;

    // pointers to task, map and object information.
    const TaskManifold*                             task_;
    const ObstacleDistanceGrid*                     map_;

    std::vector<dynamic_object_trajectory_t> objects_;

    double staticObjectUncertaintySigma_;

    // parameters for trajectory evaluation
    trajectory_evaluator_params_t params_;
};

} // mpepc
} // vulcan

#endif // MPEPC_TRAJECTORY_EVALUATOR_H
