/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mpepc_pose_follower.cpp
* \author   Jong Jin Park
*
* Definition of MPEPCPoseFollower.
*/

#include "mpepc/trajectory//mpepc_pose_follower.h"
#include "mpepc/simulator/robot_simulator.h"
#include "mpepc/trajectory/trajectory_evaluator.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "mpepc/control/control_law_coordinates.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/manifold/task_manifold.h"
#include "utils/timestamp.h"
#include <cassert>
#include <cmath>
#include <iostream>

// #define DEBUG_MPEPC_POSE_FOLLOWER

namespace vulcan
{
namespace mpepc
{

// helper functions. See end of the file.
double mpepc_pose_follower_objective(unsigned int n, const double* x, double* grad, void* data);
void   print_nlopt_return_message_mpepc_pose_follower(int error);

// constructor and destructor.
MPEPCPoseFollower::MPEPCPoseFollower(const mpepc_optimizer_params_t& params)
: optimizerInfo_(0)
, haveFoundSolution_(false)
, params_(params)
{
    setupNLOpt();
}


MPEPCPoseFollower::~MPEPCPoseFollower(void)
{
    nlopt_destroy(globalOptimizer);
    nlopt_destroy(localOptimizer);
}


void MPEPCPoseFollower::setup(RobotSimulator& simulator, TrajectoryEvaluator& evaluator)
{
    // simulator and evaluator to use
    simulator_ = &simulator;
    evaluator_ = &evaluator;
}


void MPEPCPoseFollower::setInitialGuesses(const TaskManifold& task, const motion_target_t& previousMotionTarget)
{

    // current destination pose, is sent to the optimizer as an initial guess,
    //  assuming forward approach direction, with some default velocity gain
    targetPose_ = task.target();

    candidateMotionTarget_ = motion_target_t(targetPose_);
    candidateMotionTarget_.velocityGain = previousMotionTarget.velocityGain;
}


// optimization process.
motion_target_t MPEPCPoseFollower::runOptimizer(trajectory_planner_debug_info_t& debugInfo)
{
    // pointer to the input and output data
    optimizerInfo_ = &debugInfo;

    // robot pose is required for conversion to/from polar egocentric and cartesian global coordinates
    robotPose_ = simulator_->getInitializedSimulatorState().pose;

    // number of iterations
    optimizerInfo_->iteration = 0;

    // optimizer variables
    int    nloptValue = -1;
    double cost = 0;

    double x[] = {(params_.minVelocityGain + params_.maxVelocityGain) / 2.0};

    // global optimization (does not require seed) for coarse search over the optimizaiton domain.
    if(params_.useGlobalOptimizer)
    {
        nloptValue = nlopt_optimize(globalOptimizer, x, &cost);

        print_nlopt_return_message_mpepc_pose_follower(nloptValue);
        std::cout<<"    Global optimizer terminated at ("<<x[0]<<")\n"
                 <<"    with cost "<<cost<<" after "<< optimizerInfo_->iteration<<" iterations.\n";
    }
    int globalIterations = optimizerInfo_->iteration;

    if(globalIterations > 0)
    {
        // save separate copy of the planned trajectory and its evaluation result. This does not increase the iteration count.
        motion_target_t optimizedTarget(targetPose_);
        optimizedTarget.velocityGain = x[0];

        trajectory.clear();
        simulator_->estimateRobotTrajectory(optimizedTarget, trajectory);
        evaluator_->evaluateTrajectory(trajectory);
        optimizerInfo_->coarseOptimizerOutput.assign(trajectory); // save to debug info
    }

    // local optimization for fine tuning (uses the result from the global optimization as a seed).
    if(params_.useLocalOptimizer)
    {
        x[0] = candidateMotionTarget_.velocityGain;

        nloptValue = nlopt_optimize(localOptimizer, x, &cost);

        print_nlopt_return_message_mpepc_pose_follower(nloptValue);
        std::cout<<"    Local optimizer terminated at  ("<<x[0]<<")\n"
                 <<"    with cost "<<cost<<" after "<<(optimizerInfo_->iteration - globalIterations)<<" iterations.\n";
    }

    if(optimizerInfo_->iteration - globalIterations > 0)
    {
        // save separate copy of the planned trajectory and its evaluation result. This does not increase the iteration count.
        motion_target_t optimizedTarget(targetPose_);
        optimizedTarget.velocityGain = x[0];

        trajectory.clear();
        simulator_->estimateRobotTrajectory(optimizedTarget, trajectory);
        evaluator_->evaluateTrajectory(trajectory);
        optimizerInfo_->localOptimizerOutput.assign(trajectory); // save to debug info
    }

    // optimizer status
    haveFoundSolution_ = (nloptValue >= 0);

    // WARNING: DO NOT TRUST THE NLOPT RESULT!!!!
    // It does not seem to remember past evaluations, and very often throws away perfectly good solutions that are MUCH better than the final output.

    // manually go through the trajectories to find the *real* optimum among the explored trajectories
    // initial result by the NLOpt
    motion_target_t optimizedTarget(targetPose_);
    optimizedTarget.velocityGain = x[0];

    // iterate through evaluations to find the actual minimum.
    double minCost = cost;
    for(const auto& evaluatedTrajectory : optimizerInfo_->trajectories)
    {
        if(evaluatedTrajectory.expectedCost < minCost && !evaluatedTrajectory.hasCollision)
        {
            optimizedTarget = evaluatedTrajectory.motionTarget;
            minCost = evaluatedTrajectory.expectedCost;
        }
    }

    // reset pointer
    optimizerInfo_ = 0;

    return optimizedTarget;

//     // send output
//     candidateMotionTarget_.velocityGain = x[0];
//     return candidateMotionTarget_;
}


double MPEPCPoseFollower::evaluateCost(const double x[])
{
    candidateMotionTarget_.velocityGain = x[0];

    // trajectory simulation
    trajectory.clear();
    simulator_->estimateRobotTrajectory(candidateMotionTarget_, trajectory);

    // evaluation of the expected cost
    double expectedCost = evaluator_->evaluateTrajectory(trajectory);

    // save data
    optimizerInfo_->iteration += 1;
    optimizerInfo_->addTrajectory(trajectory);

    // return cost
    return expectedCost;
}


void MPEPCPoseFollower::evaluateGradient(const double x[], double cost, double grad[])
{
    int kIdxVGain = 0;

    double forwardStep[1];
    double costAtForwardStep[1];

    forwardStep[kIdxVGain]       = x[kIdxVGain] + params_.gainGradientStepSize;
    costAtForwardStep[kIdxVGain] = evaluateCost(forwardStep);

    grad[kIdxVGain] = (costAtForwardStep[kIdxVGain] - cost) / params_.gainGradientStepSize;
}


// setting up NLOPT
void MPEPCPoseFollower::setupNLOpt(void)
{
    const int kOptimizationDimension = 1;
    int kIdxVGain = 0;

    double minBounds[kOptimizationDimension];
    double maxBounds[kOptimizationDimension];
//     double xtolForGlobalOptimizer[kOptimizationDimension];
//     double xtolForLocalOptimizer[kOptimizationDimension];
    double initialStepForLocalOptimizer[kOptimizationDimension];

    minBounds[kIdxVGain]  = params_.minVelocityGain*0.0 + 0.6;
    maxBounds[kIdxVGain]  = params_.maxVelocityGain;
//     xtolForGlobalOptimizer[kIdxVGain] = params_.globalOptTolMultiplier*params_.gainTolerance;
//     xtolForLocalOptimizer[kIdxVGain]  = params_.gainTolerance;
    initialStepForLocalOptimizer[kIdxVGain]  = params_.gainInitialStep;

    // fist-phase coarse global optimization
    globalOptimizer = nlopt_create(NLOPT_GN_DIRECT_NOSCAL, kOptimizationDimension);
//    globalOptimizer = nlopt_create(NLOPT_GN_CRS2_LM, kOptimizationDimension);

    nlopt_set_maxeval(globalOptimizer, params_.maxGlobalIterations);
    nlopt_set_min_objective(globalOptimizer, mpepc_pose_follower_objective, this);
    nlopt_set_lower_bounds(globalOptimizer, minBounds);
    nlopt_set_upper_bounds(globalOptimizer, maxBounds);
    nlopt_set_ftol_abs(globalOptimizer, params_.globalOptTolMultiplier*params_.functionTolerance);
//     nlopt_set_xtol_abs(globalOptimizer, xtolForGlobalOptimizer);


    // second-phase local gradient based optimization
//     localOptimizer = nlopt_create(NLOPT_LD_MMA, kOptimizationDimension);
//     localOptimizer = nlopt_create(NLOPT_LD_SLSQP, kOptimizationDimension);
//     localOptimizer = nlopt_create(NLOPT_LN_SBPLX, kOptimizationDimension);
//     localOptimizer = nlopt_create(NLOPT_LN_BOBYQA, kOptimizationDimension);
    localOptimizer = nlopt_create(NLOPT_LN_COBYLA ,kOptimizationDimension);

    nlopt_set_initial_step(localOptimizer, initialStepForLocalOptimizer);
    nlopt_set_maxeval(localOptimizer, params_.maxLocalIterations);
    nlopt_set_min_objective(localOptimizer, mpepc_pose_follower_objective, this);
    nlopt_set_lower_bounds(localOptimizer, minBounds);
    nlopt_set_upper_bounds(localOptimizer, maxBounds);
    nlopt_set_ftol_abs(localOptimizer, params_.functionTolerance);
//     nlopt_set_xtol_abs(localOptimizer, xtolForLocalOptimizer);
}



void print_nlopt_return_message_mpepc_pose_follower(int error)
{
    if(error < 0)
    {
        std::cout<<"WARNING: MPEPCPoseFollower: Optimizer failed: ";
    }
    else
    {
        std::cout<<"INFO: MPEPCOptimzier: Optimizer found target: ";
    }

    switch(error)
    {
    case NLOPT_SUCCESS:
        std::cout<<"NLOPT_SUCCESS\n";
        break;

    case NLOPT_STOPVAL_REACHED:
        std::cout<<"NLOPT_STOPVAL_REACHED\n";
        break;

    case NLOPT_FTOL_REACHED:
        std::cout<<"NLOPT_FTOL_REACHED:\n";
        break;

    case NLOPT_XTOL_REACHED:
        std::cout<<"NLOPT_XTOL_REACHED:\n";
        break;

    case NLOPT_MAXEVAL_REACHED:
        std::cout<<"NLOPT_MAXEVAL_REACHED:\n";
        break;

    case NLOPT_MAXTIME_REACHED:
        std::cout<<"NLOPT_MAXTIME_REACHED\n";
        break;

    case NLOPT_FAILURE:
        std::cout<<"NLOPT_FAILURE\n";
        break;

    case NLOPT_INVALID_ARGS:
        std::cout<<"NLOPT_INVALID_ARGS\n";
        assert(false);
        break;

    case NLOPT_OUT_OF_MEMORY:
        std::cout<<"NLOPT_OUT_OF_MEMORY\n";
        break;

    case NLOPT_ROUNDOFF_LIMITED:
        std::cout<<"NLOPT_ROUNDOFF_LIMITED\n";
        break;

    case NLOPT_FORCED_STOP:
        std::cout<<"NLOPT_FORCED_STOP\n";
        break;

    default:
        std::cout<<"UNKNOWN_ERROR\n";
    }
}


double mpepc_pose_follower_objective(unsigned int n, const double* x, double* grad, void* data)
{
    // all relevant data stored inside the MPEPCPoseFollower object.
    MPEPCPoseFollower* optimizer = static_cast<MPEPCPoseFollower*>(data);

    // compute cost
    double cost = optimizer->evaluateCost(x);

    // compute gradient when required
    if(grad)
    {
        optimizer->evaluateGradient(x, cost, grad);
    }

    return cost;
}



} // mpepc
} // vulcan
