/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     mpepc_optimizer.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of MPEPCOptimizer.
 */

#include "mpepc/trajectory//mpepc_optimizer.h"
#include "mpepc/control/control_law_coordinates.h"
#include "mpepc/manifold/task_manifold.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/simulator/robot_simulator.h"
#include "mpepc/trajectory/trajectory_evaluator.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "utils/timestamp.h"
#include <cassert>
#include <cmath>
#include <iostream>

// #define DEBUG_MPEPC_OPTIMIZER
// #define DEBUG_INITIAL_TARGETS

namespace vulcan
{
namespace mpepc
{

// helper functions. See end of the file.
double mpepc_objective(unsigned int n, const double* x, double* grad, void* data);
double mpepc_nl_constraint(unsigned int n, const double* x, double* grad, void* data);
void print_nlopt_return_message(int error);


// constructor and destructor.
MPEPCOptimizer::MPEPCOptimizer(const mpepc_optimizer_params_t& params)
: optimizerInfo_(0)
, haveFoundSolution_(false)
, params_(params)
{
    loadGuessesFromFile(params.targetListFile);
}


MPEPCOptimizer::~MPEPCOptimizer(void)
{
}


void MPEPCOptimizer::setup(RobotSimulator& simulator, TrajectoryEvaluator& evaluator)
{
    // simulator and evaluator to use
    simulator_ = &simulator;
    evaluator_ = &evaluator;
}


void MPEPCOptimizer::setInitialGuesses(const TaskManifold& task, const motion_target_t& previousMotionTarget)
{
    newGuesses_.clear();

    // convert goal pose to control law coords
    control_law_coordinates_t goalPoseInControlLawCoords(simulator_->getInitializedSimulatorState().pose,
                                                         task.target(),
                                                         approach_direction_t::FORWARD);

    if (goalPoseInControlLawCoords.r < params_.maxRadius) {
        // add 7 lucky guesses at the goal
        double nominalVelocityGain = params_.minVelocityGain;
        double velocityGainIncrement = (params_.maxVelocityGain - params_.minVelocityGain) / 8.0;
        while (nominalVelocityGain < params_.maxVelocityGain) {
            nominalVelocityGain += velocityGainIncrement;

            optimizer_coords_t goalPoseInOptimizerCoords(goalPoseInControlLawCoords.r,
                                                         goalPoseInControlLawCoords.theta,
                                                         goalPoseInControlLawCoords.delta,
                                                         nominalVelocityGain,
                                                         1.2,   // use reasonable defaults here
                                                         3.0);

            newGuesses_.push_back(goalPoseInOptimizerCoords);

            // add another 7 for driving straight toward the goal position
            goalPoseInOptimizerCoords.theta =
              0.0;   // this makes the orientation of the target pose aligne with the line of sight
            newGuesses_.push_back(goalPoseInOptimizerCoords);
        }
    }

    // add previous motion target (the previous optimum)
    control_law_coordinates_t previousMotionTargetInControlLawCoords(simulator_->getInitializedSimulatorState().pose,
                                                                     previousMotionTarget.pose,
                                                                     previousMotionTarget.direction);

    optimizer_coords_t previousOptimum(previousMotionTargetInControlLawCoords.r,
                                       previousMotionTargetInControlLawCoords.theta,
                                       previousMotionTargetInControlLawCoords.delta,
                                       previousMotionTarget.velocityGain,
                                       previousMotionTarget.k1);

    newGuesses_.push_back(previousOptimum);
}


// optimization process.
motion_target_t MPEPCOptimizer::runOptimizer(trajectory_planner_debug_info_t& debugInfo)
{
    setupGlobalOptimizer();
    setupLocalOptimizer();

    // pointer to the input and output data
    optimizerInfo_ = &debugInfo;

    // robot pose is required for conversion to/from polar egocentric and cartesian global coordinates
    robotPose_ = simulator_->getInitializedSimulatorState().pose;

    // number of iterations
    optimizerInfo_->iteration = 0;

    // optimizer variables
    int nloptValue = -1;
    double cost = 0;

    double x[kOptimizationDimension];
    std::copy(std::begin(globalOptimizerSeed_), std::end(globalOptimizerSeed_), std::begin(x));

    // global optimization for coarse search over the optimizaiton domain.
    if (params_.useGlobalOptimizer) {
        nloptValue = nlopt_optimize(globalOptimizer, x, &cost);

        print_nlopt_return_message(nloptValue);
        std::cout << "    Global optimizer terminated at (" << x[kIdxR] << ',' << x[kIdxTheta] << ',' << x[kIdxDelta]
                  << ',' << x[kIdxVGain] << ',' << x[kIdxK1] << ")\n"
                  << "    with cost " << cost << " after " << optimizerInfo_->iteration << " iterations.\n";
    }
    int globalIterations = optimizerInfo_->iteration;

    if (globalIterations > 0) {
        // save separate copy of the planned trajectory and its evaluation result. This does not increase the iteration
        // count.
        optimizer_coords_t globalOptimizerOutputCoords(x);
        control_law_coordinates_t globalOptimizerOptimizedCoords(globalOptimizerOutputCoords.r,
                                                                 globalOptimizerOutputCoords.theta,
                                                                 globalOptimizerOutputCoords.delta);
        motion_target_t globalOptimizerOptimizedTarget(globalOptimizerOptimizedCoords,
                                                       robotPose_,
                                                       globalOptimizerOutputCoords.velocityGain);
        globalOptimizerOptimizedTarget.k1 = globalOptimizerOutputCoords.k1;

        trajectory.clear();
        simulator_->estimateRobotTrajectory(globalOptimizerOptimizedTarget, trajectory);
        evaluator_->evaluateTrajectory(trajectory);
        optimizerInfo_->coarseOptimizerOutput.assign(trajectory);   // save to debug info
    }

    // local optimization for fine tuning (uses the result from the global optimization as a seed).
    if (params_.useLocalOptimizer) {
        // compare the result of the global optimizer to a set of some hand-picked list of targets, which should include
        // previous optimum and the goal, and use the best as the initial contidition for the local optimizer. NOTE:
        // This step is the ONLY step that can help enforce temporal consistency between consecutive solutions, so it is
        // crtical to have this.
        //       Also, it is important that this step happens here, as the seed to global optimizer usually does not do
        //       anything (see releavant references in NLOPT webpage.)
        selectBestInitialPoint(newGuesses_, x);   // store result in x

        // Intentional double optimization. Do it twice!
        nlopt_optimize(localOptimizer, x, &cost);
        nloptValue = nlopt_optimize(localOptimizer, x, &cost);

        print_nlopt_return_message(nloptValue);
        std::cout << "    Local optimizer terminated at  (" << x[kIdxR] << ',' << x[kIdxTheta] << ',' << x[kIdxDelta]
                  << ',' << x[kIdxVGain] << ',' << x[kIdxK1] << ")\n"
                  << "    with cost " << cost << " after " << (optimizerInfo_->iteration - globalIterations)
                  << " iterations.\n";
    }

    if (optimizerInfo_->iteration - globalIterations > 0) {
        // save separate copy of the planned trajectory and its evaluation result. This does not increase the iteration
        // count.
        optimizer_coords_t localOptimizerOutputCoords(x);
        control_law_coordinates_t localOptimizerOptimizedCoords(localOptimizerOutputCoords.r,
                                                                localOptimizerOutputCoords.theta,
                                                                localOptimizerOutputCoords.delta);
        motion_target_t localOptimizerOptimizedTarget(localOptimizerOptimizedCoords,
                                                      robotPose_,
                                                      localOptimizerOutputCoords.velocityGain);
        localOptimizerOptimizedTarget.k1 = localOptimizerOutputCoords.k1;

        trajectory.clear();
        simulator_->estimateRobotTrajectory(localOptimizerOptimizedTarget, trajectory);
        evaluator_->evaluateTrajectory(trajectory);
        optimizerInfo_->localOptimizerOutput.assign(trajectory);   // save to debug info
    }

    // optimizer status
    haveFoundSolution_ = (nloptValue >= 0);

    std::cout << "MPEPCOptimizer: Found solution? " << haveFoundSolution_ << '\n';

    // WARNING: DO NOT TRUST THE NLOPT RESULT!!!!
    // It does not seem to remember past evaluations, and very often throws away perfectly good solutions that are MUCH
    // better than the final output.

    // manually go through the trajectories to find the *real* optimum among the explored trajectories
    // initial result by the NLOpt
    optimizer_coords_t outputCoords(x);
    control_law_coordinates_t optimizedCoords(outputCoords.r, outputCoords.theta, outputCoords.delta);
    motion_target_t optimizedTarget(optimizedCoords, robotPose_, outputCoords.velocityGain);
    optimizedTarget.k1 = outputCoords.k1;

    double minCost = cost;

    std::size_t bestIdx = bestTrajectoryIdx();
    if (bestIdx < optimizerInfo_->numTrajectories) {
        optimizedTarget = optimizerInfo_->trajectories[bestIdx].motionTarget;
        minCost = optimizerInfo_->trajectories[bestIdx].expectedCost;
    }

    std::cout << "\nMin cost target: " << minCost << " : " << optimizedTarget.pose << "\n\n";

    // reset pointer
    optimizerInfo_ = 0;

    nlopt_destroy(globalOptimizer);
    nlopt_destroy(localOptimizer);

    return optimizedTarget;
}


double MPEPCOptimizer::evaluateCost(const optimizer_coords_t& coords)
{
    // convert optimizer coords into motion target for the simulator
    control_law_coordinates_t candidateCoords(coords.r, coords.theta, coords.delta);
    motion_target_t candidateMotionTarget(candidateCoords, robotPose_, coords.velocityGain);
    candidateMotionTarget.k1 = coords.k1;

    // trajectory simulation
    trajectory.clear();
    simulator_->estimateRobotTrajectory(candidateMotionTarget, trajectory);

    // evaluation of the expected cost
    double expectedCost = evaluator_->evaluateTrajectory(trajectory);

    // save data
    optimizerInfo_->iteration += 1;
    optimizerInfo_->addTrajectory(trajectory);
    coordEvaluated_ = coords;

    // return cost
    return expectedCost;
}


double MPEPCOptimizer::getChanceConstraint(const optimizer_coords_t& coords)
{
    // Constraint violated if something isn't evaluated in the correct order here
    if (optimizerInfo_->trajectories.empty() || (coords != coordEvaluated_)) {
        std::cerr
          << "WARNING: getChanceConstraint: Detected wrong order of cost and constraint check. Assuming failed.\n";
        return -1;
    }

    double minAcceptableChance = params_.useChanceConstraint ? params_.minAcceptableChance : 0.001;

    return -(optimizerInfo_->back().totalSurvivability
             - minAcceptableChance);   // note the negative sign. the constraint is in <= 0 form
}


void MPEPCOptimizer::evaluateGradient(const optimizer_coords_t& coords,
                                      double cost,
                                      double grad[kOptimizationDimension])
{
    double costAtForwardStep[kOptimizationDimension];
    optimizer_coords_t gradientCoords;

    gradientCoords = coords;
    gradientCoords.r += params_.radiusGradientStepSize;
    costAtForwardStep[kIdxR] = evaluateCost(gradientCoords);

    gradientCoords = coords;
    gradientCoords.theta += params_.thetaGradientStepSize;
    costAtForwardStep[kIdxTheta] = evaluateCost(gradientCoords);

    gradientCoords = coords;
    gradientCoords.delta += params_.deltaGradientStepSize;
    costAtForwardStep[kIdxDelta] = evaluateCost(gradientCoords);

    gradientCoords = coords;
    gradientCoords.velocityGain += params_.gainGradientStepSize;
    costAtForwardStep[kIdxVGain] = evaluateCost(gradientCoords);

    gradientCoords = coords;
    gradientCoords.k1 += params_.gainGradientStepSize;
    costAtForwardStep[kIdxK1] = evaluateCost(gradientCoords);

    grad[kIdxR] = (costAtForwardStep[kIdxR] - cost) / params_.radiusGradientStepSize;
    grad[kIdxTheta] = (costAtForwardStep[kIdxTheta] - cost) / params_.thetaGradientStepSize;
    grad[kIdxDelta] = (costAtForwardStep[kIdxDelta] - cost) / params_.deltaGradientStepSize;
    grad[kIdxVGain] = (costAtForwardStep[kIdxVGain] - cost) / params_.gainGradientStepSize;
    grad[kIdxK1] = (costAtForwardStep[kIdxK1] - cost) / params_.gainGradientStepSize;
}


// setting up NLOPT
void MPEPCOptimizer::setupGlobalOptimizer(void)
{
    // select method
    //     globalOptimizer = nlopt_create(NLOPT_GN_DIRECT_L_NOSCAL, kOptimizationDimension);
    //     globalOptimizer = nlopt_create(NLOPT_GN_DIRECT_NOSCAL, kOptimizationDimension);
    //     globalOptimizer = nlopt_create(NLOPT_GN_CRS2_LM, kOptimizationDimension);
    globalOptimizer = nlopt_create(NLOPT_GN_ISRES, kOptimizationDimension);
    //     globalOptimizer = nlopt_create(NLOPT_GN_ESCH, kOptimizationDimension);

    // maximum number of evaluations
    nlopt_set_maxeval(globalOptimizer, params_.maxGlobalIterations);

    // lower bound
    double minBounds[kOptimizationDimension];
    minBounds[kIdxR] = params_.minRadius;
    minBounds[kIdxTheta] = params_.minTheta;
    minBounds[kIdxDelta] = params_.minDelta;
    minBounds[kIdxVGain] = params_.minVelocityGain;
    minBounds[kIdxK1] = params_.minK1;
    nlopt_set_lower_bounds(globalOptimizer, minBounds);

    // upper bound
    double maxBounds[kOptimizationDimension];
    maxBounds[kIdxR] = std::max(0.6 * params_.maxRadius, params_.minRadius + 0.1);
    maxBounds[kIdxTheta] = params_.maxTheta;
    maxBounds[kIdxDelta] = params_.maxDelta;
    maxBounds[kIdxVGain] = std::max(0.6 * params_.maxVelocityGain, params_.minVelocityGain + 0.1);
    maxBounds[kIdxK1] = std::max(params_.maxK1, params_.minK1 + 0.1);
    nlopt_set_upper_bounds(globalOptimizer, maxBounds);

    // other non-linear inequality constraint
    nlopt_add_inequality_constraint(globalOptimizer, mpepc_nl_constraint, this, 0);

    // convergence tolerance for the variable
    //     double xTol[kOptimizationDimension];
    //     xTol[kIdxR]     = params_.globalOptTolMultiplier*params_.radiusTolerance;
    //     xTol[kIdxTheta] = params_.globalOptTolMultiplier*params_.thetaTolerance;
    //     xTol[kIdxDelta] = params_.globalOptTolMultiplier*params_.deltaTolerance;
    //     xTol[kIdxVGain] = params_.globalOptTolMultiplier*params_.gainTolerance;
    //     nlopt_set_xtol_abs(globalOptimizer, xTol);

    // convergence tolerance for the function value
    nlopt_set_ftol_abs(globalOptimizer, params_.globalOptTolMultiplier * params_.functionTolerance);

    // set objective function
    nlopt_set_min_objective(globalOptimizer, mpepc_objective, this);

    // get some initial test points from the speicified bounds
    for (int i = 0; i < kOptimizationDimension; i++) {
        globalOptimizerSeed_[i] = (minBounds[i] + maxBounds[i]) / 2.0;
    }
}


void MPEPCOptimizer::setupLocalOptimizer(void)
{
    // select method
    localOptimizer = nlopt_create(NLOPT_LN_COBYLA, kOptimizationDimension);
    //     localOptimizer = nlopt_create(NLOPT_LD_MMA, kOptimizationDimension);
    //     localOptimizer = nlopt_create(NLOPT_LD_SLSQP, kOptimizationDimension);
    //     localOptimizer = nlopt_create(NLOPT_LN_SBPLX, kOptimizationDimension);
    //     localOptimizer = nlopt_create(NLOPT_LN_BOBYQA, kOptimizationDimension);

    // maximum number of evaluations -- divide by 2 because the local optimization is run twice
    // intentionally
    nlopt_set_maxeval(localOptimizer, params_.maxLocalIterations / 2);

    // lower bound
    double minBounds[kOptimizationDimension];
    minBounds[kIdxR] = params_.minRadius;
    minBounds[kIdxTheta] = params_.minTheta;
    minBounds[kIdxDelta] = params_.minDelta;
    minBounds[kIdxVGain] = params_.minVelocityGain;
    minBounds[kIdxK1] = params_.minK1;
    nlopt_set_lower_bounds(localOptimizer, minBounds);

    // upper bound
    double maxBounds[kOptimizationDimension];
    maxBounds[kIdxR] = params_.maxRadius;
    maxBounds[kIdxTheta] = params_.maxTheta;
    maxBounds[kIdxDelta] = params_.maxDelta;
    maxBounds[kIdxVGain] = params_.maxVelocityGain;
    maxBounds[kIdxK1] = params_.maxK1;
    nlopt_set_upper_bounds(localOptimizer, maxBounds);

    // other non-linear inequality constraint
    nlopt_add_inequality_constraint(localOptimizer, mpepc_nl_constraint, this, 0);

    // convergence tolerance for the variable
    //     double xTol[kOptimizationDimension];
    //     xTol[kIdxR]     = params_.radiusTolerance;
    //     xTol[kIdxTheta] = params_.thetaTolerance;
    //     xTol[kIdxDelta] = params_.deltaTolerance;
    //     xTol[kIdxVGain] = params_.gainTolerance;
    //     nlopt_set_xtol_abs(localOptimizer, xTol);

    // convergence tolerance for the function value
    nlopt_set_ftol_abs(localOptimizer, params_.functionTolerance);

    // set objective function
    nlopt_set_min_objective(localOptimizer, mpepc_objective, this);
}


void MPEPCOptimizer::loadGuessesFromFile(const std::string& filename)
{
    std::ifstream targetFile(filename);

    if (!targetFile.is_open()) {
        std::cerr << "ERROR: MPEPCOptimizer: Failed to open initial targets file: " << filename << std::endl;
        assert(targetFile.is_open());
    }

#ifdef DEBUG_INITIAL_TARGETS
    std::cout << "INFO: MPEPCOptimizer: Loading default targets: (r,theta,delta,vel_gain)\n";
#endif

    while (targetFile.good()) {
        optimizer_coords_t target;

        targetFile >> target.r >> target.theta >> target.delta >> target.velocityGain;

        if (!targetFile.eof() && isTargetValid(target)) {
            defaultGuesses_.push_back(target);

#ifdef DEBUG_INITIAL_TARGETS
            std::cout << '(' << target.r << ',' << target.theta << ',' << target.delta << ',' << target.velocityGain
                      << ")\n";
#endif
        }
    }

    if (defaultGuesses_.empty()) {
        std::cout << "INFO: MPEPCOptimizer: Initial targets file contains no targets!\n";
    }
}


void MPEPCOptimizer::selectBestInitialPoint(const std::vector<optimizer_coords_t>& initialGuesses,
                                            double x[kOptimizationDimension])   // output
{
    // initial guess that is given by the trajectories
    double minCost = HUGE_VAL;
    optimizer_coords_t candidateTarget;

    std::size_t bestIdx = bestTrajectoryIdx();
    if (bestIdx < optimizerInfo_->numTrajectories) {
        const auto& bestTraj = optimizerInfo_->trajectories[bestIdx];

        control_law_coordinates_t bestTrajCoords(robotPose_,
                                                 bestTraj.motionTarget.pose,
                                                 bestTraj.motionTarget.direction);
        optimizer_coords_t globalOptimum(bestTrajCoords.r,
                                         bestTrajCoords.theta,
                                         bestTrajCoords.delta,
                                         bestTraj.motionTarget.velocityGain,
                                         bestTraj.motionTarget.k1);

        minCost = bestTraj.expectedCost;
        candidateTarget = globalOptimum;
    }

    // comparing to default guesses
    for (const auto& guess : defaultGuesses_) {
        double candidateCost = evaluateCost(guess);

        if (candidateCost < minCost) {
            minCost = candidateCost;
            candidateTarget = guess;
        }
    }

    // comparing to other guesses
    for (const auto& guess : initialGuesses) {
        if (isTargetValid(guess)) {
            double candidateCost = evaluateCost(guess);

            if (candidateCost < minCost) {
                minCost = candidateCost;
                candidateTarget = guess;
            }
        }
    }

    candidateTarget.toArray(x);

    std::cout << "    Selected initial point for the local optimizer at (" << x[kIdxR] << ',' << x[kIdxTheta] << ','
              << x[kIdxDelta] << ',' << x[kIdxVGain] << ")\n"
              << "    with cost " << minCost << " after " << (initialGuesses.size() + defaultGuesses_.size())
              << " comparisons.\n";
}


std::size_t MPEPCOptimizer::bestTrajectoryIdx(void)
{
    if (optimizerInfo_->numTrajectories == 0) {
        return 0;
    }

    // iterate through evaluations to find the actual minimum.
    double minCost = optimizerInfo_->trajectories.front().expectedCost;
    std::size_t minIdx = 0;
    for (std::size_t n = 0; n < optimizerInfo_->numTrajectories; ++n) {
        const auto& traj = optimizerInfo_->trajectories[n];

        if ((traj.expectedCost < minCost) && !traj.hasCollision) {
            if ((params_.useChanceConstraint && traj.totalSurvivability > params_.minAcceptableChance)
                || !params_.useChanceConstraint) {
                minCost = traj.expectedCost;
                minIdx = n;
            }
        }
    }

    return minIdx;
}


bool MPEPCOptimizer::isTargetValid(const optimizer_coords_t& target) const
{
    return (target.r >= params_.minRadius) && (target.r <= params_.maxRadius) && (target.theta >= params_.minTheta)
      && (target.theta <= params_.maxTheta) && (target.delta >= params_.minDelta) && (target.delta <= params_.maxDelta)
      && (target.velocityGain >= params_.minVelocityGain) && (target.velocityGain <= params_.maxVelocityGain)
      && (target.k1 >= params_.minK1) && (target.k1 <= params_.maxK1);
}


void print_nlopt_return_message(int error)
{
    if (error < 0) {
        std::cout << "WARNING: MPEPCOptimizer: Optimizer failed: ";
    } else {
        std::cout << "INFO: MPEPCOptimzier: Optimizer found target: ";
    }

    switch (error) {
    case NLOPT_SUCCESS:
        std::cout << "NLOPT_SUCCESS\n";
        break;

    case NLOPT_STOPVAL_REACHED:
        std::cout << "NLOPT_STOPVAL_REACHED\n";
        break;

    case NLOPT_FTOL_REACHED:
        std::cout << "NLOPT_FTOL_REACHED:\n";
        break;

    case NLOPT_XTOL_REACHED:
        std::cout << "NLOPT_XTOL_REACHED:\n";
        break;

    case NLOPT_MAXEVAL_REACHED:
        std::cout << "NLOPT_MAXEVAL_REACHED:\n";
        break;

    case NLOPT_MAXTIME_REACHED:
        std::cout << "NLOPT_MAXTIME_REACHED\n";
        break;

    case NLOPT_FAILURE:
        std::cout << "NLOPT_FAILURE\n";
        break;

    case NLOPT_INVALID_ARGS:
        std::cout << "NLOPT_INVALID_ARGS\n";
        break;

    case NLOPT_OUT_OF_MEMORY:
        std::cout << "NLOPT_OUT_OF_MEMORY\n";
        break;

    case NLOPT_ROUNDOFF_LIMITED:
        std::cout << "NLOPT_ROUNDOFF_LIMITED\n";
        break;

    case NLOPT_FORCED_STOP:
        std::cout << "NLOPT_FORCED_STOP\n";
        break;

    default:
        std::cout << "UNKNOWN_ERROR\n";
    }
}


double mpepc_objective(unsigned int n, const double* x, double* grad, void* data)
{
    // all relevant data stored inside the MPEPCOptimizer object.
    MPEPCOptimizer* optimizer = static_cast<MPEPCOptimizer*>(data);
    optimizer_coords_t coords(x);

    // compute cost
    double cost = optimizer->evaluateCost(coords);

    // compute gradient when required
    if (grad) {
        optimizer->evaluateGradient(coords, cost, grad);
    }

    return cost;
}


double mpepc_nl_constraint(unsigned int n, const double* x, double* grad, void* data)
{
    MPEPCOptimizer* optimizer = static_cast<MPEPCOptimizer*>(data);
    optimizer_coords_t coords(x);

    // get constraint and number of iterations
    double constraintf = optimizer->getChanceConstraint(coords);

    if (grad) {
        assert(false);   // only use gradient-free optimizer!
    }

    return constraintf;
}

}   // namespace mpepc
}   // namespace vulcan
