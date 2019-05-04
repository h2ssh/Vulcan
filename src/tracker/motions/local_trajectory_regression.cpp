/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_trajectory_regression.cpp
* \author   Jong Jin Park and Collin Johnson
*
* Definition of LocalTrajectoryRegression.
*/

#include <tracker/motions/local_trajectory_regression.h>
#include <utils/timestamp.h>
#include <cassert>
#include <cmath>
#include <iostream>

#define DEBUG_LOCAL_TRAJECTORY_REGRESSION_NLOPT

namespace vulcan
{
namespace tracker
{

// helper functions. See end of the file.
double nlopt_objective(unsigned int n, const double* x, double* grad, void* data);
void   print_nlopt_return_message_local_trajectory_regression(int error);


LocalTrajectoryRegression::LocalTrajectoryRegression(void)
: priorWeights_(0.1, 0.1) // TODO: Hard coded params that needs to be replaced.
{
    setupNLOpt();
}


LocalTrajectoryRegression::LocalTrajectoryRegression(const LocalTrajectoryRegression& rhs)
: LocalTrajectoryRegression()
{
    setupNLOpt();
}


LocalTrajectoryRegression::~LocalTrajectoryRegression(void)
{
    nlopt_destroy(optimizer_);
}


boost::optional<velocity_t> LocalTrajectoryRegression::fitTrajectory(const std::deque<regression_data_t>& 
                                                                        recentTrajectory,
                                                                     std::vector<Position>*  estimatedTrj)
{
    if(recentTrajectory.size() < 2 || (recentTrajectory.back().timestamp <= recentTrajectory.front().timestamp))
    {
        return boost::none;
    }
    
    // initialize global variables
    initializeData(recentTrajectory);
    numIter_ = 0; // iterations for the optimizer

    // Optimize regression parameters otherwise.
    // set initial values
    double duration = (recentTrajectory.back().timestamp - recentTrajectory.front().timestamp) / 1000000.0;
    double deltaX   = recentTrajectory.back().x - recentTrajectory.front().x;
    double deltaY   = recentTrajectory.back().y - recentTrajectory.front().y;
    
    int    nloptValue = -1;
    double cost       = 0;
    double x[kRegressionDimension] = {data_.front().x, data_.front().y, deltaX / duration, deltaY / duration};
    
    // optimize
    nloptValue = nlopt_optimize(optimizer_, x, &cost);
    
    if(nloptValue < 0)
    {
        print_nlopt_return_message_local_trajectory_regression(nloptValue);
        return boost::none;
    }
    
    if(estimatedTrj)
    {
        predictTrajectory(x[kIdxX], x[kIdxY], x[kIdxVX], x[kIdxVY], *estimatedTrj);
    }
    
    return velocity_t(x[kIdxVX], x[kIdxVY]);
}


double LocalTrajectoryRegression::evaluate(const double x[kRegressionDimension], double* grad)
{
    predictTrajectory(x[kIdxX], x[kIdxY], x[kIdxVX], x[kIdxVY], prediction_);
    
    computeGradient(grad);
    return computeCost();
}


void LocalTrajectoryRegression::initializeData(const std::deque<regression_data_t>& recentTrajectory)
{
    data_.resize(recentTrajectory.size());
    std::copy(recentTrajectory.begin(), recentTrajectory.end(), data_.begin());
    prediction_.reserve(data_.size());
   
    prior_.x = data_.front().x;
    prior_.y = data_.front().y;
}


void LocalTrajectoryRegression::predictTrajectory(double x0, 
                                                  double y0, 
                                                  double xVel, 
                                                  double yVel, 
                                                  std::vector<Position>& predictedTrajectory)
{
    predictedTrajectory.clear();
    
    int64_t startTime = data_.front().timestamp;
    
    for(std::size_t n = 0; n < data_.size(); ++n)
    {
        double dt = (data_[n].timestamp - startTime) / 1000000.0; // find time elapsed and convert to seconds
        // use a simple linear velocity model
        double x = x0 + (xVel * dt); 
        double y = y0 + (yVel * dt);
        predictedTrajectory.emplace_back(x, y); // add time-synchronized prediction
    }
}


double LocalTrajectoryRegression::computeCost(void)
{
    assert(prediction_.size() == data_.size());
    
    // need the size of data and prediction to be the same, and have more than two elements.
    double cost = 0.0;
    
    // quadratic cost
    for(std::size_t n = 0; n < data_.size(); ++n)
    {
        cost += std::pow(prediction_[n].x - data_[n].x, 2.0) + std::pow(prediction_[n].y - data_[n].y, 2.0);
    }
    
    cost /= data_.size(); // normalize
    
    // add regularizer (ensures continuity)
    double regularizer = (priorWeights_.x * std::abs(prediction_.front().x - prior_.x)) + (priorWeights_.y * 
        std::abs(prediction_.front().y - prior_.y));
    
    return cost + regularizer;
}

void LocalTrajectoryRegression::computeGradient(double* grad)
{
    if(grad == NULL)
    {
        return;
    }
    
    assert(prediction_.size() == data_.size());
    
    // gradient for the quadratic cost
    std::fill(grad, grad+kRegressionDimension, 0.0);
    
    int64_t startTime = data_.front().timestamp;
    
    for(std::size_t n = 0; n < data_.size(); ++n)
    {
        double timeElapsed = (data_[n].timestamp - startTime) / 1000000.0;
        double xDiff = prediction_[n].x - data_[n].x;
        double yDiff = prediction_[n].y - data_[n].y;
        
        grad[kIdxX]  += 2 * xDiff;             // dcost/dx0 at each datapoint
        grad[kIdxY]  += 2 * yDiff;             // dcost/dy0
        grad[kIdxVX] += 2 * xDiff * timeElapsed; // dcost/dVx 
        grad[kIdxVY] += 2 * yDiff * timeElapsed; // dcost/dVy
    }
    
    // Normalize the gradient
    for(int i = 0; i < kRegressionDimension; ++i)
    {
        grad[i] /= data_.size();
    }
    
    // add gradient for the regularizer
    grad[kIdxX] += priorWeights_.x; // dreg/dx0
    grad[kIdxY] += priorWeights_.y; // dreg/dy0
}


// setting up NLOPT
void LocalTrajectoryRegression::setupNLOpt(void)
{
    double fTol = 1e-4;
    double xTol[kRegressionDimension];
    double initialStep[kRegressionDimension];
    
    xTol[kIdxX]  = 1e-3;
    xTol[kIdxY]  = 1e-3;
    xTol[kIdxVX] = 1e-3;
    xTol[kIdxVY] = 1e-3;
        
    initialStep[kIdxX]   = 0.05;
    initialStep[kIdxY]   = 0.05;
    initialStep[kIdxVX]  = 0.05;
    initialStep[kIdxVY]  = 0.05;
    
    optimizer_ = nlopt_create(NLOPT_LD_SLSQP, kRegressionDimension);
    
    nlopt_set_min_objective(optimizer_, nlopt_objective, this);
    
    nlopt_set_maxeval (optimizer_, 100);
    nlopt_set_ftol_rel(optimizer_, fTol);
    nlopt_set_xtol_abs(optimizer_, xTol);
    
    nlopt_set_initial_step(optimizer_, initialStep);

}

void print_nlopt_return_message_local_trajectory_regression(int error)
{
    if(error < 0)
    {
        std::cout<<"WARNING: LocalTrajectoryRegression: Optimizer failed: ";
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


double nlopt_objective(unsigned int n, const double* x, double* grad, void* data)
{
    // all relevant data stored inside the MPEPCOptimizer object.
    LocalTrajectoryRegression* regression = static_cast<LocalTrajectoryRegression*>(data);
    return regression->evaluate(x, grad);
}

} // namespace tracker
} // namespace vulcan
