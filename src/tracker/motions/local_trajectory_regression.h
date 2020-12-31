/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_trajectory_regression.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of LocalTrajectoryRegression.
*/

#ifndef TRACKER_MOTIONS_LOCAL_TRAJECTORY_REGRESSION_H
#define TRACKER_MOTIONS_LOCAL_TRAJECTORY_REGRESSION_H

#include "tracker/object_state.h"
#include "core/point.h"
#include <boost/optional.hpp>
#include <nlopt.h>
#include <deque>
#include <vector>
#include <memory>

namespace vulcan
{
namespace tracker
{

struct regression_data_t
{
    int64_t timestamp;
    float   x;
    float   y;
    
    explicit regression_data_t(int64_t timestamp = 0, float x = 0.0f, float y = 0.0f)
    : timestamp(timestamp)
    , x(x)
    , y(y)
    {
    }
};
    
    
/**
* LocalTrajectoryRegression
*/
class LocalTrajectoryRegression
{
public:
    
    static const int kRegressionDimension = 4;
    static const int kIdxX  = 0;
    static const int kIdxY  = 1;
    static const int kIdxVX = 2;
    static const int kIdxVY = 3;
    
    // constructor
    LocalTrajectoryRegression(void);
    
    // Copy constructor
    LocalTrajectoryRegression(const LocalTrajectoryRegression& rhs);
    
    // destructor
    ~LocalTrajectoryRegression(void);
    
    // fit trajectory. Estimated velocity and trajectory are the outputs
    boost::optional<velocity_t> fitTrajectory(const std::deque<regression_data_t>& recentTrajectory, 
                                              std::vector<Position>*               estimatedTrajectory = nullptr);
    
    // cost funtion for fitting the trajectory
    double evaluate(const double x[kRegressionDimension], double* grad);
    
private:
    
    void initializeData (const std::deque<regression_data_t>& recentTrajectory);
    
    // construct estimated trajectory using series of timestamps from data    
    void predictTrajectory(double x0, double y0, double vx, double vy, std::vector<Position>& estimatedTrajectory);
    
    // computing energy and gradient
    double computeCost(void);
    void   computeGradient(double* grad);
    
    // setting up the optimizer
    void setupNLOpt(void);
    
    // optimizer
    nlopt_opt optimizer_;
    
    // data members
    std::vector<regression_data_t> data_;
    std::vector<Position>          prediction_;
    Position                       prior_;
    Point<double>            priorWeights_;
    
    int  numIter_;
    bool haveEnoughData_;

    // parameters
//     regression_params_t params_; // NOTE: hard coded for now
};

} // namespace mpepc
} // namespace vulcan

#endif // TRACKER_MOTIONS_LOCAL_TRAJECTORY_REGRESSION_H
