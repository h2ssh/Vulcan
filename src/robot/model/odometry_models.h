/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     odometry_models.h
* \author   Collin Johnson
* 
* Definition of InverseOdometryModel
*/

#ifndef ROBOT_MODEL_ODOMETRY_MODELS_H
#define ROBOT_MODEL_ODOMETRY_MODELS_H

#include "robot/model/motion_model.h"

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace robot
{
    
/**
* odometry_model_params_t contains the parameters defining the noise estimates
* for the odometry distribution.
*/
struct odometry_model_params_t
{
    // Parameters pulled straight from Probabilistic Robotics
    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    
    odometry_model_params_t(const utils::ConfigFile& config);
};
    
/**
* InverseOdometryModel
*/
class InverseOdometryModel : public InverseMotionModel
{
public:
    
    /**
    * Constructor for InverseOdometryModel.
    * 
    * \param    params          Parameters for the model
    */
    InverseOdometryModel(const odometry_model_params_t& params);
    
    // InverseMotionModel interface
    virtual pose_distribution_t predictPose(const pose_distribution_t& previousPose, 
                                            const motion_model_data_t& previousData, 
                                            const motion_model_data_t& currentData);
    
private:
    
    odometry_model_params_t params_;
    
};
    
}
}

#endif // ROBOT_MODEL_ODOMETRY_MODELS_H
