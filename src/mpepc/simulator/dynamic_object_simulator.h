/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object_simulator.h
* \author   Jong Jin Park
*
* Declaration of the DynamicObjectSimulator
*
*/

#ifndef MPEPC_DYNAMIC_OBJECT_SIMULATOR_H
#define MPEPC_DYNAMIC_OBJECT_SIMULATOR_H

#include <mpepc/simulator/dynamic_object_trajectory.h>
#include <mpepc/simulator/params.h>

namespace vulcan
{
struct motion_state_t;

namespace mpepc
{

class ObstacleDistanceGrid;

/**
* DynamicObjectSimulator estimates future trajectories of dynamic objects based on the estimated state of the robot
* and the information of the static environment.
*
* In simulations, objects use MPC-like lookahead-and-choose-velocity strategy to avoid collision. We probaly could use
* more sophisticated motion models (especially for pedestrians).
*/
class DynamicObjectSimulator
{
public:

    /**
    * Constructor for DynamicObjectSimulator.
    *
    * \param    params          parameters for the simulator.
    */
    DynamicObjectSimulator(const dynamic_object_simulator_params_t& params);

    // set time step and time horizon
    void setTimeStep  (float timeStep)   { simulatorTimeStep_    = timeStep; };
    void setTimeLength(float timeLength) { trajectoryTimeLength_ = timeLength; };

    void setModeQuasiStatic(bool tf) { shouldIgnoreObjectVelocities_ = tf; };

    /**
    * estimateObjectTrajectories initialize the state variables and estimate trajectories of dynamic objects of interest for the given duration specified in the parameters.
    *
    * \param[in,out] objects        Dynamic objects around the robot
    * \param         robotState     the most recent state of the robot received by the metric planner.
    * \param         map            a grid map contianing the minimum distance to the nearest static obstacle in the map
    * \param         startTimeUs    start time of the estimated trajectory in microseconds.
    */
    void estimateObjectTrajectories(std::vector<dynamic_object_trajectory_t>& objects,
                                    const motion_state_t&              robotState,
                                    const ObstacleDistanceGrid&               map,
                                    int64_t                                   startTimeUs);

private:

    float simulatorTimeStep_;
    float trajectoryTimeLength_;
    bool  shouldIgnoreObjectVelocities_;

    std::vector<dynamic_object_state_t> predictedRobotTraj_;    // velocity is unit vector

    dynamic_object_simulator_params_t params_;


    dynamic_object_state_t propagateObjectState(const dynamic_object_state_t& objectState,
                                                float objectRadius,
                                                const ObstacleDistanceGrid& map,
                                                const dynamic_object_state_t* robotState,
                                                const Point<float>& preferredVel,
                                                const pose_t* objectGoal,
                                                float timeElapsed,
                                                float timeStep);

    // Object controller model
    Point<float> objectVelocityDecider(const dynamic_object_state_t& objectState,
                                             float objectRadius,
                                             const ObstacleDistanceGrid& map,
                                             const dynamic_object_state_t* robotState,
                                             const Point<float>& preferredVel,
                                             const pose_t* objectGoal,
                                             float timeElapsed,
                                             float timeStep);
};

} // mpepc
} // vulcan

#endif // MPEPC_DYNAMIC_OBJECT_SIMULATOR_H
