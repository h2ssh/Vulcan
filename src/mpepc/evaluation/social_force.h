/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     social_force.h
 * \author   Collin Johnson
 *
 * Declaration of interaction_social_forces for computing various social force metrics for interactions between the
 * robot and other objects in the environment:
 *
 *   - force : how close two objects are
 *   - blame : like force, but takes into account velocity information to determine who is the cause of the force
 *   - passing distance : the robot is considered passing if the projection of the object falls on the left or right
 *                        side of the robot and the object is interacting with the robot
 *
 * An additional utility function trajectory_social_forces that computes forces for a sequence of interactions.
 */

#ifndef MPEPC_EVALUATION_SOCIAL_FORCE_H
#define MPEPC_EVALUATION_SOCIAL_FORCE_H

#include "core/pose.h"
#include "core/position.h"
#include "math/geometry/rectangle.h"
#include <vector>

namespace vulcan
{
namespace mpepc
{

struct interaction_t;

/**
 * passing_object_t is an object that the robot is passing by or that is passing the robot.
 */
struct passing_object_t
{
    int id = 1;   ///< Monotonically increasing count of passing events will likely be multiple passing_object_t per
                  ///< event because they are generated on a per motion-state basis. Use the id to distinguish when
                  ///< one event has ended and another has begun
                  ///< The highest id is the count of how many passing events occurred

    position_t position;          ///< Position of the object
    double passingDist;           ///< Distance between object and robot boundary
    double passingSpeed;          ///< Relative speed of robot to object (negative means agent passing us)
    math::RectSide passingSide;   ///< Which side the agent being passed was passed on. If passingSpeed is
                                  ///< positive, then it is the side the agent was passed. If passingSpeed
                                  ///< is negative, it is the side the robot was passed on. Summary: always
                                  ///< prefer to pass on the left when possible.

    passing_object_t(position_t position, double distance, double speed, math::RectSide side)
    : position(position)
    , passingDist(distance)
    , passingSpeed(speed)
    , passingSide(side)
    {
    }
};

/**
 * social_forces_t defines the social forces for a particular robot pose in a trajectory. The forces on robot are
 * force and blame. The position of the objects corresponding to these values is also provided for visualization
 * purposes.
 */
struct social_forces_t
{
    int64_t timestamp;     ///< Time at which these forces were computed
    pose_t pose;           ///< Pose of the robot at this time
    bool isInteracting;    ///< Flag indicating if the robot was actually interacting with another object
                           ///< at this time or if it was in a temporarily static environment
    position_t forceObj;   ///< Position of the object causing the force on the robot
    position_t blameObj;   ///< Position of the object causing the blame on the robot
    double force;          ///< Measured social force on the robot
    double blame;          ///< Measured blame for the robot

    // NOTE: Only support a single pass per event, so driving between two people will only be counted as one passed
    // person and it will select whoever was passed closer. Not that big a deal, but just something to be cognizant of
    std::vector<passing_object_t> passingObj;   ///< Objects robot was passing at the time
};

/**
 * social_forces_params_t defines parameters that control the computation of the social forces for the robot.
 *
 * The social forces equations are:
 *
 *   force = a_p e(-d_obj / b_p) * w(theta)
 *       w(theta) = lambda + ((1-lambda) * (1 + cos(theta)) / 2)
 *       cos(theta) = -n_i * e_r, where n_i is vector pointing to obj_i and e_r is vector of robot motion
 *   blame = max phi(||p*_ij(t) - p_j(t)||)
 *       phi = sigmoid
 *       p*_ij(t) = proj p_i(t) onto p_j(t) + v_j*tau
 *
 * Default values taken from Ferrer IROS'13 and Mehta ICRA'16.
 */
struct social_forces_params_t
{
    double a_p = 2.66;        ///< Magnitude of social force
    double b_p = 0.79;        ///< Falloff distance for social force
    double lambda_p = 0.59;   ///< Constant associated with anisotropic scaling of force

    double tau = 0.5;   ///< Lookahead time for blame

    double robotRadius = 0.35;   ///< Radius to use for the robot -- width of the robot for rectangle

    /// Boundary for passing distance
    math::Rectangle<float> robotBoundary = math::Rectangle<float>(Point<float>(0.55, 0.32),
                                                                  Point<float>(0.55, -0.32),
                                                                  Point<float>(-0.53, -0.32),
                                                                  Point<float>(-0.53, 0.32));
};

/**
 * interaction_social_forces computes the social forces associated with an interaction between the robot and zero or
 * more objects.
 *
 * \param    interaction     Interaction between the robot and objects
 * \param    params          Parameters controlling the social forces
 * \return   Measured social forces between the robot and other objects.
 */
social_forces_t interaction_social_forces(const interaction_t& interaction, const social_forces_params_t& params);

/**
 * trajectory_social_forces computes the social forces on the robot along its entire trajectory. One instance of
 * social_forces_t is computed for each motion_state_t in the provided log.
 *
 * \param    interactions    Interactions along the trajectory that  the robot has with other objects
 * \param    params          Parameters controlling the behavior of the social forces computation
 * \return   The social forces for every interaction along the robot's trajectory in the log.
 * \post ret.size() == interactions.size()
 */
std::vector<social_forces_t> trajectory_social_forces(const std::vector<interaction_t>& interactions,
                                                      const social_forces_params_t& params);

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_EVALUATION_SOCIAL_FORCE_H
