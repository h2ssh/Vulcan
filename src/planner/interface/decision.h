/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision.h
 * \author   Collin Johnson
 *
 * Declaration of Decision.
 */

#ifndef PLANNER_INTERFACE_DECISION_H
#define PLANNER_INTERFACE_DECISION_H

#include "core/point.h"
#include "hssh/local_topological/area.h"
#include "mpepc/metric_planner/task/task.h"

namespace vulcan
{
struct pose_t;
namespace planner
{

/**
 * DecisionDirection defines the direction the action takes the robot relative to its location. The relative
 * directions can be selected using the arrow keys on the keyboard, for example. The absolute directions require a more
 * sophisticated selection mechanism that doesn't map to a keyboard as easily.
 */
enum class DecisionDirection
{
    left,
    right,
    forward,
    backward,
    absolute,
};

/**
 * Decision describes an action the robot can take within its current area. The action is an abstract description
 * of a specific navigation task to be executed by the robot.
 *
 *
 */
class Decision
{
public:
    /**
     * Constructor for Decision.
     *
     * \param    direction       Direction the action takes the robot
     * \param    areaType        Type of area the action leads to
     * \param    position        Position of the decision
     * \param    orientation     Orientation of the decision
     * \param    isAbsolute      Flag indicating if the position is relative to the robot or absolute.
     *                           A relative position will be associated with a travel action. Transitions are all
     *                           absolute because they are associated with a particular gateway.
     */
    Decision(DecisionDirection direction,
             hssh::AreaType areaType,
             Point<double> position,
             double orientation,
             bool isAbsolute);

    /**
     * direction retrieves the direction in which the action carries the robot relative to its current topological
     * position.
     */
    DecisionDirection direction(void) const { return direction_; }

    /**
     * areaType retrieves the type of area the action will lead the robot to.
     */
    hssh::AreaType areaType(void) const { return type_; }

    /**
     * position retrieves the position of the action within the current map. The position provides an approximate
     * location of where the task to be performed will move the robot.
     *
     * \return   Position of the action in the current map reference frame.
     */
    Point<double> position(void) const { return position_; }

    /**
     * orientation retrieves the orientation of the action within the current map. The orientation provides an
     * approximate heading along which the robot will be moving if it executes the action.
     *
     * \return   Orientation of the action in the current map reference frame.
     */
    double orientation(void) const { return orientation_; }

    /**
     * isAbsolute checks if the decision is relative to the robot's position, i.e. it will be changing over time as the
     * robot explores, or if it is absolute and corresponds to an unchanging state.
     */
    bool isAbsolute(void) const { return isAbsolute_; }

private:
    DecisionDirection direction_;
    hssh::AreaType type_;
    Point<double> position_;
    double orientation_;
    bool isAbsolute_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_INTERFACE_DECISION_H
