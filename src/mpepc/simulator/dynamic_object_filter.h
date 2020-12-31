/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object_filter.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of DynamicObjectFilter.
*/

#ifndef MPEPC_SIMULATOR_DYNAMIC_OBJECT_FILTER_H
#define MPEPC_SIMULATOR_DYNAMIC_OBJECT_FILTER_H

#include "mpepc/simulator/params.h"
#include "mpepc/simulator/dynamic_object_trajectory.h"
#include "tracker/dynamic_object_visitor.h"

namespace vulcan
{
namespace tracker { class  DynamicObjectCollection; }
namespace mpepc
{

class ObstacleDistanceGrid;

/**
* DynamicObjectFilter filters dynamic objects estimated by the object tracking module. The objects that make it
* through the filter are those that should be considered for collisions during the trajectory planning phase.
*/
class DynamicObjectFilter : public tracker::DynamicObjectVisitor
{
public:

    /**
    * Constructor for DynamicObjectFilter.
    *
    * \param    params          Parameters controlling the filter behavior
    */
    DynamicObjectFilter(const dynamic_object_filter_params_t& params);

    /**
    * filterObjects applies the internal filters to convert DynamicObjectCollection into a
    * collection of dynamic_object_t for use internally with MPEPC.
    *
    * \param    objects         Objects to be filtered
    * \param    robotState      Location of the robot in the environment
    * \param    startTimeUs     Start time of the current planning iteration.
    * \return   Filtered collection of objects for use in MPEPC.
    */
    std::vector<dynamic_object_trajectory_t> filterObjects(const tracker::DynamicObjectCollection& objects,
                                                           const motion_state_t& robotState,
                                                           const ObstacleDistanceGrid& map,
                                                           int64_t startTimeUs);

private:

    dynamic_object_state_t initialObjectState_;    // tmp variable for use with the visitors

    dynamic_object_filter_params_t params_;


    // tracker::DynamicObjectVisitor interface
    void visitPerson(const tracker::Person& person) override;
    void visitRigid(const tracker::RigidObject& object) override;
    void visitUnclassified(const tracker::UnclassifiedObject& object) override;
    void visitPivotingObject(const tracker::PivotingObject& door) override;
    void visitSlidingObject(const tracker::SlidingObject& door) override;

    // Methods for filtering and extracting the dynamic objects
    /**
    * initializeSimulatorStates initialize the state veriables of all objects of interest for trajectory estimation.
    *
    * \param    trackedObjects  results from the laser tracker.
    * \param    robotState      the most recent state of the robot.
    * \param    map             obstacle cost map, which stores information about freesapce and minimum distance to the nearest static obstacle in the map
    * \param    startTimeUs     time at which to start the simulation in microseconds.
    * \param    timeStep        time interval between simulated points.
    */
    void initializeSimulatorStates(const tracker::DynamicObjectCollection& trackedObjects,
                                   const motion_state_t&            robotState,
                                   const ObstacleDistanceGrid&             map,
                                   int64_t                                 startTimeUs,
                                   float                                   timeStep,
                                   float                                   timeLength);

    // Pull the object state from the tracked object
    dynamic_object_state_t createObjectState(const tracker::DynamicObject& trackedObject);

    // Conditions for valid objects
    bool isNearRobot(const dynamic_object_state_t& objectState, const motion_state_t& robotState);
    bool isFarFromWalls(const dynamic_object_state_t& objectState, const ObstacleDistanceGrid& map);

    // Model potential robot-object interaction
    // NOTE: This is potentially dangerous method that lets the robot completely ignore object directly behind the robot at close range,
    //       but at least it allows us to perform demos with people closely following the robot from behind. We probably want to have more
    //       sophisticated method for handling object-robot interactions so that we do better estimation of object motion.
    dynamic_object_state_t slowdownObjectBehindRobot(const dynamic_object_state_t& objectState,
                                                     const motion_state_t& robotState);

    // Use inferred intentions to assign goal to the objects
    pose_t estimateGoal(const tracker::DynamicObject& trackedObject);
    Point<float> estimatePreferredVelocity(const dynamic_object_state_t& objectState,
                                                 const pose_t& objectGoal);

};

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_SIMULATOR_DYNAMIC_OBJECT_FILTER_H
