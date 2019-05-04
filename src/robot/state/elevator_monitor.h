/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     elevator_monitor.h
* \author   Collin Johnson
*
* Declaration of ElevatorMonitor.
*/

#ifndef STATE_MONITOR_ELEVATOR_MONITOR_H
#define STATE_MONITOR_ELEVATOR_MONITOR_H

#include <robot/state/state_estimator.h>
#include <robot/state/params.h>
#include <robot/elevator.h>
#include <utils/mutex.h>
#include <atomic>
#include <deque>

namespace vulcan
{
namespace robot
{

const std::string kElevatorMonitorType("elevator");

struct elevator_monitor_params_t
{
    int64_t motionCheckIntervalUs;
    
    double xVarianceWhenStopped;
    double yVarianceWhenStopped;
    double zVarianceWhenStopped;
    
    double  minElevatorAccelMagnitude;
    int64_t minElevatorAccelDurationUs;
    
    int64_t durationOfQueueUs;
    
    elevator_monitor_params_t(const utils::ConfigFile& config);
};

/**
* ElevatorMonitor
*/
class ElevatorMonitor : public StateEstimator
{
public:

    /**
    * Constructor for ElevatorMonitor.
    *
    * \param    params          Parameters for controlling the elevator state detection
    */
    ElevatorMonitor(const elevator_monitor_params_t& params);

    // StateMonitor interface
    virtual void initialize(system::ModuleCommunicator& communicator);
    virtual void estimate(system::ModuleCommunicator& transmitter);
    
    // Data handlers
    void handleData(const imu_data_t& imu, const std::string& channel);

private:

    enum detection_state_t
    {
        ROBOT_MOVING,
        ROBOT_STOPPED,
        ELEVATOR_MOVING
    };

    struct imu_axis_t
    {
        std::deque<double> accels;
        double mean;
        double variance;
    };

    typedef std::pair<int64_t, double> accel_t;

    detection_state_t detectionState;

    elevator_t elevator;
    bool       haveNewElevatorState;
    double     gravityEstimate;
    int64_t    elevatorStartTime;
    int64_t    elevatorStopTime;
    int64_t    lastIntegrationTime;
    double     elevatorDistance;
    double     elevatorVelocity;
    double     initialElevatorAccel;
    bool       hasElevatorSlowed;

    // INVARIANT: While elevator is moving startTime > stopTime
    std::atomic<int>    numMeasurements;
    std::deque<int64_t> timestamps;
    std::deque<int64_t> timeDiffs;
    std::deque<double>  diffFromGravity;
    imu_axis_t          xAxis;
    imu_axis_t          yAxis;
    imu_axis_t          zAxis;

    elevator_monitor_params_t params;
    utils::Mutex              dataLock;
    
    void enqueueData(const imu_data_t& data);
    void removeOldData(void);
    void calculateIMUStatistics(void);
    int  findStartIndexOfInterval(int64_t duration);
    int  findIndexWithTime(int64_t time);
    void calculateAxisStatistics(int startIndex, imu_axis_t& axis);
    void estimateGravity(std::size_t startIndex);
    
    void    determineElevatorState(void);
    bool    isRobotMoving(void) const;
    bool    isElevatorStarting(void);
    bool    isElevatorStopped(void);
    accel_t findElevatorAccel(void);
    
    void initializeElevatorMotion(void);
    void processElevatorMoving(void);
    void integrateNewData(void);
    void updateElevatorState(void);
};

}
}

#endif // STATE_MONITOR_ELEVATOR_MONITOR_H
