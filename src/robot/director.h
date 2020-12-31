/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     director.h
 * \author   Collin Johnson
 *
 * Declaration of RobotInterfaceDirector.
 */

#ifndef ROBOT_DIRECTOR_H
#define ROBOT_DIRECTOR_H

#include "core/laser_scan.h"
#include "robot/motion_command_filter.h"
#include "system/director.h"
#include "utils/mutex.h"
#include "utils/thread.h"
#include <deque>
#include <vector>

namespace vulcan
{
namespace robot
{

class MotionChecker;
class Wheelchair;
struct robot_controller_params_t;

/**
 * RobotInterfaceDirector is responsible for providing the varying pieces of the wheelchair controller
 * module with the data they need to accomplish their task.
 */
class RobotInterfaceDirector : public system::TimeTriggeredDirector
{
public:
    /**
     * Constructor for RobotInterfaceDirector.
     *
     * \param    params          Parameters defining the execution for this instance of the director
     */
    RobotInterfaceDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    /**
     * Destructor for RobotInterfaceDirector.
     */
    virtual ~RobotInterfaceDirector(void);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Handlers for incoming data to be distributed to the various modules
    virtual void handleData(const motion_command_t& command, const std::string& channel);
    virtual void handleData(const polar_laser_scan_t& scan, const std::string& channel);

private:
    struct laser_data_t
    {
        std::string channel;
        polar_laser_scan_t scan;
    };

    robot_controller_params_t params;

    MotionCommandFilter filter;
    std::unique_ptr<MotionChecker> checker;
    std::unique_ptr<Wheelchair> wheelchair;

    std::deque<motion_command_t> commandQueue;
    std::vector<laser_data_t> laserScans;
    std::vector<Point<float>> scanPoints;

    cartesian_laser_scan_t cartesianScan;

    motion_command_t activeCommand;

    int64_t updatePeriodUs;
    int64_t previousOutputTimeUs;

    utils::Thread wheelchairThread;
    utils::Mutex dataLock;

    void startWheelchairThread(void);
    motion_command_t selectMotionCommand(void);
    void convertLaserDataToScanPoints(void);
};

}   // namespace robot
}   // namespace vulcan

#endif   // ROBOT_DIRECTOR_H
