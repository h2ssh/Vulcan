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
* Declaration of MotionControllerDirector for doing the overall processing of the data.
*/

#ifndef MPEPC_MOTION_CONTROLLER_DIRECTOR_H
#define MPEPC_MOTION_CONTROLLER_DIRECTOR_H

#include "system/director.h"
#include "utils/condition_variable.h"
#include "utils/mutex.h"

#include "mpepc/motion_controller/controller/controller_chain.h"
#include "mpepc/motion_controller/params.h"
#include "core/motion_state.h"
#include "robot/commands.h"

namespace vulcan
{

namespace mpepc
{

class  MotionControllerTask;
struct motion_controller_command_message_t;

/**
* MotionControllerDirector handles distribution and processing of data in the path follower.
* The director receives data by implementing the MotionControllerInputConsumer interface. This
* data is then passed to the appropriate pieces of the path follower.
*
* The update rate of the Director hinges on receipt of pose data, as that data is needed
* for generating the feedback necessary for calculating the motion commands. If no pose
* data is arriving, then no new motion commands will be issued.
*
* After creation, the Director accepts any output consumers that wish to receive data.
* Then, the run() method should be called. This method will never return and puts the
* controller into the data processing mode.
*/
class MotionControllerDirector : public system::Director
{
public:

    /**
    * Constructor for MotionControllerDirector.
    */
    MotionControllerDirector(const utils::CommandLine& commandLine, const utils::ConfigFile& config);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& producer) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& transmitter) override;
    void shutdown(system::ModuleCommunicator& transmitter) override;

    // Data handlers
    void handleData(const std::shared_ptr<MotionControllerTask>& task,    const std::string& channel);
    void handleData(const motion_state_t&                 state,   const std::string& channel);
    void handleData(const motion_controller_command_message_t&   message, const std::string& channel);

private:

    void processTaskQueue      (void);
    void calculateNewCommand   (void);
    bool shouldCalculateCommand(void);

    motion_controller_params_t params_;

    motion_state_t currentState_;
    bool                  haveState_;

    robot::motion_command_t currentCommand_;
    bool    isPaused_;
    bool    isCancelled_;
    int64_t pausedTimestamp_;
    int64_t cancelledTimestamp_;
    int64_t previousOutputTimeUs_;

    ControllerChain controllers_;
    std::vector<std::shared_ptr<MotionControllerTask>> taskQueue_;

    utils::Mutex             dataLock_;
    utils::ConditionVariable dataTrigger_;

};

} // mpepc
} // vulcan

#endif // MPEPC_MOTION_CONTROLLER_DIRECTOR_H
