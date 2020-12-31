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
* Definition of: 
* 
*   - The Director interface for modules.
*   - TimeTriggeredDirector for modules that run at a constant update rate
*   - DataTriggeredDirector for modules that can only run after some data arrives
*/

#ifndef SYSTEM_DIRECTOR_H
#define SYSTEM_DIRECTOR_H

#include "utils/timestamp.h"
#include <unistd.h>
#include <iostream>

namespace vulcan
{
namespace utils { class CommandLine; }
namespace utils { class ConfigFile;  }
namespace system
{
    
class ModuleCommunicator;

/**
* TriggerStatus defines the possible states available when waiting for an update trigger. Either the trigger has
* fired and an update is ready or it isn't.
*/
enum class TriggerStatus
{
    ready,          ///< An update can be run because the module has achieved the conditions needed to update
    not_ready,      ///< Keep waiting for the update condition to be occur
};

/**
* UpdateStatus defines the possible states available when running a module update.
*/
enum class UpdateStatus
{
    running,        ///< Module is running successfully
    fatal_error,    ///< Module has encountered a fatal error and should shutdown
    finished,       ///< Module has completed its job and should shutdown
};

/**
* Director is the interface for the component of a Module that organizes the actual computation.
* The basic computation flow for the Director is to wait for some event to fire, then do some computation,
* and transmit the results to other modules. This data flow is implemented as two virtual functions for implementations
* of the Director interface:
* 
*   - waitForTrigger() : pause until some event occurs -- a timer firing or data arriving
*   - runUpdate()      : process new data since the last update
* 
* Initialization and shutdown are handled via two methods:
* 
*   - subscribeToData() : provides a DataProducer for subscribing to desired messages and input types
*   - shutdown()        : called when it is time to stop a module -- save state, etc. in here
*/
class Director
{
public:

    virtual ~Director(void) { }

    /**
    * subscribeToData should subscribe to all necessary data for the module. Subscribing to necessary data is part
    * of the initialization process for the module. If additional subscriptions are needed during module operation,
    * they can be handled during a call to runUpdate().
    *
    * \param    communicator            Communicator instance used for the module
    */
    virtual void subscribeToData(ModuleCommunicator& communicator) = 0;
    
    /**
    * waitForTrigger has the module wait until data is available. The trigger is any condition necessary for an
    * update to occur.
    *
    * waitForTrigger shouldn't block indefinitely, as the module cannot exit properly if waitForTrigger never returns.
    * If data isn't available after some short amount of time, say 100ms, then return a not_available. At which
    * point, module control logic can run before then next call to waitForTrigger.
    * 
    * \return   Status of update availability for the module. Either an update is ready or it isn't.
    */
    virtual TriggerStatus waitForTrigger(void) = 0;
    
    /**
    * runUpdate runs a module update. The runUpdate method performs all major computation for a module. When called,
    * the trigger for the module has said an update should be performed. During runUpdate, the module should perform
    * its calculations and then send the defined outputs for the module.
    *
    * When an update finishes, it is either ready to keep running, ready to shutdown because all functionality has
    * been achieved, or has encountered a fatal error.
    *
    * \param    communicator            Communicator to use for sending out module outputs
    * \return   The status of the module, whether it should keep running or exit for one reason or another.
    */
    virtual UpdateStatus runUpdate(ModuleCommunicator& communicator) = 0;
    
    /**
    * shutdown sends what messages are needed for successful shutdown of the module. Behaviors occurring on shutdown
    * might include saving the current map or sending commands to make sure the robot has stopped.
    *
    * \param    communicator        Communicator instance used for talking with other modules
    */
    virtual void shutdown(ModuleCommunicator& communicator) = 0;
};


/**
* TimeTriggeredDirector is a Director that tries to maintain a consistent update period. The period
* is provided in the constructor and then the runUpdate method will be called every period milliseconds.
* The time-triggering only controls when the processing starts.
*/
class TimeTriggeredDirector : public Director
{
public:
    
    /**
    * Constructor for TimeTriggeredDirector.
    * 
    * \param    periodMs            Number of milliseconds each update is expected to take
    */
    TimeTriggeredDirector(int periodMs)
    : periodUs_(periodMs*1000)
    , updateStartTimeUs_(0)
    {
    }
    
    /**
    * Destructor for TimeTriggeredDirector.
    */
    virtual ~TimeTriggeredDirector(void) { }
    
    /**
    * waitForTrigger sleeps for the remaining time in the current update period. If the
    * period was missed, then it will immediately return so processing can try to catch up.
    */
    TriggerStatus waitForTrigger(void) override
    {
        int64_t timeSinceStartUs = utils::system_time_us() - updateStartTimeUs_;
        
        if(timeSinceStartUs < periodUs_)
        {
            usleep(periodUs_ - timeSinceStartUs);
        }
        else
        {
            std::cerr << "WARNING::TimeTriggeredDirector: Missed expected period: Was: "
                      << (timeSinceStartUs/1000) << "ms. Desired: " << (periodUs_/1000) << "ms.\n";
        }
        
        updateStartTimeUs_ = utils::system_time_us();
        
        // The timed trigger always succeeds.
        return TriggerStatus::ready;
    }
    
    // Subclasses need to implement:
    //  - subscribeToData
    //  - runUpdate
    //  - shutdown
    
private:
    
    int64_t periodUs_;
    int64_t updateStartTimeUs_;
};

}
}

#endif // SYSTEM_DIRECTOR_H
