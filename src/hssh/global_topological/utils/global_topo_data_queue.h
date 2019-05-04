/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_data_queue.h
* \author   Collin Johnson
*
* Declaration of GlobalTopoDataQueue.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_UTILS_GLOBAL_TOPO_DATA_QUEUE_H
#define HSSH_GLOBAL_TOPOLOGICAL_UTILS_GLOBAL_TOPO_DATA_QUEUE_H

#include <hssh/global_topological/command.h>
#include <hssh/local_topological/event.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_metric/pose.h>
#include <utils/locked_double_buffer.h>
#include <utils/mutex.h>
#include <utils/condition_variable.h>
#include <deque>

namespace vulcan
{
namespace hssh
{

/**
* GlobalTopoDataQueue organizes and controls the flow of data into the global_topo module.
* To use the queue, simply call the waitForData() method. After the method returns, the
* most recent data can be accessed using the getCurrentData().
*/
class GlobalTopoDataQueue
{
public:

    /**
    * Constructor for GlobalTopoDataQueue.
    */
    GlobalTopoDataQueue(void);
    
    /**
    * waitForData is a blocking method that will wait until data necessary for GlobalTopo
    * arrives.
    *
    * \return   True if data was received. False if the wait timed out.
    */
    bool waitForData(void);

    /**
    * hasNewMessage checks if a new message has arrived.
    */
    bool hasNewCommand(void) const;

    /**
    * numCommands retrieves the current number of commands in the queue.
    */
    int numCommands(void) const;

    /**
    * popMessage retrieves the message at the front of the message queue.
    *
    * \return   Oldest received message if there is one. nullptr if no messages have been received.
    */
    GlobalTopoCommandPtr popCommand(void);

    /**
    * hasNewEvent checks if a new area event has occurred.
    */
    bool hasNewEvent(void) const;
    
    /**
    * numEvents retrieves the current number of events in the queue.
    */
    int numEvents(void) const;
    
    /**
    * popEvent retrieves the event at the front of the event queue.
    *
    * \return   Oldest received event if there is one. nullptr if no events have been received.
    */
    LocalAreaEventPtr popEvent(void);

    /**
    * popPose retrieves the most recent robot pose.
    */
    LocalPose popPose(void);
    
    /**
    * hasNewPose checks if a new pose is available.
    */
    bool hasNewPose(void) const;
    
    /**
    * popLocalMap retrieves the most recent LocalTopoMap. If hasLocalMap() == false, then the results of this function
    * are unpredictable.
    */
    LocalTopoMap popLocalMap(void);
    
    /**
    * hasLocalMap checks if there's a local map available.
    */
    bool hasLocalMap(void);
    
    // Data handlers
    void handleData(const LocalAreaEventVec& events, const std::string& channel);
    void handleData(const GlobalTopoCommandPtr& command, const std::string& channel);
    void handleData(const LocalPose& pose, const std::string& channel);
    void handleData(const LocalTopoMap& localMap, const std::string& channel);

private:

    std::deque<LocalAreaEventPtr> events_;
    std::deque<GlobalTopoCommandPtr> commands_;
    utils::LockedDoubleBuffer<LocalPose> latestPose_;
    utils::LockedDoubleBuffer<LocalTopoMap> latestLocalMap_;

    std::atomic<bool> haveLocalMap_;
    mutable utils::Mutex     dataLock_;
    utils::ConditionVariable dataTrigger_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_UTILS_GLOBAL_TOPO_DATA_QUEUE_H
