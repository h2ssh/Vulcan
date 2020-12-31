/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_data_queue.cpp
* \author   Collin Johnson
*
* Definition of GlobalTopoDataQueue.
*/

#include "hssh/global_topological/utils/global_topo_data_queue.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace hssh
{

GlobalTopoDataQueue::GlobalTopoDataQueue(void)
: haveLocalMap_(false)
, dataTrigger_(false)
{
}


bool GlobalTopoDataQueue::waitForData(void)
{
    // timedWait == 0 if success
    bool haveData = !dataTrigger_.timedWait(100);
    
    // If data arrived, turn off the trigger for the next update
    if(haveData)
    {
        dataTrigger_.setPredicate(false);
    }
    
    return haveData;
}


bool GlobalTopoDataQueue::hasNewCommand(void) const
{
    utils::AutoMutex autoLock(dataLock_);
    return !commands_.empty();
}


int GlobalTopoDataQueue::numCommands(void) const
{
    utils::AutoMutex autoLock(dataLock_);
    return commands_.size();
}


GlobalTopoCommandPtr GlobalTopoDataQueue::popCommand(void)
{
    utils::AutoMutex autoLock(dataLock_);

    if(commands_.empty())
    {
        return nullptr;
    }
    else
    {
        auto message = std::move(commands_.front());
        commands_.pop_front();
        return message;
    }
}


bool GlobalTopoDataQueue::hasNewEvent(void) const
{
    utils::AutoMutex autoLock(dataLock_);
    return !events_.empty();// && haveLocalMap_;
}


int GlobalTopoDataQueue::numEvents(void) const
{
    utils::AutoMutex autoLock(dataLock_);
    return events_.size();
}


LocalAreaEventPtr GlobalTopoDataQueue::popEvent(void)
{
    utils::AutoMutex autoLock(dataLock_);

    if(events_.empty())
    {
        return nullptr;
    }
    else
    {
        auto event = std::move(events_.front());
        events_.pop_front();
        return event;
    }
}


LocalPose GlobalTopoDataQueue::popPose(void)
{
    // If there's a new pose, then use it.
    if(latestPose_.hasData())
    {
        latestPose_.swapBuffers();
    }
    
    return latestPose_;
}


bool GlobalTopoDataQueue::hasNewPose(void) const
{
    return latestPose_.hasData();
}


LocalTopoMap GlobalTopoDataQueue::popLocalMap(void)
{
    // If there's a new map, then use it.
    if(latestLocalMap_.hasData())
    {
        latestLocalMap_.swapBuffers();
    }
    
    return latestLocalMap_;
}


bool GlobalTopoDataQueue::hasLocalMap(void)
{
    return haveLocalMap_;
}


void GlobalTopoDataQueue::handleData(const LocalAreaEventVec& events, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    // Copy each event into the event queue
    std::transform(events.begin(), events.end(), std::back_inserter(events_), [](const LocalAreaEventPtr& e) {
        return e->clone();
    });

    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void GlobalTopoDataQueue::handleData(const LocalPose& pose, const std::string& channel)
{
    latestPose_ = pose;
    
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void GlobalTopoDataQueue::handleData(const GlobalTopoCommandPtr& command, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    commands_.push_back(command);

    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void GlobalTopoDataQueue::handleData(const LocalTopoMap& localMap, const std::string& channel)
{
    latestLocalMap_ = localMap;
    haveLocalMap_ = true;
}

} // namespace hssh
} // namespace vulcan
