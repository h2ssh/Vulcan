/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     debug_communicator.h
* \author   Collin Johnson
* 
* Definition of DebugCommunicator adapter class.
*/

#ifndef SYSTEM_DEBUG_COMMUNICATOR_H
#define SYSTEM_DEBUG_COMMUNICATOR_H

#include <system/message_traits.h>
#include <system/module_communicator.h>
#include <type_traits>

namespace vulcan
{
namespace system
{

/**
* DebugCommunicator is an Adapter class that provides limited access to the communicator facilities of a 
* ModuleCommunicator. It enforces that only Debug and Plot data are sent. Any other data that attempts to be sent
* will result in a static_assert compiler error.
* 
* The DebugCommunicator should be passed to processing classes by a Director. This ensures that system messages flow
* only from the Director, which enforces the inputs and outputs, and not the implementation classes.
*/
class DebugCommunicator
{
public:
    
    /**
    * Constructor for DebugCommunicator.
    * 
    * \param    communicator        ModuleCommunicator instance to wrap
    */
    DebugCommunicator(ModuleCommunicator& communicator)
    : communicator_(communicator)
    {
    }
    
    /**
    * sendDebug sends a debug message.
    * 
    * \param    msg         Message to be sent
    * 
    * Concept:
    * 
    *   - T is a DebugMessage
    */
    template <typename T>
    void sendDebug(const T& msg)
    {
        static_assert(std::is_same<typename message_traits<T>::type, debug_message_tag>::value, 
                      "sendDebug: Can only send DebugMessages");
        communicator_.sendMessage(msg);
    }
    
//     /**
//     * sendPlot
//     */
//     void sendPlot(const ui::PlotData& plot)
//     {
//         communicator_.sendMessage(plot);
//     }

private:

    ModuleCommunicator& communicator_;
};

}
}

#endif // SYSTEM_DEBUG_COMMUNICATOR_H
