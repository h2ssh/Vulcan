/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     system_communicator.h
* \author   Collin Johnson
*
* Definition of SystemCommunicator.
*/

#ifndef SYSTEM_SYSTEM_COMMUNICATOR_H
#define SYSTEM_SYSTEM_COMMUNICATOR_H

#include <system/message_traits.h>
#include <system/module_communicator.h>
#include <type_traits>

namespace vulcan
{
namespace system
{

/**
* SystemCommunicator is an Adapter class that provides limited access to the communicator facilities of a
* ModuleCommunicator. It enforces that only System messages are sent. Any other data that attempts to be sent
* will result in a static_assert compiler error.
*
* The SystemCommunicator should be passed to processing classes by a Director. This ensures that system messages flow
* only from classes specified by the Director, keeping implementation classes from inadventantly sending messages that
* aren't part of the defined input/output.
*/
class SystemCommunicator
{
public:

    /**
    * Constructor for SystemCommunicator.
    *
    * \param    communicator        ModuleCommunicator instance to wrap
    */
    SystemCommunicator(ModuleCommunicator& communicator)
    : communicator_(communicator)
    {
    }

    /**
    * sendSystem sends a system message.
    *
    * \param    msg         Message to be sent
    *
    * Concept:
    *
    *   - T is a SystemMessage
    */
    template <typename T>
    void sendSystem(const T& msg)
    {
        static_assert(std::is_same<typename message_traits<T>::type, system_message_tag>::value,
                        "sendSystem: Can only send SystemMessages");
        communicator_.sendMessage(msg);
    }


private:

    ModuleCommunicator& communicator_;
};

}
}

#endif // SYSTEM_SYSTEM_COMMUNICATOR_H
