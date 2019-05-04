/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     heartbeat.h
* \author   Collin Johnson
* 
* Declaration of Heartbeat sent out by a Module after every update.
*/

#ifndef SYSTEM_HEARTBEAT_H
#define SYSTEM_HEARTBEAT_H

#include <cereal/access.hpp>

namespace vulcan
{
namespace system
{

/**
* Heartbeat
*/
class Heartbeat
{
public:
    
private:
    
    int64_t timestamp_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        // TODO:
    }
};

}
}

#endif // SYSTEM_HEARTBEAT_H
