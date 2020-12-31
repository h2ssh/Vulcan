/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     command.h
* \author   Collin Johnson
* 
* Definition of LocalMetricCommand abstract base class.
*/

#ifndef HSSH_LOCAL_METRIC_COMMAND_H
#define HSSH_LOCAL_METRIC_COMMAND_H

#include "system/message_traits.h"
#include <iostream>
#include <string>
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>

namespace vulcan
{
namespace hssh
{
    
class  Localizer;
class  Mapper;
class  MetricRelocalizer;
struct metric_slam_data_t;

/**
* LocalMetricCommand is the abstract base class for commands to control the functionality of the Local Metric HSSH
* module. Commands can be issued to the following parts of the module:
* 
*   - Localizer   : handles the localization half of SLAM
*   - Mapper      : handles the mapping half of SLAM
*   - Relocalizer : allows the robot to relocalize itself in a previously existing map
*/
class LocalMetricCommand
{
public:
    
    /**
    * Constructor for LocalMetricCommand.
    * 
    * \param    source          Source of the command (for informational purposes)
    */
    explicit LocalMetricCommand(const std::string& source = "")
    : source_(source)
    {
    }
    
    /**
    * issue issues the command to the module. The components that can be controlled are all provided and the command can
    * interact with whichever components it needs to to complete its task.
    * 
    * \param    data                Current data from sensors
    * \param    localizer           Localizer instance for the module
    * \param    mapper              Mapper instance for the module
    * \param    relocalizer         Relocalizer instance for the module
    */
    virtual void issue(const metric_slam_data_t& data, 
                       Localizer&                localizer, 
                       Mapper&                   mapper, 
                       MetricRelocalizer&        relocalizer) const = 0;
    
    /**
    * print prints an informational message about the command. The print method is called from operator<<.
    * 
    * \param    out         Stream to write information to.
    */
    virtual void print(std::ostream& out) const = 0;
    
    /**
    * source retrieves the source module of the command.
    */
    std::string source(void) const { return source_; }

private:

    std::string source_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( source_);
    }
};


// Output operator for LocalMetricCommand
inline std::ostream& operator<<(std::ostream& out, const LocalMetricCommand& command)
{
    command.print(out);
    return out;
}

}
}

DEFINE_SYSTEM_MESSAGE(std::shared_ptr<hssh::LocalMetricCommand>, ("HSSH_LOCAL_METRIC_COMMAND"))

#endif // HSSH_LOCAL_METRIC_COMMAND_H
