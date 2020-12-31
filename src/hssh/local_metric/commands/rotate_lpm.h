/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rotate_lpm.h
* \author   Collin Johnson
* 
* Declaration of RotateLpmCommand.
*/

#ifndef HSSH_LOCAL_METRIC_COMMANDS_ROTATE_LPM_H
#define HSSH_LOCAL_METRIC_COMMANDS_ROTATE_LPM_H

#include "hssh/local_metric/command.h"
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
* RotateLpmCommand tells the Mapper to rotate the map by the specified number of radians.
*/
class RotateLpmCommand : public LocalMetricCommand
{
public:
    
    /**
    * Construtor for RotateLpmCommand.
    * 
    * \param    source          Source module for the command
    * \param    radians         Radians to rotate the map
    */
    RotateLpmCommand(const std::string& source, float radians);
    
    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data, 
               Localizer&   localizer, 
               Mapper&                   mapper, 
               MetricRelocalizer&        relocalizer) const override;
    void print(std::ostream& out) const override;

private:

    float radians_;
    
    // Serialization support
    RotateLpmCommand(void) { }
    
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<LocalMetricCommand>(this),
            radians_);
    }
};

}
}

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::RotateLpmCommand)

#endif // HSSH_LOCAL_METRIC_COMMANDS_ROTATE_LPM_H
