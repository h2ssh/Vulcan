/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     relocalize_in_lpm.h
* \author   Collin Johnson
* 
* Declaration of RelocalizeInLpmCommand.
*/

#ifndef HSSH_LOCAL_METRIC_COMMANDS_RELOCALIZE_IN_LPM_H
#define HSSH_LOCAL_METRIC_COMMANDS_RELOCALIZE_IN_LPM_H

#include "hssh/local_metric/command.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/metrical/relocalization/filter_initializer.h"
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
* RelocalizeInLpmCommand issues a command to relocalize in the specified LPM using the specified filter initializer.
*/
class RelocalizeInLpmCommand : public LocalMetricCommand
{
public:
    
    /**
    * Constructor for RelocalizeInLpmCommand.
    * 
    * \param    source          Source module of the command
    * \param    map             Map in which to relocalize 
    * \param    initializer     Filter initializer to use for the relocalization
    */
    RelocalizeInLpmCommand(const std::string&                 source, 
                           const LocalPerceptualMap&          map,
                           std::shared_ptr<FilterInitializer> initializer);
    
    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data,
               Localizer&   localizer, 
               Mapper&                   mapper, 
               MetricRelocalizer&        relocalizer) const override;
    void print(std::ostream& out) const override;

private:

    LocalPerceptualMap map_;    
    std::shared_ptr<FilterInitializer> initializer_;
    
    // Serialization support
    RelocalizeInLpmCommand(void) { }
    
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<LocalMetricCommand>(this),
            map_,
            initializer_);
    }
    
};

}
}

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::RelocalizeInLpmCommand)

#endif // HSSH_LOCAL_METRIC_COMMANDS_RELOCALIZE_IN_LPM_H
