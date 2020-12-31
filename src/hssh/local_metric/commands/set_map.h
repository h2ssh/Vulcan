/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     set_map.h
* \author   Collin Johnson
* 
* Declaration of SetMapCommand.
*/

#ifndef HSSH_LOCAL_METRIC_COMMANDS_SET_MAP_H
#define HSSH_LOCAL_METRIC_COMMANDS_SET_MAP_H

#include "hssh/local_metric/command.h"
#include "hssh/local_metric/lpm.h"

namespace vulcan
{
namespace hssh
{

/**
* SetMapCommand sets the map being used by local_metric_hssh without updating the relocalization.
* 
* This command should be used whenever manual annotations are made to an LPM while the robot is currently building the
* map. The main flow is:
* 
*   - Capture an LPM via the MapEditor
*   - Annotate it with the tools
*   - Send it back to local_metric_hssh with the updated annotations
*/
class SetMapCommand : public LocalMetricCommand
{
public:
    
    /**
    * Constructor for SetMapCommand.
    * 
    * \param    map         Map to be set
    * \param    source      Source of the command (for informational purposes, optional)
    */
    explicit SetMapCommand(LocalPerceptualMap map, const std::string& source = "");
    
    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data, 
               Localizer& localizer, 
               Mapper& mapper, 
               MetricRelocalizer& relocalizer) const override;
    void print(std::ostream& out) const override;

private:

    LocalPerceptualMap lpm_;
    
    // For serialization support
    SetMapCommand(void) { }
    
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<LocalMetricCommand>(this),
            lpm_);
    }
};

}
}

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::SetMapCommand)

#endif // HSSH_LOCAL_METRIC_COMMANDS_SET_MAP_H
