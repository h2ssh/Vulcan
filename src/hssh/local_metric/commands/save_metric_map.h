/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     save_metric_map.h
* \author   Collin Johnson
* 
* Declaration of SaveMetricMapCommand.
*/

#ifndef HSSH_LOCAL_METRIC_COMMANDS_SAVE_METRIC_MAP_H
#define HSSH_LOCAL_METRIC_COMMANDS_SAVE_METRIC_MAP_H

#include <hssh/local_metric/command.h>

namespace vulcan
{
namespace hssh
{

/**
* SaveMetricMapCommand tells the local_metric_hssh module to save the current LPM to the specified file. There is
* not currently a way to tell if saving the map was successful or not beyond checking manually if the file was created.
* However, as long as permissions are okay and the file can be opened, then saving should be successful. Failure to
* initially create the file is the only reason saving should fail.
*/
class SaveMetricMapCommand : public LocalMetricCommand
{
public:
    
    SaveMetricMapCommand(const std::string& source, const std::string& filename);
    
    void issue(const metric_slam_data_t& data, 
               Localizer& localizer, 
               Mapper& mapper, 
               MetricRelocalizer& relocalizer) const override;
    void print(std::ostream& out) const override;

private:

    std::string filename_;

    // Serialization support
    SaveMetricMapCommand(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<LocalMetricCommand>(this),
            filename_);
    }
};

}
}

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::SaveMetricMapCommand)

#endif // HSSH_LOCAL_METRIC_COMMANDS_SAVE_METRIC_MAP_H
