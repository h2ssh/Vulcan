/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     save_metric_map.cpp
 * \author   Collin Johnson
 *
 * Definition of SaveMetricMapCommand.
 */

#include "hssh/local_metric/commands/save_metric_map.h"
#include "hssh/metrical/mapping/mapper.h"
#include "utils/serialized_file_io.h"

namespace vulcan
{
namespace hssh
{

SaveMetricMapCommand::SaveMetricMapCommand(const std::string& source, const std::string& filename)
: LocalMetricCommand(source)
, filename_(filename)
{
}


void SaveMetricMapCommand::issue(const metric_slam_data_t& data,
                                 Localizer& localizer,
                                 Mapper& mapper,
                                 MetricRelocalizer& relocalizer) const
{
    if (utils::save_serializable_to_file(filename_, mapper.getLPM())) {
        std::cout << "INFO: SaveMetricMapCommand: Saved LPM to " << filename_ << '\n';
    } else {
        std::cout << "ERROR: SaveMetricMapCommand: Failed to save LPM to " << filename_ << '\n';
    }
}


void SaveMetricMapCommand::print(std::ostream& out) const
{
    out << "SaveMetricMapCommand from " << source() << " Saving to: " << filename_;
}

}   // namespace hssh
}   // namespace vulcan
