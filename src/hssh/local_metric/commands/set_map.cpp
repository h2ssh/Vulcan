/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     set_map.cpp
* \author   Collin Johnson
* 
* Definition of SetMapCommand.
*/

#include "hssh/local_metric/commands/set_map.h"
#include "hssh/metrical/mapping/mapper.h"

namespace vulcan 
{
namespace hssh
{
    
SetMapCommand::SetMapCommand(LocalPerceptualMap map, const std::string& source)
: LocalMetricCommand(source)
, lpm_(map)
{
}
    
    
void SetMapCommand::issue(const metric_slam_data_t& data, 
                          Localizer& localizer, 
                          Mapper& mapper, 
                          MetricRelocalizer& relocalizer) const
{
    mapper.setMap(lpm_);
}


void SetMapCommand::print(std::ostream& out) const
{
    out << "INFO: SetMapCommand: Setting map for local_metric_hssh from source: " << source() << '\n';
}
    
}
}
