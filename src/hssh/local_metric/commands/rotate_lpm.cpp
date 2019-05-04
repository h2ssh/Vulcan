/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rotate_lpm.cpp
* \author   Collin Johnson
* 
* Definition of RotateLpmCommand.
*/

#include <hssh/local_metric/commands/rotate_lpm.h>
#include <hssh/metrical/mapping/mapper.h>

namespace vulcan
{
namespace hssh
{

RotateLpmCommand::RotateLpmCommand(const std::string& source, float radians)
: LocalMetricCommand(source)
, radians_(radians)
{
}


void RotateLpmCommand::issue(const metric_slam_data_t& data, 
                             Localizer&   localizer, 
                             Mapper&                   mapper, 
                             MetricRelocalizer&        relocalizer) const
{
    mapper.rotateMap(radians_);
}
            
            
void RotateLpmCommand::print(std::ostream& out) const
{
    out << "RotateLpmCommand from " << source() << " Radians:" << radians_;
}

}
}