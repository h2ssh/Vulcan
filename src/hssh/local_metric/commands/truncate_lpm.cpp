/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     truncate_lpm.h
* \author   Collin Johnson
* 
* Declaration of TruncateLpmCommand.
*/

#include "hssh/local_metric/commands/truncate_lpm.h"
#include "hssh/metrical/mapping/mapper.h"

namespace vulcan
{
namespace hssh
{

TruncateLpmCommand::TruncateLpmCommand(const std::string& source, const math::Rectangle<float>& boundary)
: LocalMetricCommand(source)
, boundary_(boundary)
{
}
    

void TruncateLpmCommand::issue(const metric_slam_data_t& data, 
                               Localizer&   localizer, 
                               Mapper&                   mapper, 
                               MetricRelocalizer&        relocalizer) const
{
    mapper.truncateMap(boundary_);
}


void TruncateLpmCommand::print(std::ostream& out) const
{
    out << "TruncateLpmCommand from " << source() << ": Boundary:" << boundary_;
}

}
}