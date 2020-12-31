/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     relocalize_in_lpm.cpp
 * \author   Collin Johnson
 *
 * Definition of RelocalizeInLpmCommand.
 */

#include "hssh/local_metric/commands/relocalize_in_lpm.h"
#include "hssh/metrical/relocalization/metric_relocalizer.h"

namespace vulcan
{
namespace hssh
{

RelocalizeInLpmCommand::RelocalizeInLpmCommand(const std::string& source,
                                               const LocalPerceptualMap& map,
                                               std::shared_ptr<FilterInitializer> initializer)
: LocalMetricCommand(source)
, map_(map)
, initializer_(std::move(initializer))
{
    assert(initializer_);
}


void RelocalizeInLpmCommand::issue(const metric_slam_data_t& data,
                                   Localizer& localizer,
                                   Mapper& mapper,
                                   MetricRelocalizer& relocalizer) const
{
    relocalizer.startRelocalization(data, map_, *initializer_);
}


void RelocalizeInLpmCommand::print(std::ostream& out) const
{
    out << "RelocalizeInLpmCommand from " << source();
}

}   // namespace hssh
}   // namespace vulcan
