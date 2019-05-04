/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     toggle_glass_mapping.cpp
* \author   Collin Johnson
*
* Definition of ToggleGlassMapping.
*/

#include <hssh/local_metric/commands/toggle_glass_mapping.h>
#include <hssh/metrical/mapping/mapper.h>

namespace vulcan
{
namespace hssh
{

ToggleGlassMapping::ToggleGlassMapping(bool shouldMap, const std::string& source)
: LocalMetricCommand(source)
, shouldMap_(shouldMap)
{

}

// LocalMetricCommand interface
void ToggleGlassMapping::issue(const metric_slam_data_t& data,
                               Localizer& localizer,
                               Mapper& mapper,
                               MetricRelocalizer& relocalizer) const
{
    mapper.shouldBuildGlassMap(shouldMap_);
}


void ToggleGlassMapping::print(std::ostream& out) const
{
    out << "INFO: ToggleGlassMapping: Turning glass mapping " << (shouldMap_ ? "on" : "off")
        << " via " << source() << '\n';
}

}
}
