/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     set_slam_mode.cpp
* \author   Collin Johnson
* 
* Definition of SetSlamModeCommand.
*/

#include "hssh/local_metric/commands/set_slam_mode.h"
#include "hssh/metrical/mapping/mapper.h"

namespace vulcan
{
namespace hssh
{

void issue_slam_mode_command     (SlamMode      mode, Mapper& mapper);
void issue_mapping_mode_command  (MappingMode   mode, Mapper& mapper);
void issue_map_resolution_command(MapResolution res,  Mapper& mapper);
    
std::ostream& operator<<(std::ostream& out, SlamMode      mode);
std::ostream& operator<<(std::ostream& out, MappingMode   mode);
std::ostream& operator<<(std::ostream& out, MapResolution mode);


SetSlamModeCommand::SetSlamModeCommand(const std::string& source,
                                       SlamMode           slamMode,
                                       MappingMode        mapMode,
                                       MapResolution      resolutionMode)
: LocalMetricCommand(source)
, slamMode_(slamMode)
, mappingMode_(mapMode)
, resolution_(resolutionMode)
{
}
    

SetSlamModeCommand::SetSlamModeCommand(const std::string& source, SlamMode slamMode)
: SetSlamModeCommand(source, slamMode, MappingMode::unchanged, MapResolution::unchanged)
{
}


SetSlamModeCommand::SetSlamModeCommand(const std::string& source, MappingMode mapMode)
: SetSlamModeCommand(source, SlamMode::unchanged, mapMode, MapResolution::unchanged)
{
}


SetSlamModeCommand::SetSlamModeCommand(const std::string& source, MapResolution resolutionMode)
: SetSlamModeCommand(source, SlamMode::unchanged, MappingMode::unchanged, resolutionMode)
{
}


void SetSlamModeCommand::issue(const metric_slam_data_t& data, 
                               Localizer&   localizer, 
                               Mapper&                   mapper, 
                               MetricRelocalizer&        relocalizer) const
{
    issue_slam_mode_command(slamMode_, mapper);
    issue_mapping_mode_command(mappingMode_, mapper);
    issue_map_resolution_command(resolution_, mapper);
}


void SetSlamModeCommand::print(std::ostream& out) const
{
    out << "SetSlamModeCommand from " << source() << ": Slam:" << slamMode_ << " Mapping:" << mappingMode_
        << " Resolution:" << resolution_;
}


void issue_slam_mode_command(SlamMode mode, Mapper& mapper)
{
    switch(mode)
    {
    case SlamMode::full_slam:
        mapper.shouldUpdateMap(true);
        break;
        
    case SlamMode::localization_only:
        mapper.shouldUpdateMap(false);
        break;
        
    case SlamMode::unchanged:
        // Don't change things
        break;
    }
}


void issue_mapping_mode_command(MappingMode mode, Mapper& mapper)
{
    switch(mode)
    {
    case MappingMode::expanding:
        mapper.setMappingMode(MapperMode::expanding);
        break;
        
    case MappingMode::scrolling:
        mapper.setMappingMode(MapperMode::scrolling);
        break;
        
    case MappingMode::unchanged:
        // Don't change things
        break;
    }
}


void issue_map_resolution_command(MapResolution res, Mapper& mapper)
{
    switch(res)
    {
    case MapResolution::high:
        mapper.shouldBuildHighResMap(true);
        break;
        
    case MapResolution::normal:
        mapper.shouldBuildHighResMap(false);
        break;
        
    case MapResolution::unchanged:
        // Don't change things
        break;
    }
}


std::ostream& operator<<(std::ostream& out, SlamMode mode)
{
    switch(mode)
    {
    case SlamMode::full_slam:
        out << "full slam";
        break;
        
    case SlamMode::localization_only:
        out << "localization only";
        break;
        
    case SlamMode::unchanged:
        out << "unchanged";
        break;
    }
    
    return out;
}


std::ostream& operator<<(std::ostream& out, MappingMode mode)
{
    switch(mode)
    {
    case MappingMode::expanding:
        out << "expanding";
        break;
        
    case MappingMode::scrolling:
        out << "scrolling";
        break;
        
    case MappingMode::unchanged:
        out << "unchanged";
        break;
    }
    
    return out;
}


std::ostream& operator<<(std::ostream& out, MapResolution mode)
{
    switch(mode)
    {
    case MapResolution::high:
        out << "high";
        break;
        
    case MapResolution::normal:
        out << "normal";
        break;
        
    case MapResolution::unchanged:
        out << "unchanged";
        break;
    }
    
    return out;
}

} // namespace hssh
} // namespace vulcan
