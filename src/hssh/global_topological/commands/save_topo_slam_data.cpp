/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     save_topo_slam_data.cpp
 * \author   Collin Johnson
 *
 * Definition of SaveTopoSlamDataCommand.
 */

#include "hssh/global_topological/commands/save_topo_slam_data.h"
#include "hssh/global_topological/topo_slam.h"

namespace vulcan
{
namespace hssh
{

std::ostream& operator<<(std::ostream& out, TopoSlamDataType type);


SaveTopoSlamDataCommand::SaveTopoSlamDataCommand(TopoSlamDataType type,
                                                 const std::string& filename,
                                                 const std::string& source)
: GlobalTopoCommand(source)
, type_(type)
, filename_(filename)
{
}


void SaveTopoSlamDataCommand::issue(TopologicalSLAM& slam) const
{
    bool success = false;

    switch (type_) {
    case TopoSlamDataType::map_cache:
        success = slam.saveMapCache(filename_);
        break;

    case TopoSlamDataType::tree_of_maps:
        success = slam.saveTreeOfMaps(filename_);
        break;

    case TopoSlamDataType::visit_sequence:
        success = slam.saveVisitSequence(filename_);
        break;
    }

    if (!success) {
        std::cerr << "ERROR: SaveTopoSlamDataCommand: Failed to save " << type_ << " to " << filename_ << '\n';
    }
}


void SaveTopoSlamDataCommand::print(std::ostream& out) const
{
    out << "SaveTopoSlamDataCommand from " << source() << " saving " << type_ << " to " << filename_ << '\n';
}


std::ostream& operator<<(std::ostream& out, TopoSlamDataType type)
{
    switch (type) {
    case TopoSlamDataType::map_cache:
        out << "map_cache";
        break;

    case TopoSlamDataType::tree_of_maps:
        out << "tree_of_maps";
        break;

    case TopoSlamDataType::visit_sequence:
        out << "visit_sequence";
        break;
    }

    return out;
}

}   // namespace hssh
}   // namespace vulcan
