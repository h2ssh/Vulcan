/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     save_topo_slam_data.h
* \author   Collin Johnson
* 
* Declaration of SaveTopoSlamDataCommand and TopoSlamDataType enum.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_COMMANDS_SAVE_TOPO_SLAM_DATA_H
#define HSSH_GLOBAL_TOPOLOGICAL_COMMANDS_SAVE_TOPO_SLAM_DATA_H

#include "hssh/global_topological/command.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh 
{
    
/**
* TopoSlamDataType defines the types of TopoSLAM data that can be saved via the SaveTopoSlamDataCommand.
*/
enum class TopoSlamDataType
{
    map_cache,
    tree_of_maps,
    visit_sequence,
};

/**
* SaveTopoSlamDataCommand is a command to save the requested type of data to a file. The types of data that can
* be saved are specified in the TopoSlamDataType enum. These values correspond to the exposed saveXXXX methods in the 
* TopologicalSLAM class.
*/
class SaveTopoSlamDataCommand : public GlobalTopoCommand
{
public:
    
    /**
    * Constructor for SaveTopoSlamDataCommand.
    * 
    * \param    type                Type of data to be saved
    * \param    filename            Name of the file in which to save the data
    * \param    source              Source of the message
    */
    SaveTopoSlamDataCommand(TopoSlamDataType type, const std::string& filename, const std::string& source);
    
    
    /////   GlobalTopoCommand interface   /////
    void issue(TopologicalSLAM& slam) const override;
    void print(std::ostream& out) const override;

private:
    
    TopoSlamDataType type_;
    std::string filename_;
    
    // Serialization support 
    friend class cereal::access; 
    
    SaveTopoSlamDataCommand(void) = default;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( type_,
            filename_
        );
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_COMMANDS_SAVE_TOPO_SLAM_DATA_H
