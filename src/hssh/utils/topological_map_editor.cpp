/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map_editor.cpp
* \author   Collin Johnson
*
* Definition of TopologicalMapEditor.
*/

#include <hssh/utils/topological_map_editor.h>

namespace vulcan
{
namespace hssh
{

TopologicalMapEditor::TopologicalMapEditor(MetricMapCache& manager, size_t undoSteps)
                                    : manager(manager)
                                    , maxHistorySize(undoSteps)
{
}


bool TopologicalMapEditor::loadInitialMap(const std::string& filename)
{
    // TODO: Create file format for topological maps
}


void TopologicalMapEditor::createNewMap(void)
{

}


bool TopologicalMapEditor::saveMap(const std::string& filename)
{
    // TODO: Create file format for topological maps
}


uint32_t TopologicalMapEditor::addPlace(const LocalPlace& place, const pose_t& transform)
{

}


bool TopologicalMapEditor::deletePlace(uint32_t id)
{
    if(unattachedPlaces.find(id) != unattachedPlaces.end())
    {
        unattachedPlaces.erase(id);
        return true;
    }

    return false;
}


bool TopologicalMapEditor::connectPlaces(uint32_t placeId, uint32_t otherPlaceId, uint32_t fragmentId, uint32_t otherFragmentId)
{

}


bool TopologicalMapEditor::undo(void)
{

}


bool TopologicalMapEditor::redo(void)
{

}


const std::vector<GlobalPlace>& TopologicalMapEditor::getPlaces(void) const
{

}


const std::vector<GlobalPath>& TopologicalMapEditor::getPaths(void) const
{

}

} // namespace hssh
} // namespace vulcan
