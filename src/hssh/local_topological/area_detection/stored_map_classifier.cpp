/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stored_map_classifier.cpp
* \author   Collin Johnson
*
* Definition of StoredMapClassifier.
*/

#include "hssh/local_topological/area_detection/stored_map_classifier.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/local_topo_map.h"
#include "utils/serialized_file_io.h"

namespace vulcan
{
namespace hssh
{

StoredMapClassifier::StoredMapClassifier(const std::string& mapFilename)
{
    LocalTopoMap map;

    areas_.second = 0.0;

    if(utils::load_serializable_from_file(mapFilename, map))
    {
        for(auto& area : map)
        {
            areas_.first.push_back(area);
        }
    }
    else
    {
        // TODO: Throw exception if the file doesn't exist
        std::cerr << "ERROR: StoredMapClassifier: Desired map doesn't exist:" << mapFilename << '\n';
        assert(!"Have grouth-truth map");
    }
}


LabelingError StoredMapClassifier::classifyAreas(const std::vector<Gateway>& gateways,
                                                 const VoronoiSkeletonGrid&  skeleton,
                                                 const VoronoiIsovistField&  isovistField,
                                                 const LocalPerceptualMap&   lpm)
{
    // Nothing to label, so always successful as long as some areas were loaded
    return areas_.first.empty() ? LabelingError::no_labeling_solution : LabelingError::success;
}


void StoredMapClassifier::processAreaEvents(const LocalAreaEventVec& events)
{
    // Nothing to process
}


StoredMapClassifier::Areas StoredMapClassifier::currentAreas(void) const
{
    return areas_;
}


void StoredMapClassifier::sendDebug(system::DebugCommunicator& communicator) const
{
    // Nothing to send
}

}
}
