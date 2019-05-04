/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stored_map_classifier.h
* \author   Collin Johnson
* 
* Declaration of StoredMapClassifier.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_STORED_MAP_CLASSIFIER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_STORED_MAP_CLASSIFIER_H

#include <hssh/local_topological/area_detection/area_classifier.h>

namespace vulcan
{
namespace hssh
{
    
const std::string kStoredMapClassifierType("stored-map");


/**
* StoredMapClassifier is an AreaClassifier instance that loads a LocalTopoMap from file and always returns it. It
* doesn't do any labeling. This classifier allows for using the global_metric_hssh to drive via the Decision interface
* through a known, either batch-labeled or hand-labeled, environment.
*/
class StoredMapClassifier : public AreaClassifier
{
public:
    
    /**
    * Constructor for StoredMapClassifier.
    * 
    * \param    mapFilename         File containing the map to be loaded
    * \throws   
    */
    StoredMapClassifier(const std::string& mapFilename);
    
    // AreaClassifier interface
    LabelingError classifyAreas(const std::vector<Gateway>& gateways,
                                const VoronoiSkeletonGrid&  skeleton,
                                const VoronoiIsovistField&  isovistField,
                                const LocalPerceptualMap&   lpm) override;
    void processAreaEvents(const LocalAreaEventVec& events) override;
    Areas currentAreas(void) const override;
    void  sendDebug(system::DebugCommunicator& communicator) const override;

private:

    Areas areas_;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_STORED_MAP_CLASSIFIER_H
