/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_classifier.cpp
 * \author   Collin Johnson
 *
 * Definition of create_area_classifier factory function.
 */

#include "hssh/local_topological/area_detection/area_classifier.h"
#include "hssh/local_topological/area_detection/affordance_labeling_classifier.h"
#include "hssh/local_topological/area_detection/stored_map_classifier.h"
#include "hssh/local_topological/params.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<AreaClassifier>
  create_area_classifier(const std::string& type, const std::string& mapName, const area_classifier_params_t& params)
{
    if (type == kStoredMapClassifierType) {
        return std::unique_ptr<AreaClassifier>(new StoredMapClassifier(params.storedMapFile));
    } else if (type == kAffordanceLabelingClassifier) {
        return std::unique_ptr<AreaClassifier>(new AffordanceLabelingClassifier(params, mapName));
    }

    std::cerr << "ERROR: create_area_classifier: Attempted to create subclass of unknown type: " << type << '\n';
    assert(-1);
    return std::unique_ptr<AreaClassifier>();
}

}   // namespace hssh
}   // namespace vulcan
