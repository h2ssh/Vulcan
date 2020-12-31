/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeled_area_data.h
* \author   Collin Johnson
* 
* Declaration of LabeledAreaData.
*/

#ifndef HSSH_AREA_DETECTION_LABELING_LABELED_AREA_DATA_H
#define HSSH_AREA_DETECTION_LABELING_LABELED_AREA_DATA_H

#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "hssh/local_topological/training/topo_training_data.h"

namespace vulcan
{
namespace hssh
{
    
/**
* LabeledFeatures contains a labeled example of the features calculated for an area.
*/
struct LabeledFeatures
{
    HypothesisType     type;
    HypothesisFeatures features;
};


class LabeledAreaData : public TopoTrainingData<LabeledFeatures>
{
    // LabeledAreaData is just a specific instance of the general TopoTrainingData.
    // Use of subclass allows forward declaration of LabeledAreaData whereever needed.
};

/**
* Equality operator. Equal when the type and features match.
*/
bool operator==(const LabeledFeatures& lhs, const LabeledFeatures& rhs);
bool operator!=(const LabeledFeatures& lhs, const LabeledFeatures& rhs);

std::ostream& operator<<(std::ostream& out, const LabeledFeatures& example);
std::istream& operator>>(std::istream& in, LabeledFeatures& example);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_AREA_DETECTION_LABELING_LABELED_AREA_DATA_H
