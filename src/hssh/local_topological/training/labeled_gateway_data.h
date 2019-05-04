/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeled_gateway_data.h
* \author   Collin Johnson
* 
* Declaration of LabeledGatewayData and LabeledGatewayFeatures.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_TRAINING_LABELED_GATEWAY_DATA_H
#define HSSH_LOCAL_TOPOLOGICAL_TRAINING_LABELED_GATEWAY_DATA_H

#include <hssh/local_topological/training/topo_training_data.h>
#include <hssh/types.h>
#include <utils/feature_vector.h>

namespace vulcan
{
namespace hssh
{
    
/**
* LabeledGatewayFeatures contains a labeled example of the features calculated for an area.
*/
struct LabeledGatewayFeatures
{
    bool isGateway;
    cell_t cell;
    utils::FeatureVector features;
};

/**
* LabeledGatewayData contains a set of labeled areas and their features for one or more completed maps. The data is
* organized both on a per-map and a complete basis. The data consists of a sequence of LabeledGatewayFeatures, each of which
* contains the extracted HypothesisFeatures for an area along with its hand-assigned label. The data is intended to be
* used for training and testing purposes for a HypothesisFeaturesClassifier.
*/
class LabeledGatewayData : public TopoTrainingData<LabeledGatewayFeatures>
{
public:
    
    LabeledGatewayData(const TopoTrainingData<LabeledGatewayFeatures>& features)
    : TopoTrainingData<LabeledGatewayFeatures>(features)
    {
    }
    
    LabeledGatewayData(void) = default;
};

// Operators

/**
* Equality operator. Equal when the type and features match.
*/
bool operator==(const LabeledGatewayFeatures& lhs, const LabeledGatewayFeatures& rhs);
bool operator!=(const LabeledGatewayFeatures& lhs, const LabeledGatewayFeatures& rhs);


std::ostream& operator<<(std::ostream& out, const LabeledGatewayFeatures& example);
std::istream& operator>>(std::istream& in, LabeledGatewayFeatures& example);
    
} // namespace hssh 
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_TRAINING_LABELED_GATEWAY_DATA_H
