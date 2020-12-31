/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     labeled_boundary_data.h
 * \author   Collin Johnson
 *
 * Declaration of LabeledBoundaryFeatures and LabeledBoundaryData.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_LABELED_BOUNDARY_DATA_H
#define HSSH_LOCAL_TOPOLOGICAL_LABELED_BOUNDARY_DATA_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_type.h"
#include "hssh/local_topological/training/topo_training_data.h"

namespace vulcan
{
namespace hssh
{

/**
 * LabeledBoundaryFeatures stores the types of the areas on either side of a boundary.
 */
struct LabeledBoundaryFeatures
{
    // Necessary to satisfy the LabeledData concept even though only one feature version exists
    struct DummyFeatureVersion
    {
        int version(void) const { return 0; }
    };

    std::array<HypothesisType, 2> types;   // types of areas on either side of the boundary
    bool isOn;                             // flag indicating if this gateway was accepted
    DummyFeatureVersion features;
};


/**
 * LabeledBoundaryData stores the data about the boundary adjacencies found in the ground-truth topological maps.
 */
class LabeledBoundaryData : public TopoTrainingData<LabeledBoundaryFeatures>
{
public:
    LabeledBoundaryData(const TopoTrainingData<LabeledBoundaryFeatures>& data)
    : TopoTrainingData<LabeledBoundaryFeatures>(data)
    {
    }

    LabeledBoundaryData(void) = default;
};

// Operators

/**
 * Equality operator. Equal when the type and features match.
 */
bool operator==(const LabeledBoundaryFeatures& lhs, const LabeledBoundaryFeatures& rhs);
bool operator!=(const LabeledBoundaryFeatures& lhs, const LabeledBoundaryFeatures& rhs);


std::ostream& operator<<(std::ostream& out, const LabeledBoundaryFeatures& example);
std::istream& operator>>(std::istream& in, LabeledBoundaryFeatures& example);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_LABELED_BOUNDARY_DATA_H
