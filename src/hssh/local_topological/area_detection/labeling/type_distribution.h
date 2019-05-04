/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     type_distribution.h
* \author   Collin Johnson
*
* Declaration of HypothesisTypeDistribution and BoundaryTypeDistribution.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_TYPE_DISTRIBUTION_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_TYPE_DISTRIBUTION_H

#include <hssh/local_topological/area_detection/labeling/hypothesis_type.h>
#include <hssh/types.h>
#include <core/matrix.h>

namespace vulcan
{
namespace hssh
{

const int kPathIdx = 0;
const int kDecisionIdx = 1;
const int kDestIdx = 2;
const int kPathEndIdx = 3;

/**
* HypothesisTypeDistribution stores the appropriateness distribution for the type of a particular AreaHypothesis. The
* distribution stores the appropriateness of a type being a path, decision, or destination.
*/
struct HypothesisTypeDistribution
{
    double path = 1.0 / 3.0;            ///< Appropriateness of the hypothesis being a path segment
    double destination = 1.0 / 3.0;     ///< Appropriateness of the hypothesis being a destination
    double decision = 1.0 / 3.0;        ///< Appropriateness of the hypothesis being a decision point

    HypothesisType mostAppropriate(void) const
    {
        if((path > destination) && (path > decision))
        {
            return HypothesisType::kPath;
        }
        else if(destination > decision)
        {
            return HypothesisType::kDest;
        }
        else
        {
            return HypothesisType::kDecision;
        }
    }

    double maxAppropriateness(void) const
    {
        if(path > destination && path > decision)
        {
            return path;
        }
        else if(destination > path && destination > decision)
        {
            return destination;
        }
        else
        {
            return decision;
        }
    }

    double typeAppropriateness(HypothesisType type) const
    {
        switch(type)
        {
        case HypothesisType::kDecision:
            return decision;

        case HypothesisType::kDest:
            return destination;

        case HypothesisType::kPath:
            return path;

        default:
            return 0.0;
        }
    };

    void normalize(void)
    {
        double total = path + destination + decision;

        if(total > 0.0)
        {
            path /= total;
            destination /= total;
            decision /= total;
        }
    }
};

/**
* BoundaryTypeDistribution represents the 4x4 matrix of possible combinations of area types for an area boundary. The
* indices into the matrix can be obtained via the typeIdx(HypothesisType) method.
*/
struct BoundaryTypeDistribution
{
    Matrix posModel;  // 4x4 -- gateway on prior
    Matrix negModel;  // 4x4 -- gateway off prior

    /**
    * typeIdx converts a hypothesis type to the corresponding index into the distribution matrix.
    */
    int typeIdx(HypothesisType type) const
    {
        switch(type)
        {
        case HypothesisType::kPathDest:
            return kPathIdx;
        case HypothesisType::kDecision:
            return kDecisionIdx;
        case HypothesisType::kDest:
            return kDestIdx;
        case HypothesisType::kPathEndpoint:
            return kPathEndIdx;
        default:
            return -1;
        }

        return -1;
    }

    int typeIdx(AreaType type) const
    {
        switch(type)
        {
        case AreaType::path_segment:
            return kPathIdx;
        case AreaType::decision_point:
            return kDecisionIdx;
        case AreaType::destination:
            return kDestIdx;
        default:
            return -1;
        }

        return -1;
    }
};


} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_TYPE_DISTRIBUTION_H
