/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     csp_debug.h
* \author   Collin Johnson
*
* Definition of CSPDebugInfo used for visualizing how the CSP search is solved.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_CSP_DEBUG_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_CSP_DEBUG_H

#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_extent.h>
#include <math/geometry/rectangle.h>
#include <system/message_traits.h>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

/**
* CSPArea defines the state of an area being solved during the CSP search for a labeling.
*/
struct CSPArea
{
    math::Rectangle<float> boundary;
    HypothesisType oldType;
    HypothesisType newType;
};

/**
* CSPIteration defines an iteration of the CSP algorithm. It includes all areas that were failing constraints and the
* area that was selected and updated.
*/
struct CSPIteration
{
    std::vector<HypothesisType> labels;         // label assigned to each extent
    std::vector<double> strains;                // strain for each extent
    std::vector<bool> isFailing;                // flag indicating if the particular extent belongs to a failing area
    std::vector<Endpoints> pathEndpoints;       // endpoints for those areas that are path segments
    std::vector<Line<double>> gateways;   // active gateways
    std::vector<CSPArea> failedAreas;           // Areas with failed constraints
    CSPArea updatedArea;                        // Area that was updated on this iteration
};

/**
* CSPDebugInfo contains the results of the a CSP search. It includes each iteration of the search from beginning to end.
*/
struct CSPDebugInfo
{
    std::vector<AreaExtent> extents;
    std::vector<CSPIteration> iterations;
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, CSPArea& area)
{
    ar( area.boundary,
        area.oldType,
        area.newType);
}

template <class Archive>
void serialize(Archive& ar, CSPIteration& iteration)
{
    ar( iteration.labels,
        iteration.strains,
        iteration.isFailing,
        iteration.pathEndpoints,
        iteration.gateways,
        iteration.failedAreas,
        iteration.updatedArea);
}

template <class Archive>
void serialize(Archive& ar, CSPDebugInfo& info)
{
    ar( info.extents,
        info.iterations);
}

}
}

DEFINE_DEBUG_MESSAGE(hssh::CSPDebugInfo, ("HSSH_CSP_DEBUG_INFO"))

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_CSP_DEBUG_H
