/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     feature_extraction.h
* \author   Collin Johnson
*
* Declaration of functions for feature extraction for finding gateways:
*
*   - extract_gateway_features_default: create a GatewayFeatures instance for each cell in the skeleton
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_FEATURE_EXTRACTION_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_FEATURE_EXTRACTION_H

#include "hssh/types.h"
#include "utils/feature_vector.h"
#include "utils/isovist.h"
#include <utility>
#include <vector>

namespace vulcan
{
namespace hssh
{

class VoronoiEdges;
class VoronoiIsovistField;

/**
* SkeletonFeatures are a pairing of the the skeleton cell with a computed feature vector.
*/
using SkeletonFeatures = CellToTypeMap<utils::FeatureVector>;

/**
* extract_gateway_features_default extract a feature vector for each cell in the skeleton that is at least distance >= 'radius'
* from a junction point.
*
* The computed features use gradients in the isovist scalar field to define the changes over a small region of the
* skeleton centered around a single cell and extending 'radius' cells in both directions along the edge.
*
* \param    edges           Edges in the skeleton
* \param    isovists        The isovist field for the skeleton cells in the edges
* \param    radius          Radius to extend the feature vector (optional)
* \return   A set of features for each cell in the skeleton at least 'radius' distance from a junction point.
*/
SkeletonFeatures extract_gateway_features_default(const VoronoiEdges& edges,
                                                  const VoronoiIsovistField& isovists,
                                                  int radius = 1);

/**
* extract_gateway_features extract a feature vector for each cell in the skeleton that is at least distance >= 'radius'
* from a junction point.
*
* The computed features use gradients in the isovist scalar field to define the changes over a small region of the
* skeleton centered around a single cell and extending 'radius' cells in both directions along the edge.
*
* \param    edges           Edges in the skeleton
* \param    isovists        The isovist field for the skeleton cells in the edges
* \param    scalars         Isovist scalars to defining the isovist fields to compute features for
* \param    radius          Radius to extend the feature vector (optional, default = 2)
* \return   A set of features for each cell in the skeleton at least 'radius' distance from a junction point.
*/
SkeletonFeatures extract_gateway_features(const VoronoiEdges& edges,
                                          const VoronoiIsovistField& isovists,
                                          const std::vector<utils::Isovist::Scalar>& scalars,
                                          int radius = 1);

/**
* gateway_feature_name retrieves the name of the feature with the given index.
*
* \param    index           Feature index
* \param    radius          Radius used for the features (optional, default = 1)
* \return   Name of the feature:  isovist radius (if there is one). Radius of -1 means it's a scalar deriv.
*/
std::string gateway_feature_name(int index, int radius = 1);

/**
* set_default_gateway_features
*/
void set_default_gateway_features(const std::vector<utils::Isovist::Scalar>& scalars);

/**
* current_gateway_features_version retrieves the current feature version number of the gateways.
*/
int current_gateway_features_version(void);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_FEATURE_EXTRACTION_H
