/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     feature_extraction.cpp
 * \author   Collin Johnson
 *
 * Definition of functions for feature extraction for finding gateways:
 *
 *   - extract_gateway_features_default
 */

#include "hssh/local_topological/area_detection/gateways/feature_extraction.h"
#include "hssh/local_topological/area_detection/gateways/isovist_gradients.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_edges.h"
#include "utils/plot2d.h"
#include <array>
#include <cassert>

// #define DEBUG_FEATURES

namespace vulcan
{
namespace hssh
{

const int kGatewayFeatureVersion = 96;

static std::vector<utils::Isovist::Scalar> kDerivs = {
  utils::Isovist::kArea,
  utils::Isovist::kOrientation,
  utils::Isovist::kWeightedOrientation,
  utils::Isovist::kShapeEccentricity,
  utils::Isovist::kShapeCompactness,
  utils::Isovist::kMaxThroughDistOrientation,   // orientation of max through dist
  utils::Isovist::kDmax,
  utils::Isovist::kDmin,
  utils::Isovist::kDavg,
  utils::Isovist::kShapeDistVariation,
  utils::Isovist::kDistRelationAvg,
  utils::Isovist::kDistRelationStd,
};

static std::vector<utils::Isovist::Scalar> kScalars = {utils::Isovist::kAngleBetweenMinDists,
                                                       utils::Isovist::kMinHalfArea,
                                                       utils::Isovist::kMinNormalDiff};

// Integrated gradient, r = [0,R], + scalar derivative
inline std::size_t feature_width(std::size_t radius)
{
    return radius + 2;
}

SkeletonFeatures allocate_feature_vectors(const VoronoiEdges& edges, std::size_t radius, std::size_t numFeatures);
void add_features_to_cell(cell_t cell,
                          int radius,
                          utils::Isovist::Scalar scalar,
                          const CellVector& edge,
                          const VoronoiIsovistField& isovists,
                          const VoronoiIsovistGradients& gradients,
                          utils::FeatureVector::iterator featureBegin);


std::string gateway_feature_name(int index, int radius)
{
    if ((index < 0) || (radius < 0)) {
        return "";
    }

    std::ostringstream out;

    int numDerivs = static_cast<int>(feature_width(radius) * kDerivs.size());

    if (index >= numDerivs && (index - numDerivs < static_cast<int>(kScalars.size()))) {
        out << utils::Isovist::scalarName(kScalars[index - numDerivs]);
    } else {
        int derivIndex = index / feature_width(radius);
        int derivRad = index % feature_width(radius);
        out << utils::Isovist::scalarName(kDerivs[derivIndex]) << ' ' << (derivRad - 1);
    }

    return out.str();
}


void set_default_gateway_features(const std::vector<utils::Isovist::Scalar>& scalars)
{
    kDerivs = scalars;
}


int current_gateway_features_version(void)
{
    return kGatewayFeatureVersion;
}


SkeletonFeatures
  extract_gateway_features_default(const VoronoiEdges& edges, const VoronoiIsovistField& isovists, int radius)
{
    return extract_gateway_features(edges, isovists, kDerivs, radius);
}


SkeletonFeatures extract_gateway_features(const VoronoiEdges& edges,
                                          const VoronoiIsovistField& isovists,
                                          const std::vector<utils::Isovist::Scalar>& scalars,
                                          int radius)
{
    radius = std::max(radius, 0);   // radius can't be negative

    const std::size_t numDerivFeatures = feature_width(radius) * scalars.size();
    const std::size_t numFeatures = numDerivFeatures + kScalars.size();
    SkeletonFeatures features = allocate_feature_vectors(edges, radius, numFeatures);

    VoronoiIsovistGradients gradients(edges);
    // For each feature, add the gradients to each of the skeleton features
    for (std::size_t n = 0; n < scalars.size(); ++n) {
        gradients.calculateGradients(scalars[n], isovists);

        std::size_t featureStartIndex = n * feature_width(radius);

        // Each skeleton feature needs this set of gradients features added to it.
        for (auto& f : features) {
            auto edgeIt = edges.findEdgeForCell(f.first);
            assert(edgeIt != edges.end());

            add_features_to_cell(f.first,
                                 radius,
                                 scalars[n],
                                 *edgeIt,
                                 isovists,
                                 gradients,
                                 f.second.begin() + featureStartIndex);
        }
    }

    // Add non-deriv features
    for (std::size_t n = 0; n < kScalars.size(); ++n) {
        for (auto& f : features) {
            f.second[numDerivFeatures + n] = isovists.at(f.first).scalar(kScalars[n]);
        }
    }

#ifdef DEBUG_FEATURES
    utils::Plot2D plot("Eccentricity vs. Area", "eccentricity", "area");
    std::vector<double> eccentricity;
    std::vector<double> orientation;

    gradients.calculateGradients(utils::Isovist::kShapeEccentricity, isovists);
    for (auto& g : gradients) {
        eccentricity.push_back(g.value);
    }

    gradients.calculateGradients(utils::Isovist::kArea, isovists);
    for (auto& g : gradients) {
        orientation.push_back(g.value);
    }

    plot.addData(eccentricity, orientation);
    plot.plot(utils::PlotStyle::points);
#endif   // DEBUG_FEATURES

    return features;
}


SkeletonFeatures allocate_feature_vectors(const VoronoiEdges& edges, std::size_t radius, std::size_t numFeatures)
{
    //     const std::size_t kMinEdgeLength = feature_width(radius);

    SkeletonFeatures features;

    for (auto& e : edges) {
        for (auto& cell : e) {
            features.emplace(cell, utils::FeatureVector{numFeatures, kGatewayFeatureVersion});
        }
    }

    return features;
}


void add_features_to_cell(cell_t cell,
                          int radius,
                          utils::Isovist::Scalar scalar,
                          const CellVector& edge,
                          const VoronoiIsovistField& isovists,
                          const VoronoiIsovistGradients& gradients,
                          utils::FeatureVector::iterator featureBegin)
{
    //     *featureBegin++ = isovists.at(cell).scalar(scalar);
    *featureBegin++ = isovists.at(cell).scalarDeriv(scalar);

    auto cellIt = std::find(edge.begin(), edge.end(), cell);
    assert(cellIt != edge.end());

    int leftRadius = std::min(static_cast<long>(radius), std::distance(edge.begin(), cellIt));
    int rightRadius = std::min(static_cast<long>(radius), std::distance(cellIt, edge.end()) - 1);

    double sumAlongEdge = std::abs(gradients.gradientAt(*cellIt).value);
    *featureBegin++ = sumAlongEdge;

    for (int r = 1; r <= radius; ++r) {
        if (r <= leftRadius) {
            sumAlongEdge += std::abs(gradients.gradientAt(*(cellIt - r)).value);
        }
        if (r <= rightRadius) {
            sumAlongEdge += std::abs(gradients.gradientAt(*(cellIt + r)).value);
        }

        *featureBegin++ = sumAlongEdge;
    }
}

}   // namespace hssh
}   // namespace vulcan
