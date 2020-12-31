/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_features.cpp
 * \author   Collin Johnson
 *
 * Definition of HypothesisFeatures.
 */

#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "core/float_comparison.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "math/covariance.h"
#include "math/moments_features.h"
#include "math/statistics.h"
#include "math/zernike_moments.h"
#include "utils/float_io.h"
#include "utils/visibility_graph.h"
#include "utils/visibility_graph_feature.h"
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <cmath>
#include <functional>
#include <sstream>

namespace vulcan
{
namespace hssh
{

const int kFeaturesVersion = 249;   // version number to assign to newly calculated features
const int kIsovistFeaturesVersion = 237;

// Keep a single instance so the pre-cached values can be used repeatedly, which will speed up the overall moments
// computation, which can be quite slow
static math::ZernikeMoments<5> zernike;

enum FeatureIndex
{
    kLoopDist,
    //     kUniqueNeighbors,
    kNumGateways,
    kEdgeLength,
    kAvgGatewayLength,
    kMaxGatewayLength,
    kGatewayRatio,
    //     kShapeArea,
    //     kShapePerimeter,
    kAxisRatioIndex,
    //     kWeightedAxisRatio,
    //     kMeanAreaComp,
    //     kStdAreaComp,
    //     kMeanPerimeterComp,
    //     kStdPerimeterComp,
    kShapeEccentricity,
    kShapeCircularity,
    //     kShapeCompactness,
    //     kDiscretizedCompactness,
    //     kAminCompactness,
    //     kAmaxCompactness,
    //     kPminCompactness,
    //     kPmaxCompactness,
    //     kShapeWaviness,
    //     kIsovistMeanArea,
    //     kIsovistStdArea,
    kIsovistVariationArea,
    //     kIsovistMeanPerimeter,
    //     kIsovistStdPerimeter,
    kIsovistVariationPerimeter,
    kIsovistMeanEccentricity,
    kIsovistStdEccentricity,
    kIsovistMeanCircularity,
    kIsovistStdCircularity,
    kIsovistMeanWaviness,
    kIsovistStdWaviness,
    kIsovistMeanCompactness,
    kIsovistStdCompactness,
    kIsovistMeanRayCompactness,
    kIsovistStdRayCompactness,
    kIsovistMeanShapeDistCompactness,
    kIsovistStdShapeDistCompactness,
    //     kIsovistMeanRelation,
    //     kIsovistStdRelation,
    kIsovistVariationRelation,
    //     kIsovistMeanDelta,
    //     kIsovistStdDelta,
    kIsovistVariationDelta,
    kVisGraphMeanClusterCoeff,
    kVisGraphStdClusterCoeff,
    kVisGraphMeanBetweenness,
    kVisGraphStdBetweenness,
    kVisGraphMeanPageRank,
    kVisGraphStdPageRank,
    kSkeletonMaxBetweenness,
    kSkeletonMeanBetweenness,
    kSkeletonStdBetweenness,
    //     kIsovistDistMinMean,
    //     kIsovistDistMinStd,
    //     kIsovistDistMinVariation,
    //     kIsovistDistMean,
    //     kIsovistDistStd,
    kIsovistDistVariation,
    //     kIsovistDistFromCenterMean,
    //     kIsovistDistFromCenterStd,
    kIsovistDistFromCenterVariation,
    kIsovistAngleBetweenMinDistsMean,
    kIsovistAngleBetweenMinDistsStd,
    //     kIsovistAreaBalanceMean,
    //     kIsovistAreaBalanceStd,
    kNumFixedFeatures
};

enum IsovistFeatureIndex
{
    kSingleEccentricity,
    kSingleCircularity,
    kSingleWaviness,
    kSingleCompactness,
    kSingleRayCompactness,
    kSingleShapeDistCompactness,
    //     kSingleDVariation,
    //     kSingleDeltaVariation,
    kSingleAvgDelta,
    kSingleStdDelta,
    kSingleAvgRelation,
    kSingleStdRelation,
    //     kSingleDistMin,
    kSingleDistMean,
    kSingleDistStd,
    //     kSingleDistSkewness,
    kSingleDistFromCenterMean,
    kSingleDistFromCenterStd,
    //     kSingleDistFromCenterVariation,
    kSingleAngleBetweenMinDists,
    //     kSingleMinHalfArea,
    //     kSingleAreaBalance,
    kSingleArea,
    kSinglePerimeter,
    kNumSingle,
};


struct CachedFeatures
{
    std::vector<cell_t> boundaryCells;
    HypothesisFeatures features;
};


static std::unordered_map<std::size_t, std::vector<CachedFeatures>> kFeaturesCache;

HypothesisFeatures* find_cached_features(const AreaHypothesis& hyp);
void add_features_to_cache(const AreaHypothesis& hyp, const HypothesisFeatures& features);


int current_hypothesis_features_version(void)
{
    return kFeaturesVersion;
}


////////////////////  Operators ///////////////////////////

std::ostream& operator<<(std::ostream& out, const HypothesisFeatures& features)
{
    out << features.version() << ' ' << features.numFeatures() << ' ';
    for (auto f : features) {
        utils::save_floating_point(out, f);
    }

    out << features.numIsovists() << ' ' << features.numIsovistFeatures() << ' ';
    for (auto f : features.isovistFeatures()) {
        utils::save_floating_point(out, f);
    }
    utils::save_floating_point(out, features.explorationAmount());
    return out;
}


std::istream& operator>>(std::istream& in, HypothesisFeatures& features)
{
    in >> features.version_;

    int numFeatures = 0;
    in >> numFeatures;

    features.features_.resize(numFeatures);
    for (int n = 0; n < numFeatures; ++n) {
        features.features_[n] = utils::load_floating_point(in);
    }

    int numIsovists = 0;
    int numIsovistFeatures = 0;
    in >> numIsovists >> numIsovistFeatures;
    features.isovistFeatures_.resize(numIsovistFeatures, numIsovists);
    for (auto& f : features.isovistFeatures_) {
        f = utils::load_floating_point(in);
    }
    features.exploration_ = utils::load_floating_point(in);
    return in;
}


bool operator==(const HypothesisFeatures& lhs, const HypothesisFeatures& rhs)
{
    if (lhs.numFeatures() != rhs.numFeatures()) {
        return false;
    }

    bool areSame = absolute_fuzzy_equal(lhs.explorationAmount(), rhs.explorationAmount());

    for (std::size_t n = 0; n < lhs.numFeatures(); ++n) {
        areSame = absolute_fuzzy_equal(lhs.featureAt(n), rhs.featureAt(n));

        if (!areSame) {
            break;
        }
    }

    return areSame;
}


bool operator!=(const HypothesisFeatures& lhs, const HypothesisFeatures& rhs)
{
    return !(lhs == rhs);
}


//////////////////// HypothesisFeatures /////////////////////////////

void HypothesisFeatures::ClearCache(void)
{
    kFeaturesCache.clear();
}


HypothesisFeatures::HypothesisFeatures(const AreaHypothesis& hypothesis,
                                       const AreaExtent& extent,
                                       const VoronoiSkeletonGrid& grid,
                                       const VoronoiIsovistField& isovistField,
                                       const utils::VisibilityGraph& visGraph,
                                       const utils::VisibilityGraph& skeletonGraph)
: HypothesisFeatures()
{
    //     HypothesisFeatures* cached = find_cached_features(hypothesis);
    //     if(cached)
    //     {
    //         *this = *cached;
    //     }
    //     else
    //     {
    assert(features_.size() == numFeatures());

    exploration_ = 1.0 - hypothesis.extent().frontierRatio();

    features_.fill(NAN);
    calculateStructureFeatures(hypothesis, grid);
    calculateAxisFeatures(hypothesis, grid);
    calculateIsovistFeatures(hypothesis, isovistField);
    calculateVisGraphFeatures(hypothesis, visGraph);
    calculateSkeletonGraphFeatures(hypothesis, skeletonGraph);
    calculateShapeFeatures(extent, grid);
    //         calculateZernikeFeature(extent);

    //         axisRatio_ = features_[kAxisRatioIndex];

    //         add_features_to_cache(hypothesis, *this);

    for (arma::uword idx = 0; idx < features_.n_elem; ++idx) {
        if (std::isnan(features_[idx])) {
            std::cout << "Found NAN in feature: " << featureName(idx)
                      << " Area:" << extent.rectangleBoundary(math::ReferenceFrame::GLOBAL) << '\n'
                      << " Num cells:" << hypothesis.numSkeleton() << '\n';
            assert(!std::isnan(features_[idx]));
        }
    }
    //     }
}


HypothesisFeatures::HypothesisFeatures(const Vector& features, double exploration, double axisRatio)
: version_(kFeaturesVersion)
, isovistVersion_(kIsovistFeaturesVersion)
, features_(features)
, exploration_(exploration)
, axisRatio_(axisRatio)
{
}


HypothesisFeatures::HypothesisFeatures(void)
: version_(kFeaturesVersion)
, isovistVersion_(kIsovistFeaturesVersion)
, features_(kNumFixedFeatures)
// , features_(kNumFixedFeatures + zernike.numMoments())
{
}


std::string HypothesisFeatures::featureName(int index)
{
    FeatureIndex feature = static_cast<FeatureIndex>(index);

    switch (feature) {
    case kLoopDist:
        return "loop dist";
        //         case kUniqueNeighbors:
        //             return "unique neighbors";
    case kNumGateways:
        return "num gateways";
    case kEdgeLength:
        return "edge length";
    case kAvgGatewayLength:
        return "avg gateway length";
    case kMaxGatewayLength:
        return "max gateway length";
    case kGatewayRatio:
        return "gateway ratio";
        //         case kShapeArea:
        //             return "shape area";
        //         case kShapePerimeter:
        //             return "shape perimeter";
    case kAxisRatioIndex:
        return "axis ratio";
        //         case kWeightedAxisRatio:
        //             return "weighted axis ratio";
        //         case kMeanAreaComp:
        //             return "area comp mean";
        //         case kStdAreaComp:
        //             return "area comp std";
        //         case kMeanPerimeterComp:
        //             return "perimeter comp mean";
        //         case kStdPerimeterComp:
        //             return "perimeter comp std";
    case kShapeEccentricity:
        return "shape eccentricity";
    case kShapeCircularity:
        return "shape circularity";
        //         case kShapeCompactness:
        //             return "shape compactness";
        //         case kDiscretizedCompactness:
        //             return "discretized compactness";
        //         case kAminCompactness:
        //             return "a_min compactness";
        //         case kAmaxCompactness:
        //             return "a_max compactness";
        //         case kPminCompactness:
        //             return "p_min compactness";
        //         case kPmaxCompactness:
        //             return "p_max compactness";
        //         case kShapeWaviness:
        //             return "shape waviness";
        //         case kIsovistMeanArea:
        //             return "area mean";
        //         case kIsovistStdArea:
        //             return "area std";
    case kIsovistVariationArea:
        return "area variation";
        //         case kIsovistMeanPerimeter:
        //             return "perimeter mean";
        //         case kIsovistStdPerimeter:
        //             return "perimeter std";
    case kIsovistVariationPerimeter:
        return "perimeter variation";
    case kIsovistMeanEccentricity:
        return "eccentricity mean";
    case kIsovistStdEccentricity:
        return "eccentricity std";
    case kIsovistMeanCircularity:
        return "circularity mean";
    case kIsovistStdCircularity:
        return "circularity std";
    case kIsovistMeanWaviness:
        return "waviness mean";
    case kIsovistStdWaviness:
        return "waviness std";
    case kIsovistMeanCompactness:
        return "compactness mean";
    case kIsovistStdCompactness:
        return "compactness std";
    case kIsovistMeanRayCompactness:
        return "ray compactness mean";
    case kIsovistStdRayCompactness:
        return "ray compactness std";
    case kIsovistMeanShapeDistCompactness:
        return "shape dist compactness mean";
    case kIsovistStdShapeDistCompactness:
        return "shape dist compactness std";
        //         case kIsovistMeanRelation:
        //             return "relation mean";
        //         case kIsovistStdRelation:
        //             return "relation std";
    case kIsovistVariationRelation:
        return "relation variation";
        //         case kIsovistMeanDelta:
        //             return "delta mean";
        //         case kIsovistStdDelta:
        //             return "delta std";
    case kIsovistVariationDelta:
        return "delta variation";
    case kVisGraphMeanClusterCoeff:
        return "cluster coeff mean";
    case kVisGraphStdClusterCoeff:
        return "cluster coeff std";
    case kVisGraphMeanBetweenness:
        return "betweenness mean";
    case kVisGraphStdBetweenness:
        return "betweenness std";
    case kVisGraphMeanPageRank:
        return "pagerank mean";
    case kVisGraphStdPageRank:
        return "pagerank std";
    case kSkeletonMaxBetweenness:
        return "skel betweenness max";
    case kSkeletonMeanBetweenness:
        return "skel betweenness mean";
    case kSkeletonStdBetweenness:
        return "skel betweenness std";
        //         case kIsovistDistMinMean:
        //             return "min dist mean";
        //         case kIsovistDistMinStd:
        //             return "min dist std";
        //         case kIsovistDistMinVariation:
        //             return "min dist variation";
        //         case kIsovistDistMean:
        //             return "dist mean";
        //         case kIsovistDistStd:
        //             return "dist std";
    case kIsovistDistVariation:
        return "dist variation";
        //         case kIsovistDistFromCenterMean:
        //             return "dist from center mean";
        //         case kIsovistDistFromCenterStd:
        //             return "dist from center std";
    case kIsovistDistFromCenterVariation:
        return "dist from center variation";
    case kIsovistAngleBetweenMinDistsMean:
        return "angle between min mean";
    case kIsovistAngleBetweenMinDistsStd:
        return "angle between min std";
        //         case kIsovistAreaBalanceMean:
        //             return "area balance mean";
        //         case kIsovistAreaBalanceStd:
        //             return "area balance std";
    case kNumFixedFeatures:
    default:
        break;
    }

    if ((index >= kNumFixedFeatures) && (index < static_cast<int>(kNumFixedFeatures + zernike.numMoments()))) {
        std::ostringstream str;
        str << "zernike: " << (index - kNumFixedFeatures);
        return str.str();
    }
    return "error";
}


std::string HypothesisFeatures::isovistFeatureName(int index)
{
    switch (index) {
    case kSingleEccentricity:
        return utils::Isovist::scalarName(utils::Isovist::kShapeEccentricity);
    case kSingleCircularity:
        return utils::Isovist::scalarName(utils::Isovist::kCircularity);
    case kSingleWaviness:
        return utils::Isovist::scalarName(utils::Isovist::kShapeWaviness);
    case kSingleCompactness:
        return utils::Isovist::scalarName(utils::Isovist::kShapeCompactness);
    case kSingleRayCompactness:
        return utils::Isovist::scalarName(utils::Isovist::kRayCompactness);
    case kSingleShapeDistCompactness:
        return utils::Isovist::scalarName(utils::Isovist::kShapeDistCompactness);
        //     case kSingleDVariation:
        //         return utils::Isovist::scalarName(utils::Isovist::kDVariation);
        //     case kSingleDeltaVariation:
        //         return utils::Isovist::scalarName(utils::Isovist::kDeltaVariation);
    case kSingleAvgDelta:
        return utils::Isovist::scalarName(utils::Isovist::kDeltaAvg);
    case kSingleStdDelta:
        return utils::Isovist::scalarName(utils::Isovist::kDeltaStd);
    case kSingleAvgRelation:
        return utils::Isovist::scalarName(utils::Isovist::kDistRelationAvg);
    case kSingleStdRelation:
        return utils::Isovist::scalarName(utils::Isovist::kDistRelationStd);
        //     case kSingleDistMin:
        //         return utils::Isovist::scalarName(utils::Isovist::kDmin);
    case kSingleDistMean:
        return utils::Isovist::scalarName(utils::Isovist::kDavg);
    case kSingleDistStd:
        return utils::Isovist::scalarName(utils::Isovist::kDstd);
        //     case kSingleDistSkewness:
        //         return utils::Isovist::scalarName(utils::Isovist::kDskewness);
    case kSingleDistFromCenterMean:
        return utils::Isovist::scalarName(utils::Isovist::kShapeDistAvg);
    case kSingleDistFromCenterStd:
        return utils::Isovist::scalarName(utils::Isovist::kShapeDistStd);
        //     case kSingleDistFromCenterVariation:
        //         return utils::Isovist::scalarName(utils::Isovist::kShapeDistVariation);
    case kSingleAngleBetweenMinDists:
        return utils::Isovist::scalarName(utils::Isovist::kAngleBetweenMinDists);
        //     case kSingleMinHalfArea:
        //         return utils::Isovist::scalarName(utils::Isovist::kMinHalfArea);
        //     case kSingleAreaBalance:
        //         return utils::Isovist::scalarName(utils::Isovist::kAreaBalance);
    case kSingleArea:
        return utils::Isovist::scalarName(utils::Isovist::kArea);
    case kSinglePerimeter:
        return utils::Isovist::scalarName(utils::Isovist::kPerimeter);
    }

    std::ostringstream str;
    str << "zernike: " << (index - kNumSingle);
    return str.str();
}


void HypothesisFeatures::calculateStructureFeatures(const AreaHypothesis& hypothesis, const VoronoiSkeletonGrid& grid)
{
    double totalGwyPerim = 0.0;

    for (auto& bnd : boost::make_iterator_range(hypothesis.beginBoundary(), hypothesis.endBoundary())) {
        totalGwyPerim += gateway_cell_perimeter(bnd->getGateway(), grid.metersPerCell());
    }

    if (hypothesis.extent().perimeter() > 0.0) {
        features_[kGatewayRatio] = totalGwyPerim / hypothesis.extent().perimeter();
    } else {
        features_[kGatewayRatio] = 0.0;
    }

    double minLoop = HUGE_VAL;
    for (auto& node : boost::make_iterator_range(hypothesis.beginNode(), hypothesis.endNode())) {
        minLoop = std::min(minLoop, node->getLoopDistance());
    }

    features_[kLoopDist] = minLoop;

    //     auto neighbors = hypothesis.findAdjacentHypotheses();
    //     double totalNeighbors = neighbors.size();
    //     std::sort(neighbors.begin(), neighbors.end());
    //     utils::erase_unique(neighbors);
    //
    //     features_[kUniqueNeighbors] = neighbors.empty() ? 0.0 : neighbors.size() / totalNeighbors;

    features_[kNumGateways] = hypothesis.numBoundaries();

    AreaGraph* graph = hypothesis.areaGraph();
    double totalLength = 0.0;
    int totalDists = 0;
    for (auto& node : boost::make_iterator_range(hypothesis.beginNode(), hypothesis.endNode())) {
        for (auto& otherNode : boost::make_iterator_range(hypothesis.beginNode(), hypothesis.endNode())) {
            totalLength += graph->distanceBetweenNodes(node, otherNode);
            ++totalDists;
        }
    }

    features_[kEdgeLength] = (totalDists > 0) ? totalLength / totalDists : 0.0;

    double totalGwyLength = 0.0;
    double maxGwyLength = 0.0;
    int numGwys = 0;

    for (auto& bnd : boost::make_iterator_range(hypothesis.beginBoundary(), hypothesis.endBoundary())) {
        totalGwyLength += bnd->length();
        maxGwyLength = std::max(bnd->length(), maxGwyLength);
    }

    features_[kAvgGatewayLength] = (numGwys > 0) ? totalGwyLength / numGwys : 0.0;
    features_[kMaxGatewayLength] = maxGwyLength;
}


void HypothesisFeatures::calculateAxisFeatures(const AreaHypothesis& hypothesis, const VoronoiSkeletonGrid& grid)
{
    std::vector<double> weights(hypothesis.numSkeleton(), 1.0);
    auto features =
      math::point2D_covariance_properties(hypothesis.beginSkeleton(), hypothesis.endSkeleton(), weights.begin());
    features_[kAxisRatioIndex] = features.eccentricity;

    //     std::transform(hypothesis.beginSkeleton(), hypothesis.endSkeleton(), weights.begin(), [&grid](cell_t c) {
    //         return grid.getMetricDistance(c.x, c.y);
    //     });
    //
    //     auto weightedFeatures = math::point2D_covariance_properties(hypothesis.beginSkeleton(),
    //                                                                 hypothesis.endSkeleton(),
    //                                                                 weights.begin());
    //     features_[kWeightedAxisRatio] = weightedFeatures.eccentricity;
}


void HypothesisFeatures::calculateShapeFeatures(const AreaExtent& extent, const VoronoiSkeletonGrid& grid)
{
    auto shapeFeatures = math::shape_features(extent.begin(), extent.end(), extent.area(), extent.center().toPoint());
    features_[kShapeEccentricity] = shapeFeatures[math::kShapeEccentricity];
    if (extent.perimeter() > 0.0) {
        features_[kShapeCircularity] = (4.0 * M_PI * extent.area()) / std::pow(extent.perimeter(), 2);
        //         features_[kShapeWaviness] = extent.hullPerimeter(grid) / extent.perimeter();
    } else {
        features_[kShapeCircularity] = 1.0;
        //         features_[kShapeWaviness] = 1.0;
    }

    //     features_[kShapeCompactness] = shapeFeatures[math::kShapeCompactness];
    //
    //     extent_compactness_t compactness = extent.compactness();
    //     features_[kShapeCircularity] = compactness.circularity;
    //     features_[kDiscretizedCompactness] = compactness.ndc;
    //     features_[kAminCompactness] = compactness.aMin;
    //     features_[kAmaxCompactness] = compactness.aMax;
    //     features_[kPminCompactness] = compactness.pMin;
    //     features_[kPmaxCompactness] = compactness.pMax;

    //     features_[kShapeArea] = extent.area();
    //     features_[kShapePerimeter] = extent.perimeter();
}


void HypothesisFeatures::calculateIsovistFeatures(const AreaHypothesis& hypothesis,
                                                  const VoronoiIsovistField& isovistField)
{
    using namespace boost::accumulators;
    using IsovistAcc = accumulator_set<double, stats<tag::mean, tag::max, tag::sum, tag::variance, tag::median>>;

    IsovistAcc eccentricityAcc;
    IsovistAcc circularityAcc;
    IsovistAcc compactnessAcc;
    IsovistAcc rayCompactnessAcc;
    IsovistAcc shapeDistCompactnessAcc;
    IsovistAcc wavinessAcc;
    IsovistAcc avgRelationAcc;
    IsovistAcc stdRelationAcc;
    IsovistAcc avgDeltaAcc;
    IsovistAcc stdDeltaAcc;
    IsovistAcc minDistAcc;
    IsovistAcc avgDistAcc;
    IsovistAcc stdDistAcc;
    IsovistAcc avgDistFromCenterAcc;
    IsovistAcc stdDistFromCenterAcc;
    IsovistAcc areaAcc;
    IsovistAcc perimAcc;
    IsovistAcc areaRatioAcc;
    IsovistAcc perimRatioAcc;
    IsovistAcc minDistAngleAcc;
    IsovistAcc areaBalanceAcc;

    double area = hypothesis.extent().area();
    double perimeter = hypothesis.extent().perimeter();

    //     int numIsovistMoments = (isovistField[0].numScalars() - utils::Isovist::kNumScalars);
    int numIsovistFeatures = kNumSingle;   // + numIsovistMoments;
    arma::colvec singleFeatures(numIsovistFeatures);
    isovistFeatures_.resize(numIsovistFeatures, std::distance(hypothesis.beginSkeleton(), hypothesis.endSkeleton()));
    int isovistCol = 0;

    for (auto& cell : boost::make_iterator_range(hypothesis.beginSkeleton(), hypothesis.endSkeleton())) {
        // Sum the variances to get the std for the full isovist field in the area
        const auto& isovist = isovistField[cell];
        eccentricityAcc(isovist.scalar(utils::Isovist::kShapeEccentricity));
        circularityAcc(isovist.scalar(utils::Isovist::kCircularity));
        wavinessAcc(isovist.scalar(utils::Isovist::kShapeWaviness));
        compactnessAcc(isovist.scalar(utils::Isovist::kShapeCompactness));
        rayCompactnessAcc(isovist.scalar(utils::Isovist::kRayCompactness));
        shapeDistCompactnessAcc(isovist.scalar(utils::Isovist::kShapeDistCompactness));
        avgRelationAcc(isovist.scalar(utils::Isovist::kDistRelationAvg));
        stdRelationAcc(std::pow(isovist.scalar(utils::Isovist::kDistRelationStd), 2.0));
        avgDeltaAcc(isovist.scalar(utils::Isovist::kDeltaAvg));
        stdDeltaAcc(std::pow(isovist.scalar(utils::Isovist::kDeltaStd), 2.0));
        minDistAcc(isovist.scalar(utils::Isovist::kDmin));
        avgDistAcc(isovist.scalar(utils::Isovist::kDavg));
        stdDistAcc(std::pow(isovist.scalar(utils::Isovist::kDstd), 2.0));
        areaAcc(isovist.scalar(utils::Isovist::kArea));
        perimAcc(isovist.scalar(utils::Isovist::kPerimeter));
        areaRatioAcc(isovist.scalar(utils::Isovist::kArea) / area);
        perimRatioAcc(isovist.scalar(utils::Isovist::kPerimeter) / perimeter);
        avgDistFromCenterAcc(isovist.scalar(utils::Isovist::kShapeDistAvg));
        stdDistFromCenterAcc(std::pow(isovist.scalar(utils::Isovist::kShapeDistStd), 2.0));
        minDistAngleAcc(isovist.scalar(utils::Isovist::kAngleBetweenMinDists));
        areaBalanceAcc(isovist.scalar(utils::Isovist::kAreaBalance));

        singleFeatures[kSingleEccentricity] = isovist.scalar(utils::Isovist::kShapeEccentricity);
        singleFeatures[kSingleCircularity] = isovist.scalar(utils::Isovist::kCircularity);
        singleFeatures[kSingleWaviness] = isovist.scalar(utils::Isovist::kShapeWaviness);
        singleFeatures[kSingleCompactness] = isovist.scalar(utils::Isovist::kShapeCompactness);
        singleFeatures[kSingleRayCompactness] = isovist.scalar(utils::Isovist::kRayCompactness);
        singleFeatures[kSingleShapeDistCompactness] = isovist.scalar(utils::Isovist::kShapeDistCompactness);
        //         singleFeatures[kSingleDistMin] = isovist.scalar(utils::Isovist::kDmin);
        singleFeatures[kSingleDistStd] = isovist.scalar(utils::Isovist::kDstd);
        singleFeatures[kSingleDistMean] = isovist.scalar(utils::Isovist::kDavg);
        //         singleFeatures[kSingleDVariation] = isovist.scalar(utils::Isovist::kDVariation);
        singleFeatures[kSingleDistFromCenterMean] = isovist.scalar(utils::Isovist::kShapeDistAvg);
        singleFeatures[kSingleDistFromCenterStd] = isovist.scalar(utils::Isovist::kShapeDistStd);
        //         singleFeatures[kSingleDistFromCenterVariation] = isovist.scalar(utils::Isovist::kShapeDistVariation);
        //         singleFeatures[kSingleDeltaVariation] = isovist.scalar(utils::Isovist::kDeltaVariation);
        singleFeatures[kSingleAvgRelation] = isovist.scalar(utils::Isovist::kDistRelationAvg);
        singleFeatures[kSingleStdRelation] = isovist.scalar(utils::Isovist::kDistRelationStd);
        singleFeatures[kSingleAvgDelta] = isovist.scalar(utils::Isovist::kDeltaAvg);
        singleFeatures[kSingleStdDelta] = isovist.scalar(utils::Isovist::kDeltaStd);
        //         singleFeatures[kSingleDistSkewness] = isovist.scalar(utils::Isovist::kDskewness);
        singleFeatures[kSingleArea] = isovist.scalar(utils::Isovist::kArea);
        singleFeatures[kSinglePerimeter] = isovist.scalar(utils::Isovist::kPerimeter);
        singleFeatures[kSingleAngleBetweenMinDists] = isovist.scalar(utils::Isovist::kAngleBetweenMinDists);
        //         singleFeatures[kSingleMinHalfArea] = isovist.scalar(utils::Isovist::kMinHalfArea);
        //         singleFeatures[kSingleAreaBalance] = isovist.scalar(utils::Isovist::kAreaBalance);

        //         for(int n = 0; n < numIsovistMoments; ++n)
        //         {
        //             singleFeatures[kNumSingle + n] =
        //             isovist.scalar(static_cast<utils::Isovist::Scalar>(utils::Isovist::kNumScalars + n));
        //         }

        singleFeatures[kSingleArea] = isovist.scalar(utils::Isovist::kArea);
        singleFeatures[kSinglePerimeter] = isovist.scalar(utils::Isovist::kPerimeter);
        isovistFeatures_.col(isovistCol++) = singleFeatures;
    }

    auto numCells = std::distance(hypothesis.beginSkeleton(), hypothesis.endSkeleton());
    assert(numCells > 1);

    //     features_[kIsovistMeanArea] = mean(areaAcc);
    //     features_[kIsovistStdArea] = std::sqrt(variance(areaAcc));
    features_[kIsovistVariationArea] = (mean(areaAcc) > 0.0) ? std::sqrt(variance(areaAcc)) / mean(areaAcc) : 1e7;
    //     features_[kMeanAreaComp] = (max(areaAcc) > 0.0) ? mean(areaAcc) / max(areaAcc) : 1e7;
    //     features_[kStdAreaComp] = std::sqrt(variance(areaRatioAcc));

    //     features_[kIsovistMeanPerimeter] = mean(perimAcc);
    //     features_[kIsovistStdPerimeter] = std::sqrt(variance(perimAcc));
    features_[kIsovistVariationPerimeter] =
      (mean(perimAcc) > 0.0) ? std::sqrt(variance(perimAcc)) / mean(perimAcc) : 1e7;
    //     features_[kMeanPerimeterComp] = (max(perimAcc) > 0.0) ? mean(perimAcc) / max(perimAcc) : 1e7;
    //     features_[kStdPerimeterComp] = std::sqrt(variance(perimRatioAcc));

    features_[kIsovistMeanEccentricity] = mean(eccentricityAcc);
    features_[kIsovistStdEccentricity] = std::sqrt(variance(eccentricityAcc));

    features_[kIsovistMeanCircularity] = mean(circularityAcc);
    features_[kIsovistStdCircularity] = std::sqrt(variance(circularityAcc));

    features_[kIsovistMeanWaviness] = mean(wavinessAcc);
    features_[kIsovistStdWaviness] = std::sqrt(variance(wavinessAcc));

    features_[kIsovistMeanCompactness] = mean(compactnessAcc);
    features_[kIsovistStdCompactness] = std::sqrt(variance(compactnessAcc));

    features_[kIsovistMeanRayCompactness] = mean(rayCompactnessAcc);
    features_[kIsovistStdRayCompactness] = std::sqrt(variance(rayCompactnessAcc));

    features_[kIsovistMeanShapeDistCompactness] = mean(shapeDistCompactnessAcc);
    features_[kIsovistStdShapeDistCompactness] = std::sqrt(variance(shapeDistCompactnessAcc));

    //     features_[kIsovistMeanRelation] = mean(avgRelationAcc);
    //     features_[kIsovistStdRelation] = std::sqrt(variance(avgRelationAcc));
    features_[kIsovistVariationRelation] =
      (mean(avgRelationAcc) > 0.0) ? std::sqrt(sum(stdRelationAcc)) / mean(avgRelationAcc) : 1e7;

    //     features_[kIsovistMeanDelta] = mean(avgDeltaAcc);
    //     features_[kIsovistStdDelta] = std::sqrt(variance(avgDeltaAcc));
    features_[kIsovistVariationDelta] =
      (mean(avgDeltaAcc) > 0.0) ? std::sqrt(sum(stdDeltaAcc)) / mean(avgDeltaAcc) : 1e7;

    //     features_[kIsovistDistMinMean] = mean(minDistAcc);
    //     features_[kIsovistDistMinStd] = std::sqrt(variance(minDistAcc));
    //     features_[kIsovistDistMinVariation] = (mean(minDistAcc) > 0.0) ?
    //         std::sqrt(variance(minDistAcc)) / mean(minDistAcc) : 1e7;

    //     features_[kIsovistDistMean] = mean(avgDistAcc);
    //     features_[kIsovistDistStd] = std::sqrt(sum(stdDistAcc));
    features_[kIsovistDistVariation] = (mean(avgDistAcc) > 0.0) ? std::sqrt(sum(stdDistAcc)) / mean(avgDistAcc) : 1e7;

    //     features_[kIsovistDistFromCenterMean] = mean(avgDistFromCenterAcc);
    //     features_[kIsovistDistFromCenterStd] = std::sqrt(sum(stdDistFromCenterAcc));
    features_[kIsovistDistFromCenterVariation] =
      (mean(avgDistFromCenterAcc) > 0.0) ? std::sqrt(sum(stdDistFromCenterAcc)) / mean(avgDistFromCenterAcc) : 1e7;

    features_[kIsovistAngleBetweenMinDistsMean] = mean(minDistAngleAcc);
    features_[kIsovistAngleBetweenMinDistsStd] = std::sqrt(variance(minDistAngleAcc));

    //     features_[kIsovistAreaBalanceMean] = mean(areaBalanceAcc);
    //     features_[kIsovistAreaBalanceStd] = std::sqrt(variance(areaBalanceAcc));
}


void HypothesisFeatures::calculateZernikeFeature(const AreaExtent& extent)
{
    if (extent.area() > 0.0) {
        auto boundary = extent.rectangleBoundary(math::ReferenceFrame::GLOBAL);
        auto zernikeFeatures = zernike.moments<double>(extent.begin(),
                                                       extent.end(),
                                                       extent.center().toPoint(),
                                                       std::max(boundary.width(), boundary.height()));
        std::copy(zernikeFeatures.begin(), zernikeFeatures.end(), features_.begin() + kNumFixedFeatures);
    } else {
        std::fill(features_.begin() + kNumFixedFeatures, features_.end(), 0.0);
    }
}


void HypothesisFeatures::calculateVisGraphFeatures(const AreaHypothesis& hypothesis,
                                                   const utils::VisibilityGraph& visGraph)
{
    auto clusterCoeffFeature = visGraph.calculateFeature(utils::VisibilityGraphFeatureType::clustering_coeff);
    auto clusterCoeffStats = clusterCoeffFeature.stats(hypothesis.beginVisVertices(), hypothesis.endVisVertices());
    features_[kVisGraphMeanClusterCoeff] = clusterCoeffStats.mean;
    features_[kVisGraphStdClusterCoeff] = clusterCoeffStats.std;

    auto betweennessFeature = visGraph.calculateFeature(utils::VisibilityGraphFeatureType::betweenness_centrality);
    auto betweennessStats = betweennessFeature.stats(hypothesis.beginVisVertices(), hypothesis.endVisVertices());
    features_[kVisGraphMeanBetweenness] = betweennessStats.mean;
    features_[kVisGraphStdBetweenness] = betweennessStats.std;

    auto pagerankFeature = visGraph.calculateFeature(utils::VisibilityGraphFeatureType::pagerank);
    auto pagerankStats = pagerankFeature.stats(hypothesis.beginVisVertices(), hypothesis.endVisVertices());
    features_[kVisGraphMeanPageRank] = pagerankStats.mean;
    features_[kVisGraphStdPageRank] = pagerankStats.std;
}


void HypothesisFeatures::calculateSkeletonGraphFeatures(const AreaHypothesis& hypothesis,
                                                        const utils::VisibilityGraph& skeletonGraph)
{
    auto betweennessFeature = skeletonGraph.calculateFeature(utils::VisibilityGraphFeatureType::betweenness_centrality);
    auto betweennessStats = betweennessFeature.stats(hypothesis.beginGraphVertices(), hypothesis.endGraphVertices());
    features_[kSkeletonMeanBetweenness] = betweennessStats.mean;
    features_[kSkeletonStdBetweenness] = betweennessStats.std;
    features_[kSkeletonMaxBetweenness] = betweennessStats.max;
}


HypothesisFeatures* find_cached_features(const AreaHypothesis& hyp)
{
    auto cacheIt = kFeaturesCache.find(hyp.numBoundaries());

    if (cacheIt == kFeaturesCache.end()) {
        return nullptr;
    }

    for (auto& cached : cacheIt->second) {
        bool matchesCache = true;
        for (auto& boundary : boost::make_iterator_range(hyp.beginBoundary(), hyp.endBoundary())) {
            if (std::find(cached.boundaryCells.begin(),
                          cached.boundaryCells.end(),
                          boundary->getGateway().skeletonCell())
                == cached.boundaryCells.end()) {
                matchesCache = false;
                break;
            }
        }

        if (matchesCache) {
            return &(cached.features);
        }
    }

    return nullptr;
}


void add_features_to_cache(const AreaHypothesis& hyp, const HypothesisFeatures& features)
{
    CachedFeatures cached;
    cached.features = features;
    cached.boundaryCells.reserve(hyp.numBoundaries());

    for (auto& boundary : boost::make_iterator_range(hyp.beginBoundary(), hyp.endBoundary())) {
        cached.boundaryCells.push_back(boundary->getGateway().skeletonCell());
    }

    kFeaturesCache[cached.boundaryCells.size()].push_back(std::move(cached));
}

}   // namespace hssh
}   // namespace vulcan
