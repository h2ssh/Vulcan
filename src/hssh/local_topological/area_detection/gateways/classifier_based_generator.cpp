/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     classifier_based_generator.cpp
 * \author   Collin Johnson
 *
 * Definition of ClassifierBasedGenerator.
 */

#include "hssh/local_topological/area_detection/gateways/classifier_based_generator.h"
#include "hssh/local_topological/area_detection/gateways/feature_extraction.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_edges.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/params.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"

namespace vulcan
{
namespace hssh
{

const double kProbThresh = 0.15;


struct ClassifierBasedGenerator::Impl
{
    int32_t nextId_ = 0;
    int featureRadius_;
    GatewayClassifier classifier_;
    std::vector<Line<int>> possibleBoundaries_;
    std::vector<double> edgeProb_;   // cache data about gateways along an edge
    std::vector<int> edgeIdx_;
    std::vector<int> regionIdx_;
    CellVector regionCells_;
    std::vector<double> regionProbs_;
    std::vector<WeightedGateway> regionGateways_;
    CellToIntMap priorCells_;
    const std::vector<WeightedGateway>* priorGateways_;

    const std::size_t kRegionWidth_;

    Impl(int featureRadius, const std::string& classifierFilename)
    : featureRadius_(featureRadius)
    , classifier_(classifierFilename)
    , kRegionWidth_((featureRadius_ * 2) + 1)
    {
        classifier_.setThreshold(kProbThresh);
    }


    std::vector<WeightedGateway> generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                  const VoronoiIsovistField& isovists,
                                                  const VoronoiSkeletonGrid& skeleton,
                                                  const EndpointValidator& validator)
    {
        VoronoiEdges edges(skeleton, SKELETON_CELL_REDUCED_SKELETON);
        auto features = extract_gateway_features_default(edges, isovists);   //, featureRadius_);

        std::vector<WeightedGateway> gateways;
        priorCells_.clear();
        priorGateways_ = &priorGateways;
        for (std::size_t n = 0; n < priorGateways.size(); ++n) {
            const auto& gwy = priorGateways[n];
            // Always save the transition gateway
            if (gwy.isTransition) {
                gateways.push_back(gwy);
            } else {
                priorCells_[gwy.gateway.skeletonCell()] = n;
            }
        }

        for (auto& e : edges) {
            createGatewaysAlongEdge(e, features, skeleton, isovists, gateways);
        }

        return gateways;
    }


    void createGatewaysAlongEdge(const CellVector& edge,
                                 const SkeletonFeatures& features,
                                 const VoronoiSkeletonGrid& skeleton,
                                 const VoronoiIsovistField& isovists,
                                 std::vector<WeightedGateway>& gateways)
    {
        // Need at least two cells for algorithm to work
        if (edge.size() < 2) {
            return;
        }

        bool debug = false;   // utils::contains(edge, cell_t(2060, 347));

        computeProbabilitiesAlongEdge(edge, features);

        const double kHighProb = 0.5;
        const int kSentinel = -1;
        int index = 0;
        edgeIdx_.resize(edgeProb_.size());
        std::fill(edgeIdx_.begin(), edgeIdx_.end(), kSentinel);
        // On the first pass, create the regions with p > 0.5, i.e. most likely a gateway
        for (std::size_t n = 0; n < edgeIdx_.size(); ++n) {
            // By only incrementing when not probable enough, adjacent cells with high
            // probability get clumped into the same region
            if (edgeProb_[n] > kHighProb) {
                edgeIdx_[n] = index;
            } else {
                ++index;
            }
        }

        // On the second pass, grow out the high probability regions
        for (std::size_t n = 1; n < edgeIdx_.size(); ++n) {
            // By only incrementing when not probable enough, adjacent cells with high
            // probability get clumped into the same region
            if ((edgeProb_[n] >= kProbThresh) && (edgeIdx_[n] == kSentinel) && (edgeIdx_[n - 1] != kSentinel)) {
                edgeIdx_[n] = edgeIdx_[n - 1];
            }
        }

        for (std::size_t n = edgeIdx_.size(); n > 1; --n) {
            // By only incrementing when not probable enough, adjacent cells with high
            // probability get clumped into the same region
            if ((edgeProb_[n - 2] >= kProbThresh) && (edgeIdx_[n - 2] == kSentinel) && (edgeIdx_[n - 1] != kSentinel)) {
                edgeIdx_[n - 2] = edgeIdx_[n - 1];
            }
        }

        // On the third pass, run the marking algorithm again, but only mark low probability regions that weren't
        // included in the previous sweep
        for (std::size_t n = 0; n < edgeIdx_.size(); ++n) {
            if ((edgeIdx_[n] == kSentinel) && (edgeProb_[n] <= kHighProb) && (edgeProb_[n] > kProbThresh)) {
                edgeIdx_[n] = index;
            } else {
                ++index;
            }
        }

        auto non_sentinel_func = [kSentinel](int idx) {
            return idx != kSentinel;
        };

        // Maintain three values:
        //  - startIt = start of the change region (usually a 0 prob cell half distance to previous change region)
        //  - regionStartIt = start of region with prob > 0
        //  - regionEndIt = end of region with prob > 0
        auto regionStartIt = std::find_if(edgeIdx_.begin(), edgeIdx_.end(), non_sentinel_func);
        // Are there any change regions? If not, then ignore all of them.
        if (regionStartIt == edgeIdx_.end()) {
            return;
        }

        // Now find each region where a gateway exists and create the best gateway in that region
        for (auto startIt = edgeIdx_.begin(), endIt = edgeIdx_.end(); startIt != endIt;) {
            auto regionEndIt = std::find_if(regionStartIt, endIt, [regionStartIt](int idx) {
                return idx != *regionStartIt;
            });
            auto zeroEndIt = std::find_if(regionEndIt, endIt, non_sentinel_func);

            // If the zeros go to the end of the edge, include them all
            if (zeroEndIt == endIt) {
                regionEndIt = zeroEndIt;
            }
            // Otherwise include half of them, as the other half will go to
            else {
                regionEndIt += std::distance(regionEndIt, zeroEndIt) / 2;
            }

            regionGateways_.clear();
            // Find the indices included in the gateway region
            regionIdx_.resize(std::distance(startIt, regionEndIt));
            std::iota(regionIdx_.begin(), regionIdx_.end(), std::distance(edgeIdx_.begin(), startIt));

            regionCells_.clear();
            regionProbs_.clear();
            for (auto idx : regionIdx_) {
                regionCells_.push_back(edge[idx]);
                regionProbs_.push_back(edgeProb_[idx]);
            }

            auto maxIt =
              std::max_element(edgeProb_.begin() + regionIdx_.front(), edgeProb_.begin() + regionIdx_.back() + 1);
            auto maxCell = edge[std::distance(edgeProb_.begin(), maxIt)];

            auto regionGateway = createGatewayForChangeRegion(maxCell, *maxIt, skeleton, isovists);

            if (regionGateway) {
                gateways.push_back(*regionGateway);
            }

            if (debug) {
                std::cout << "Region:" << regionCells_.front() << " -> " << regionCells_.back() << " Max: " << maxCell
                          << " prob:" << *maxIt << " Gwy? " << (regionGateway != boost::none) << '\n';
            }

            // Skip the rest of this region
            startIt = regionEndIt;
            regionStartIt = std::find_if(regionEndIt, endIt, non_sentinel_func);
        }

        if (debug) {
            std::cout << "Edge regions:\n";
            for (std::size_t n = 0; n < edgeProb_.size(); ++n) {
                std::cout << n << '\t' << edgeIdx_[n] << '\t' << edgeProb_[n] << '\n';
            }
        }
    }

    void computeProbabilitiesAlongEdge(const CellVector& edge, const SkeletonFeatures& features)
    {
        // For each cell along the edge, compute the probability of it being a gateway. If it isn't classified as
        // such then mark it as 0 probability to note regions where no gateway exists.
        edgeProb_.clear();
        for (auto& cell : edge) {
            if (features.find(cell) != features.end()) {
                auto classification = classifier_.classifyGateway(features.at(cell), GatewayClassifier::adaboost);
                edgeProb_.push_back(classification.probability);
            } else {
                edgeProb_.push_back(0.0);
            }

            // If this cell is part of a prior gateway, then set the probability of gateway to be max of the prior
            // and the current probability
            if (priorCells_.find(cell) != priorCells_.end()) {
                double gwyProb = priorGateways_->at(priorCells_[cell]).gateway.probability();
                edgeProb_.back() = std::max(edgeProb_.back(), gwyProb);
            }
        }
    }


    boost::optional<WeightedGateway> createGatewayForChangeRegion(cell_t maxCell,
                                                                  double probability,
                                                                  const VoronoiSkeletonGrid& skeleton,
                                                                  const VoronoiIsovistField& isovists)
    {
        assert(regionCells_.size() == regionProbs_.size());

        int regionId = nextId_++;

        regionGateways_.clear();

        //         double skeletonNormal = angle_sum(isovists.at(maxCell).scalar(utils::Isovist::kMinDistOrientation),
        //                                                 M_PI_2);
        //         double skeletonNormal = isovists.at(maxCell).scalar(utils::Isovist::kMinLineNormal);
        //
        //         for(std::size_t n = 0; n < regionCells_.size(); ++n)
        //         {
        //             cell_t cell = regionCells_[n];
        //             double prob = regionProbs_[n];
        //
        //             if(isovists.contains(cell))
        //             {
        //                 double normal = isovists.at(cell).scalar(utils::Isovist::kMinLineNormal);
        //                 auto gateway = create_gateway_at_cell(cell, normal, regionId, skeleton, 1.0);
        //                 if(gateway)
        //                 {
        //                     double weight = (M_PI_2 - angle_diff_abs_pi_2(gateway->direction(), skeletonNormal))
        //                         / gateway->length() * prob;
        //                     regionGateways_.emplace_back(WeightedGateway{*gateway, weight, false});
        //                 }
        //             }
        //         }
        //
        //         // Highest score wins!
        //         auto bestGatewayIt = std::max_element(regionGateways_.begin(), regionGateways_.end());
        //
        //         if(bestGatewayIt != regionGateways_.end())
        //         {
        // //             std::cout << "Created gateway for change region with max " << maxCell << " : "
        // //                 << bestGatewayIt->gateway.boundary() << '\n';
        //             bestGatewayIt->gateway.setProbability(probability);
        //             return *bestGatewayIt;
        //         }
        //
        //             return boost::none;
        //         }
        //     }

        float skeletonNormal = 0.0f;

        // Use regionCells_ to get the appropriate possibilities
        auto bestSources = skeleton.getSourceCells(maxCell.x, maxCell.y);
        auto sourceGateway = create_gateway_between_sources(bestSources, maxCell, regionId, skeleton, isovists);
        if (sourceGateway) {
            skeletonNormal = sourceGateway->direction();
            regionGateways_.emplace_back(WeightedGateway{*sourceGateway, probability, false});
        } else {
            skeletonNormal = angle_sum(isovists.at(maxCell).scalar(utils::Isovist::kMinDistOrientation), M_PI_2);
        }

        for (std::size_t n = 0; n < regionCells_.size(); ++n) {
            cell_t cell = regionCells_[n];
            double prob = regionProbs_[n];

            for (auto& source : bestSources) {
                double normal = std::atan2(cell.y - source.y, cell.x - source.x) + M_PI_2;
                auto gateway = create_gateway_at_cell(cell, normal, regionId, skeleton);
                if (gateway && ((gateway->cellBoundary().a == source) || (gateway->cellBoundary().b == source))) {
                    double weight =
                      (M_PI_2 - angle_diff_abs_pi_2(gateway->direction(), skeletonNormal)) / gateway->length() * prob;
                    regionGateways_.emplace_back(WeightedGateway{*gateway, weight, false});
                }
            }

            // If there was a prior gateway at this cell, include it in the possible gateways
            if (priorCells_.find(cell) != priorCells_.end()) {
                int idx = priorCells_[cell];
                const auto& gwy = priorGateways_->at(idx);
                double weight = (M_PI_2 - angle_diff_abs_pi_2(gwy.gateway.direction(), skeletonNormal))
                  / gwy.gateway.length() * gwy.gateway.probability();
                regionGateways_.emplace_back(WeightedGateway{gwy.gateway, weight, false});
            }
        }

        // Highest score wins!
        auto bestGatewayIt = std::max_element(regionGateways_.begin(), regionGateways_.end());

        if (bestGatewayIt != regionGateways_.end()) {
            bestGatewayIt->gateway.setProbability(probability);
            return *bestGatewayIt;
        }

        return boost::none;
    }
};


ClassifierBasedGenerator::ClassifierBasedGenerator(const classifier_based_generator_params_t& params,
                                                   const std::string& mapName)
: ClassifierBasedGenerator(params.featureRadius, mapName)
{
}


ClassifierBasedGenerator::ClassifierBasedGenerator(int radius, const std::string& classifierFilename)
: impl_(std::make_unique<Impl>(radius, classifierFilename))
{
}


ClassifierBasedGenerator::~ClassifierBasedGenerator(void)
{
    // For std::unique_ptr
}


std::vector<WeightedGateway>
  ClassifierBasedGenerator::generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                             const VoronoiIsovistField& isovists,
                                             const VoronoiSkeletonGrid& skeleton,
                                             const EndpointValidator& validator)
{
    return impl_->generateGateways(priorGateways, isovists, skeleton, validator);
}

}   // namespace hssh
}   // namespace vulcan
