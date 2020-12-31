/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_similarity_star_builder.cpp
 * \author   Collin Johnson
 *
 * Definition of PathSimilarityStarBuilder.
 */

#include "hssh/local_topological/area_detection/labeling/path_similarity_star_builder.h"
#include "hssh/local_topological/place_extent.h"
#include "hssh/local_topological/small_scale_star.h"
#include "math/geometry/rectangle.h"
#include "math/geometry/shape_fitting.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <set>

// #define DEBUG_ANGLE_SIMILARITY
// #define DEBUG_DIST_SIMILARITY
// #define DEBUG_MOST_SIMILAR
// #define DEBUG_PATH_MATCHES

namespace vulcan
{
namespace hssh
{

math::Rectangle<float> boundary_around_gateways(const std::vector<Gateway>& gateways);
double distance_similarity_score(const Gateway& gateway, double direction);
double direction_towards_center(const Gateway& gateway, const Point<float>& center);
local_path_fragment_t create_path_fragment(int index, float direction, const Gateway& gateway);


PathSimilarityStarBuilder::PathSimilarityStarBuilder(const path_similarity_star_builder_params_t& params)
: params(params)
{
}


SmallScaleStar PathSimilarityStarBuilder::buildStar(const std::vector<Gateway>& gateways) const
{
    // If there aren't any gateways, then there can't be a star!
    if (gateways.empty()) {
        return SmallScaleStar();
    }

    auto boundary = boundary_around_gateways(gateways);
    return buildStar(gateways, boundary.center(), boundary);
}


SmallScaleStar PathSimilarityStarBuilder::buildStar(const std::vector<Gateway>& gateways,
                                                    const Point<float>& center,
                                                    const math::Rectangle<float>& boundary) const
{
    // If there aren't any gateways, then there can't be a star!
    if (gateways.empty()) {
        return SmallScaleStar();
    }

#if defined(DEBUG_ANGLE_SIMILARITY) || defined(DEBUG_DIST_SIMILARITY) || defined(DEBUG_MOST_SIMILAR)
    std::cout << "DEBUG:PathSimilarityStarBuilder: Gateways for star: (id, center)\n";
    for (auto& g : gateways) {
        std::cout << g.id() << ',' << g.center() << '\n';
    }
#endif

    statistics_.clear();
    statistics_.resize(gateways.size());

    gateways_ = &gateways;
    center_ = center;
    boundary_ = boundary;

    for (std::size_t n = 0; n < gateways_->size(); ++n) {
        calculateSimilarityScores(n);
    }

    findMostSimilarGateways();
    return establishPaths();
}


int PathSimilarityStarBuilder::numGatewaysAlignedToAxis(const std::vector<Gateway>& gateways,
                                                        double axisDirection) const
{
    // Need at least two gateways for them to be aligned
    if (gateways.size() < 2) {
        return 0;
    }

    const float kMaxAlignedDiff = M_PI / 4.0;

    int numAligned = 0;

    auto boundary = boundary_around_gateways(gateways);

    // For each gateway, find distance similarity to all other gateways
    for (std::size_t n = 0; n < gateways.size(); ++n) {
        // Can't be algined if the gateway direction is too far from the axis
        if (angle_diff_abs_pi_2(gateways[n].direction(), axisDirection) > kMaxAlignedDiff) {
            continue;
        }

        // The normal line starts at this gateway and runs along the axis
        Line<float> normal{gateways[n].center(),
                           Point<float>(gateways[n].center().x + std::cos(axisDirection),
                                        gateways[n].center().y + std::sin(axisDirection))};

        for (std::size_t i = n + 1; i < gateways.size(); ++i) {
            if (angle_diff_abs_pi_2(gateways[i].direction(), axisDirection) > kMaxAlignedDiff) {
                continue;
            }

            // If the similarity is less than the required value for two gateways being on the same fragment
            // then the gateways cannot be aligned to the same axis
            float direction = direction_towards_center(gateways[i], boundary.center());
            float angleSimilarity = std::abs(std::cos(axisDirection - direction));
            float distSimilarity = distance_similarity_score(gateways[i], axisDirection);

            // These two gateways are aligned to the axis
            if (angleSimilarity * distSimilarity >= params.minimumPathSimilarity) {
                numAligned += 2;
            }
        }
    }

    return numAligned;
}


bool PathSimilarityStarBuilder::areGatewaysAligned(const Gateway& lhs,
                                                   const Gateway& rhs,
                                                   const Point<float>& center) const
{
    center_ = lhs.center() + rhs.center();
    center_.x /= 2;
    center_.y /= 2;

    double similarity = angleSimilarity(lhs, rhs) * distanceSimilarity(lhs, rhs);
    return similarity >= params.minimumPathSimilarity;
}


void PathSimilarityStarBuilder::calculateSimilarityScores(std::size_t gatewayIndex) const
{
    auto& similarityScores = statistics_[gatewayIndex].similarityScores;

    similarityScores.resize(gateways_->size());
    similarityScores[gatewayIndex] = -100000.0f;

    for (std::size_t n = 0; n < gateways_->size(); ++n) {
        if (n != gatewayIndex) {
            similarityScores[n] = angleSimilarity(gateways_->at(gatewayIndex), gateways_->at(n))
              * distanceSimilarity(gateways_->at(gatewayIndex), gateways_->at(n));
        }
    }
}


float PathSimilarityStarBuilder::angleSimilarity(const Gateway& source, const Gateway& potential) const
{
    auto sourceDirection = direction_towards_center(source, center_);
    auto potentialDirection = direction_towards_center(potential, center_);

#ifdef DEBUG_ANGLE_SIMILARITY
    std::cout << "DEBUG:angle_sim:" << source.id() << "->" << potential.id() << ':'
              << -cos(sourceDirection - potentialDirection) << " dirs:(" << sourceDirection << ',' << potentialDirection
              << ")\n";
#endif

    return -std::cos(sourceDirection - potentialDirection);
}


float PathSimilarityStarBuilder::distanceSimilarity(const Gateway& source, const Gateway& potential) const
{
    float similarity = distance_similarity_score(potential, direction_towards_center(source, boundary_.center()));

#ifdef DEBUG_DIST_SIMILARITY
    std::cout << "DEBUG:dist_sim:" << source.id() << "->" << potential.id() << ':' << similarity << '\n';
#endif

    return similarity;
}


void PathSimilarityStarBuilder::findMostSimilarGateways(void) const
{
    for (std::size_t n = 0; n < statistics_.size(); ++n) {
        statistics_[n].mostSimilarGateway = mostSimilar(n);
    }
}


int PathSimilarityStarBuilder::mostSimilar(std::size_t index) const
{
    const auto& scores = statistics_[index].similarityScores;

    // An ambiguous score occurs when the max score is less than some percentage greater than the next closest score
    float bestScore = 0;
    int bestId = -1;
    bool isAmbiguous = false;

#ifdef DEBUG_MOST_SIMILAR
    std::cout << "DEBUG:similarity:source " << gateways_->at(index).id() << ": ";
#endif

    for (std::size_t n = 0; n < scores.size(); ++n) {
        if (scores[n] > bestScore) {
            isAmbiguous = scores[n] * params.ambiguousScorePercent < bestScore;
            bestId = n;
            bestScore = scores[n];
        }

#ifdef DEBUG_MOST_SIMILAR
        std::cout << n << ':' << scores[n] << ' ';
#endif
    }

    int mostSimilarId = (isAmbiguous || bestScore < params.minimumPathSimilarity) ? -1 : bestId;

#ifdef DEBUG_MOST_SIMILAR
    std::cout << "amb:" << isAmbiguous << " Most similar:" << mostSimilarId << " Score:" << bestScore << '\n';
#endif

    return mostSimilarId;
}


SmallScaleStar PathSimilarityStarBuilder::establishPaths(void) const
{
    std::vector<local_path_fragment_t> fragments;
    std::vector<bool> inPath(gateways_->size(), false);
    int8_t pathIndex = 0;

    std::vector<int> gatewayOrder(gateways_->size());
    std::iota(gatewayOrder.begin(), gatewayOrder.end(), 0);
    std::sort(gatewayOrder.begin(), gatewayOrder.end(), [this](int lhsIdx, int rhsIdx) {
        return *std::max_element(statistics_[lhsIdx].similarityScores.begin(),
                                 statistics_[lhsIdx].similarityScores.end())
          > *std::max_element(statistics_[rhsIdx].similarityScores.begin(), statistics_[rhsIdx].similarityScores.end());
    });

    // Use two passes for now, establishing the two-fragment gateways and then the one fragment
    for (int n = 0, end = gatewayOrder.size(); n < end; ++n) {
        // If the gateway has already been added to a path, then ignore it
        int index = gatewayOrder[n];
        if (inPath[index]) {
            continue;
        }

        if (statistics_[index].mostSimilarGateway != -1) {
            int mostSimilarIndex = statistics_[index].mostSimilarGateway;

            // Only create a two fragment path if there is an unambiguous match between the two gateway, i.e. they point
            // to each other for their most similar gateways
            if (!inPath[mostSimilarIndex] && (statistics_[mostSimilarIndex].mostSimilarGateway == index))
            //             if(!inPath[mostSimilarIndex])
            {
                addTwoFragmentPath(pathIndex++, gateways_->at(index), gateways_->at(mostSimilarIndex), fragments);

                inPath[mostSimilarIndex] = true;
                inPath[index] = true;
            }
        }
    }

    for (int n = 0, end = gatewayOrder.size(); n < end; ++n) {
        int index = gatewayOrder[n];
        // If the gateway has already been added to a path, then ignore it
        if (!inPath[index]) {
            // Any remaining gateways didn't match anything in the two fragment case. Create the single gateway fragment
            addSingleFragmentPath(pathIndex++, gateways_->at(index), fragments);
        }
    }

    // Sort the fragments counter-clockwise. Select the fragment with angle closest to zero as the
    // starting point to keep the star stable across updates
    auto minIt = std::min_element(fragments.begin(), fragments.end(), [](auto& lhs, auto& rhs) {
        return std::abs(lhs.gateway.direction()) < std::abs(rhs.gateway.direction());
    });

    float sortAngle = minIt->gateway.direction();

    std::sort(fragments.begin(),
              fragments.end(),
              [sortAngle](const local_path_fragment_t& lhs, const local_path_fragment_t& rhs) {
                  return wrap_to_2pi(lhs.gateway.direction() - sortAngle)
                    < wrap_to_2pi(rhs.gateway.direction() - sortAngle);
              });

    assert(fragments[0].gateway.direction() == sortAngle);

#ifdef DEBUG_PATH_MATCHES
    std::cout << "DEBUG:establishPaths:final fragment order:\n";
    for (auto fragIt = fragments.begin(); fragIt != fragments.end(); ++fragIt) {
        std::cout << "id:" << (int)fragIt->pathId << " dir:" << fragIt->direction << " nav:" << fragIt->navigable
                  << " orient:" << fragIt->gateway.direction() << " gateway:" << fragIt->gateway.boundary() << '\n';
    }
#endif

    return SmallScaleStar(fragments);
}


void PathSimilarityStarBuilder::addSingleFragmentPath(int pathId,
                                                      const Gateway& gateway,
                                                      std::vector<local_path_fragment_t>& fragments) const
{
    local_path_fragment_t gatewayFrag =
      create_path_fragment(pathId, angle_to_point(boundary_.center(), gateway.center()), gateway);
    gatewayFrag.navigable = true;
    fragments.push_back(gatewayFrag);

    local_path_fragment_t otherFrag =
      create_path_fragment(pathId, angle_to_point(gateway.center(), boundary_.center()), gateway);
    otherFrag.navigable = false;
    otherFrag.direction = (gatewayFrag.direction == PATH_FRAGMENT_PLUS) ? PATH_FRAGMENT_MINUS : PATH_FRAGMENT_PLUS;
    fragments.push_back(otherFrag);
}


void PathSimilarityStarBuilder::addTwoFragmentPath(int pathId,
                                                   const Gateway& gateway,
                                                   const Gateway& otherGateway,
                                                   std::vector<local_path_fragment_t>& fragments) const
{
    local_path_fragment_t gatewayFrag =
      create_path_fragment(pathId, angle_to_point(otherGateway.center(), gateway.center()), gateway);
    gatewayFrag.navigable = true;
    fragments.push_back(gatewayFrag);

    local_path_fragment_t otherFrag =
      create_path_fragment(pathId, angle_to_point(gateway.center(), otherGateway.center()), otherGateway);
    otherFrag.navigable = true;
    assert(otherFrag.direction != gatewayFrag.direction);
    fragments.push_back(otherFrag);
}


math::Rectangle<float> boundary_around_gateways(const std::vector<Gateway>& gateways)
{
    std::vector<Point<float>> points;

    for (auto& gateway : gateways) {
        points.push_back(gateway.boundary().a);
        points.push_back(gateway.boundary().b);
    }

    return math::axis_aligned_bounding_rectangle<float>(points.cbegin(), points.cend());
}


double distance_similarity_score(const Gateway& gateway, double direction)
{
    Line<double> normal(
      gateway.center(),
      Point<double>(gateway.center().x + std::cos(direction), gateway.center().y + std::sin(direction)));

    double similarity = 0.0;

    Point<double> intersectionPoint;

    if (line_intersection_point(gateway.boundary(), normal, intersectionPoint)) {
        float normalizedIntersection = distance_between_points(gateway.center(), intersectionPoint) / gateway.length();
        similarity = std::max(0.0f, 1.0f - (normalizedIntersection / 2.0f));
    }

    return similarity;
}


double direction_towards_center(const Gateway& gateway, const Point<float>& center)
{
    double angleToCenter = angle_to_point(gateway.center(), center);
    double leftDiff = angle_diff_abs(gateway.leftDirection(), angleToCenter);
    double rightDiff = angle_diff_abs(gateway.rightDirection(), angleToCenter);
    return (leftDiff < rightDiff) ? gateway.leftDirection() : gateway.rightDirection();
}


local_path_fragment_t create_path_fragment(int index, float direction, const Gateway& gateway)
{
    local_path_fragment_t fragment;
    fragment.pathId = index;

    // If the direction to the fragment is in quadrants I or IV, then the direction is positive
    fragment.direction = (direction >= 0 && (direction < M_PI)) ? PATH_FRAGMENT_PLUS : PATH_FRAGMENT_MINUS;
    fragment.gateway = gateway;

    // The gateway direction is away from the center, which corresponds to the provided direction, as it is the
    // direction from one gateway to the other (or center to the gateway)
    if (angle_diff_abs(direction, gateway.rightDirection()) < angle_diff_abs(direction, gateway.leftDirection())) {
        fragment.gateway.reverseDirections();
    }

    return fragment;
}

}   // namespace hssh
}   // namespace vulcan
