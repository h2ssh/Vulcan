/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_similarity_star_builder.h
 * \author   Collin Johnson
 *
 * Declaration of PathSimilarityStarBuilder.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_PATH_SIMILARITY_STAR_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_PATH_SIMILARITY_STAR_BUILDER_H

#include "core/line.h"
#include "hssh/local_topological/area_detection/labeling/small_scale_star_builder.h"
#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/params.h"
#include <map>
#include <vector>

namespace vulcan
{
namespace math
{
template <typename T>
class Rectangle;
}
namespace hssh
{

struct local_path_fragment_t;

const std::string PATH_SIMILARITY_STAR_BUILDER_TYPE("path_similarity");

/**
 * PathSimilarityStarBuilder builds a small-scale star using a path similarity metric
 * to determine the gateways in a path. The path similarity metric uses angular similarity
 * and the distance between the center lines of the gateways. The similarity formula, given
 * two gateways A and B, is:
 *
 *   sim = -cos(theta_A - theta_B) * (1 - minCenterLineDist / diameter_place)
 *
 * Each gateway direction points into the center of the place. The centerline for the gateway is
 * oriented in the gateway direction.
 *
 * The angle metric is chosen because the two fragments for a path should be oriented in opposite
 * directions. Orthogonal gateways will have an angular sum around pi/2, thus cos() ~ 0. If two gateways
 * are oriented in the same direction, they'll have a negative similarity value.
 *
 * The distance metric provides a measure of how close the gateways are relative to the overall size
 * of the place. Each gateway has a centerline. The closeness of these center lines to the center lines
 * of the other gateway on the path gives an indication of how close they are.
 *
 */
class PathSimilarityStarBuilder : public SmallScaleStarBuilder
{
public:
    /**
     * Constructor for PathSimilarityStarBuilder.
     *
     * \param    params          Parameters controlling the stars being built
     */
    PathSimilarityStarBuilder(const path_similarity_star_builder_params_t& params);

    // SmallScaleStarBuilder interface
    SmallScaleStar buildStar(const std::vector<Gateway>& gateways) const override;
    SmallScaleStar buildStar(const std::vector<Gateway>& gateways,
                             const Point<float>& center,
                             const math::Rectangle<float>& boundary) const override;
    int numGatewaysAlignedToAxis(const std::vector<Gateway>& gateways, double axisDirection) const override;
    bool areGatewaysAligned(const Gateway& lhs, const Gateway& rhs, const Point<float>& center) const override;

private:
    struct similarities_t
    {
        std::vector<float> similarityScores;
        int mostSimilarGateway;
    };

    mutable std::vector<similarities_t> statistics_;

    mutable const std::vector<Gateway>* gateways_;
    mutable Point<float> center_;
    mutable math::Rectangle<float> boundary_;

    path_similarity_star_builder_params_t params;

    void calculateSimilarityScores(std::size_t gatewayIndex) const;
    float angleSimilarity(const Gateway& source, const Gateway& potential) const;
    float distanceSimilarity(const Gateway& source, const Gateway& potential) const;
    void findMostSimilarGateways(void) const;
    int mostSimilar(std::size_t index) const;   // -1 = ambiguous
    SmallScaleStar establishPaths(void) const;

    void addSingleFragmentPath(int pathId, const Gateway& gateway, std::vector<local_path_fragment_t>& fragments) const;
    void addTwoFragmentPath(int pathId,
                            const Gateway& gateway,
                            const Gateway& otherGateway,
                            std::vector<local_path_fragment_t>& fragments) const;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_PATH_SIMILARITY_STAR_BUILDER_H
