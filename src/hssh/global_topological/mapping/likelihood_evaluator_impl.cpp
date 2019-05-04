/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     likelihood_evaluator_impl.cpp
* \author   Collin Johnson
*
* Definition of the various likelihood evaluator subclasses and the create_topo_map_likelihood_evaluator() factory.
*/

#include <hssh/global_topological/mapping/likelihood_evaluator_impl.h>
#include <hssh/local_topological/areas/place.h>
#include <hssh/global_topological/state.h>
#include <hssh/global_topological/utils/metric_map_cache.h>
#include <iostream>

// #define DEBUG_COMPATIBILITY
// #define DEBUG_COMPATIBILITY_OVERLAP
// #define DEBUG_CHI_LAMBDAS

namespace vulcan
{
namespace hssh
{

double calculate_place_compatibility(const pose_t& poseA,
                                     const pose_t& poseB,
                                     const CachedMap& placeA,
                                     const CachedMap& placeB);
void   transform_rectangle(math::Rectangle<float>& rect, const pose_t& center);  // move a rectangle to be centered around a new position

////////////////////////////////////// EdgeLengthEvaluator definition ////////////////////////////////////////
EdgeLengthEvaluator::EdgeLengthEvaluator(const edge_length_evaluator_params_t& params)
                                    : params(params)
{
}


double EdgeLengthEvaluator::calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places)
{
    return 0.0;
}

////////////////////////////////////// LPMMatchEvaluator definition ////////////////////////////////////////
LPMMatchEvaluator::LPMMatchEvaluator(const lpm_match_evaluator_params_t& params)
                                : params(params)
{

}


double LPMMatchEvaluator::calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places)
{
    return 0.0;
}

////////////////////////////////////// PlaceLayoutConformityEvaluator definition ////////////////////////////////////////
PlaceLayoutCompatibilityEvaluator::PlaceLayoutCompatibilityEvaluator(const place_layout_compatibility_evaluator_params_t& params)
                                                                : params(params)
{
}


double PlaceLayoutCompatibilityEvaluator::calculateLogLikelihood(const TopologicalState& state,
                                                                 const MetricMapCache& places)
{
    const auto& mapPlaces = state.map->places();

    double overallLogLikelihood = 0.0;
    double compatibility        = 1.0;

    for(auto placeIt = mapPlaces.begin(), placeEnd = mapPlaces.end(); placeIt != placeEnd; ++placeIt)
    {
        auto otherPlaceIt = placeIt;
        ++otherPlaceIt;

        auto place = places.loadMap(placeIt->second->metricPlaceId());
        auto placePose = state.map->referenceFrame(placeIt->first);

        // If the place doesn't exist, don't process the compatibility.
        if(!place)
        {
            continue;
        }

        for(; otherPlaceIt != placeEnd; ++otherPlaceIt)
        {
            auto otherPlace = places.loadMap(otherPlaceIt->second->metricPlaceId());
            if(otherPlace)
            {
                compatibility = calculate_place_compatibility(placePose,
                                                              state.map->referenceFrame(otherPlaceIt->first),
                                                              *place,
                                                              *otherPlace
                                                              );
                overallLogLikelihood += compatibility;
            }
        }
    }

#ifdef DEBUG_COMPATIBILITY
    std::cout<<"DEBUG:CompatibilityLikelihood:"<<overallLogLikelihood<<'\n';
#endif

    return overallLogLikelihood;

}

////////////////////////////////////// ChiLikelihoodEvaluator definition ////////////////////////////////////////
ChiLikelihoodEvaluator::ChiLikelihoodEvaluator(const chi_likelihood_evaluator_params_t& params)
                                            : params(params)
{
}


double ChiLikelihoodEvaluator::calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places)
{
#ifdef DEBUG_CHI_LAMBDAS
    std::cout<<"DEBUG:ChiLikelihood:Likelihood of lambdas:(chi)->(measured)=log likelihood\n";

    double logLikelihood = 0.0;

    for(auto idToSeg : state.map->segments())
    {
        const auto& segment = *idToSeg.second;

        // Ignore all frontiers, they don't count
        if(segment.isFrontier())
        {
            continue;
        }
        // Compute the lambda for this map from the place reference frames
        Lambda lambda(state.map->referenceFrame(segment.minusPlace().id()),
                      state.map->referenceFrame(segment.plusPlace().id()));
        std::cout << "Segment chi lambda:" << idToSeg.first << '\n' << lambda.transformDistribution.getMean()
            << lambda.transformDistribution.getCovariance();

        for(auto& meas : segment.getAllLambdas())
        {
            double measLoglihood = lambda.transformDistribution.logPosterior(meas.transformDistribution.getMean());
            std::cout << "meas lambda prob:" << measLoglihood << '\n' << meas.transformDistribution.getMean()
                << meas.transformDistribution.getCovariance();
            logLikelihood += measLoglihood;
        }
    }

    std::cout<<"DEBUG:ChiLikelihood: Chi:"<<state.map->chiLogLikelihood()<< " Meas:" << logLikelihood << '\n';
#endif

    return state.map->chiLogLikelihood();
}


double calculate_place_compatibility(const pose_t& poseA,
                                     const pose_t& poseB,
                                     const CachedMap& placeA,
                                     const CachedMap& placeB)
{
    math::Rectangle<float> boundaryA = placeA.rectangleBoundary();
    math::Rectangle<float> boundaryB = placeB.rectangleBoundary();

    transform_rectangle(boundaryA, poseA);
    transform_rectangle(boundaryB, poseB);

#ifdef DEBUG_COMPATIBILITY_OVERLAP
    if(boundaryA.overlap(boundaryB) > 0)
    {
        std::cout<<"DEBUG:PlaceCompatibility:poseA:"<<poseA<<" poseB:"<<poseB<<'\n';
        std::cout<<"DEBUG:PlaceCompatibility:BoundaryA:"<<boundaryA<<" BoundaryB:"<<boundaryB<<" Overlap:"<<boundaryA.overlap(boundaryB)<<'\n';
    }
#endif

    double overlap = boundaryA.overlap(boundaryB);
    return -10*overlap;
}


void transform_rectangle(math::Rectangle<float>& rect, const pose_t& center)
{
    rect.bottomLeft.x  += center.x;
    rect.bottomRight.x += center.x;
    rect.topLeft.x     += center.x;
    rect.topRight.x    += center.x;

    rect.bottomLeft.y  += center.y;
    rect.bottomRight.y += center.y;
    rect.topLeft.y     += center.y;
    rect.topRight.y    += center.y;
}

} // namespace hssh
} // namespace vulcan
