/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     grid_based_observation_models.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of EndpointObservationModel and BeamObservationModel.
 */

#include "hssh/metrical/localization/grid_based_observation_models.h"
#include "core/angle_functions.h"
#include "core/line.h"
#include "core/multivariate_gaussian.h"
#include "hssh/metrical/localization/debug_info.h"
#include "hssh/metrical/localization/scan_weighting.h"
#include "hssh/metrical/occupancy_grid.h"
#include "laser/laser_scan_lines.h"
#include "laser/moving_laser_scan.h"
#include "math/coordinates.h"
#include "utils/ray_tracing.h"
#include "utils/timestamp.h"
#include <cmath>
#include <iostream>

// #define DEBUG_ENDPOINT
// #define DEBUG_COVARIANCE
// #define DEBUG_BEAM
// #define DEBUG_WEIGHTED_BEAM

namespace vulcan
{
namespace hssh
{

bool is_ray_endpoint(const OccupancyGrid& grid, Point<int> cell)
{
    return grid.getCellType(cell.x, cell.y) & (kOccupiedOccGridCell);
}


//////////////////////////    Endpoint Observation Model implementation ///////////////////////
EndpointObservationModel::EndpointObservationModel(const endpoint_observation_model_params_t& params)
: useScanWeights(params.useCovarianceWeighting)
, useMovingScan(params.useMovingScan)
, maxLaserDistance(params.maxEndpointDistance)
, stride(params.rayStride)
{
}


void EndpointObservationModel::initializeModel(const laser::laser_scan_lines_t& scan,
                                               const MultivariateGaussian& proposalDistribution)
{
    calculate_covariance_weights(scan, proposalDistribution, stride, scanWeights);

    if (!useScanWeights) {
        std::fill(scanWeights.begin(), scanWeights.end(), 1.0);
    }
}


double EndpointObservationModel::sampleLikelihood(const particle_t& sample,
                                                  const laser::laser_scan_lines_t& scan,
                                                  const OccupancyGrid& map,
                                                  particle_filter_debug_info_t* debug)
{
    const float kMinRayLength = 0.1f;

    bool shouldSaveParticleInfo = false;   // debug != nullptr;

    auto movingScan = useMovingScan ? laser::MovingLaserScan(scan.scan, sample.parent, sample.pose, stride)
                                    : laser::MovingLaserScan(scan.scan, sample.pose, sample.pose, stride);

    if (scanWeights.size() != movingScan.size()) {
        std::cerr << "ERROR:EndpointObservationModel: Scan weights not same size as num scan rays: Num: Weights:"
                  << scanWeights.size() << " Rays:" << movingScan.size() << "Raw:" << scan.scan.numRanges << '\n';
        assert(scanWeights.size() == movingScan.size());
    }

    particle_grid_score_t particleScore;
    if (shouldSaveParticleInfo) {
        particleScore.endpoints.reserve(movingScan.size());
        particleScore.scores.reserve(movingScan.size());
        particleScore.particleId = sample.id;
    }

    double scanLikelihood = 0.0;

    for (std::size_t n = 0; n < movingScan.size(); ++n) {
        const auto& ray = movingScan[n];

        if ((ray.range > maxLaserDistance) || (ray.range < kMinRayLength)) {
            continue;
        }

        Point<double> rayEnd = utils::global_point_to_grid_point(ray.endpoint, map);
        double rayLikelihood = hitLikelihood(rayEnd, map);
        scanLikelihood += rayLikelihood * scanWeights[n];

        if (shouldSaveParticleInfo) {
            particleScore.endpoints.push_back(ray.endpoint);
            //             particleScore.scores.push_back(scanWeights[n]);
            particleScore.scores.push_back(rayLikelihood);
        }
    }

    if (shouldSaveParticleInfo) {
        debug->particleScores.push_back(std::move(particleScore));
    }

    return scanLikelihood;
}


double EndpointObservationModel::hitLikelihood(Point<int> cell, const OccupancyGrid& map)
{
    const uint8_t kInitialGridCost = 127;

    //     const double regularizer = 1;
    //     const double scale = 1.0 / (255.0 + regularizer);
    double rayCost = 0.0;   // 1e-5;

    if (map.isCellInGrid(cell)) {
        cell_type_t rayType = map.getCellTypeNoCheck(cell);
        int cost = map.getCostNoCheck(cell);
        //         (cost > kInitialGridCost) &&
        // (cost > kInitialGridCost) &&
        if ((cost > kInitialGridCost)
            && (~rayType & (kDynamicOccGridCell | kHazardOccGridCell | kUnobservedOccGridCell))) {
            //             rayCost = 255;//(regularizer * cost);
            rayCost = cost;
        }
    }

    return rayCost;
}


///////////////////////   Beam Observation Model implementation //////////////////////
BeamObservationModel::BeamObservationModel(const beam_observation_model_params_t& params)
: pHit(0, params.laserVariance)
, pShort(params.lambdaShort)
, params(params)
{
}


void BeamObservationModel::initializeModel(const laser::laser_scan_lines_t& scan,
                                           const MultivariateGaussian& proposalDistribution)
{
    if (params.useCovarianceWeighting) {
        calculate_covariance_weights(scan,
                                     proposalDistribution,
                                     rayStride(scan.scan.angularResolution),
                                     covarianceWeights);
    } else {
        covarianceWeights.resize(scan.scan.numRanges, 1.0f);
    }
}


double BeamObservationModel::sampleLikelihood(const particle_t& sample,
                                              const laser::laser_scan_lines_t& scan,
                                              const OccupancyGrid& map,
                                              particle_filter_debug_info_t* debug)
{
    // setup distributions for likelihood computation
    maxRange = scan.scan.maxRange;
    pRand.setRange(0.0, maxRange);

    particle_grid_score_t particleScore;
    if (debug) {
        particleScore.particleId = sample.id;
        particleScore.scores.resize(scan.scan.numRanges);
    }

    int stepSize = rayStride(scan.scan.angularResolution);
    laser::MovingLaserScan movingScan(scan.scan, sample.parent, sample.pose, stepSize);

    double weightedScanScore = 0.0;   // weighted scan score to be computed

    for (std::size_t i = 0; i < movingScan.size(); ++i) {
        auto& ray = movingScan[i];
        float rayBearing = angle_to_point(ray.position, ray.endpoint);
        double rayWeight = 0.0;

        if ((ray.range > 0.0) && (ray.range < params.maxRayLength)) {
            auto gridPoint = utils::global_point_to_grid_cell(ray.endpoint, map);

            if (map.isCellInGrid(gridPoint)) {
                // occupancy-based weight
                rayWeight = computeOccupancyWeight(gridPoint, map);
                rayWeight *= computeDistanceWeight(ray.range);
                rayWeight *= computeLineWeight(scan, i * stepSize, rayBearing);
                rayWeight *= covarianceWeights[i];
            }
        }   // rayWeight is zero if the range is outside acceptable bound

        // sum up weighted ray likelihoods to get the score
        double rayLikelihood =
          (rayWeight > params.minRayWeight) ? beamLikelihood(ray.range, rayBearing, ray.position, map) : 0.0;
        weightedScanScore += rayWeight * rayLikelihood;

        if (debug) {
            particleScore.scores[i] = weightedScanScore;
        }
    }

    // normalize the score to get the liklihood for a scan
    assert(movingScan.size() != 0);
    double likelihood = weightedScanScore / movingScan.size();

    if (debug) {
        debug->particleScores.push_back(particleScore);
    }

#ifdef DEBUG_WEIGHTED_BEAM
    std::cout << "INFO: BeamObservationModel: sample:" << sample.pose << " likelihood:" << likelihood << '\n';
#endif

    return likelihood;
}


double BeamObservationModel::computeOccupancyWeight(const Point<int>& gridPoint, const OccupancyGrid& map)
{
    double occupancyWeight = 0.0;

    if (map.getCellTypeNoCheck(gridPoint) & (kUnsafeOccGridCell | kUnknownOccGridCell)) {
        occupancyWeight = static_cast<double>(map.getCostNoCheck(gridPoint)) / map.getMaxCellCost();
    }

    return occupancyWeight;
}


double BeamObservationModel::computeDistanceWeight(double range)
{
    if (params.useDistanceWeighting) {
        return params.minDistanceWeight + (1 - params.minDistanceWeight) * (range / params.maxRayLength);
    } else {
        return 1.0;
    }
}


double BeamObservationModel::computeLineWeight(const laser::laser_scan_lines_t& scan, int rayIndex, double rayAngle)
{
    if (!params.useLineWeighting) {
        return 1.0;
    }

    double lineWeight = params.defaultLineWeight;

    int rayLineIndex = scan.scanPointToLineIndices[rayIndex];

    if (rayLineIndex != -1) {
        float lineLength = length(scan.lines[rayLineIndex]);

        if (lineLength > 0.05)   // ignore very short lines
        {
            float lineOrientationX = (scan.lines[rayLineIndex].b.x - scan.lines[rayLineIndex].a.x) / lineLength;
            float lineOrientationY = (scan.lines[rayLineIndex].b.y - scan.lines[rayLineIndex].a.y) / lineLength;

            float rayAngleX = cos(rayAngle);
            float rayAngleY = sin(rayAngle);

            lineWeight = std::abs(rayAngleX * lineOrientationX
                                  + rayAngleY * lineOrientationY);   // weight based on beam incident angle
        }
    }

    return lineWeight;
}


double BeamObservationModel::beamLikelihood(double radius,
                                            double theta,
                                            const Point<double>& laserPosition,
                                            const OccupancyGrid& map)
{
    auto startingGridPoint = utils::global_point_to_grid_point(laserPosition, map);
    auto expectedEndpoint = utils::trace_ray_until_condition(startingGridPoint, theta, 30.0f, map, is_ray_endpoint);
    double expectedDistance = distance_between_points(startingGridPoint, expectedEndpoint);
    double zMaxLikelihood = (radius == maxRange) ? 1.0 : 0.0;

    pHit.setMean(expectedDistance);
    pShort.setMaxValue(expectedDistance);

    return pHit.likelihood(radius) * params.zHit + pShort.likelihood(radius) * params.zShort
      + pRand.likelihood(radius) * params.zRand + zMaxLikelihood * params.zMax;
}


int BeamObservationModel::rayStride(float scanAngularResolution)
{
    return std::max(static_cast<int>(params.angleBetweenBeams / std::abs(scanAngularResolution)), 1);
}

}   // namespace hssh
}   // namespace vulcan
