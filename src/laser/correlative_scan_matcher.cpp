/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     correlative_scan_matcher.cpp
* \author   Collin Johnson
*
* Definition of CorrelativeScanMatcher.
*/

#include <laser/correlative_scan_matcher.h>
#include <core/matrix.h>
#include <core/vector.h>
#include <core/multivariate_gaussian.h>
#include <algorithm>
#include <iostream>
#include <cassert>


// #define DEBUG_SEARCH_VOLUME
// #define DEBUG_TRANSFORM_SCORES
// #define DEBUG_COARSE_SEARCH
// #define DEBUG_FINE_SEARCH
// #define DEBUG_TRANSFORM_STATISTICS

namespace vulcan
{
namespace laser
{

// Helper structs for organizing the computation
struct search_range_t
{
    float   minValue;
    float   increment;    // discretization of the search range
    uint8_t numSteps;
};

struct search_volume_t
{
    search_range_t xRange;
    search_range_t yRange;
    search_range_t thetaRange;
};

struct search_volume_scores_t
{
    search_volume_scores_t(const search_volume_t& volume);
    ~search_volume_scores_t(void);

    uint32_t*** scores;

    uint8_t numThetaSteps;
    uint8_t numXSteps;
    uint8_t numYSteps;
};

struct search_voxel_t
{
    // (x,y,theta) is lower-left position of the voxel
    float x;
    float y;
    float theta;

    // How long voxel is along the given dimension
    float xLength;
    float yLength;
    float thetaLength;

    // Score assigned when voxel was created
    float score;
};

// These are the statistics that are needed to determine the covariance matrix of
// the transform estimate
struct search_statistics_t
{
    search_statistics_t(void);

    Matrix weightedVoxelSquareSum;
    Vector weightedVoxelSum;
    double             sumOfVoxelLikelihoods;
};


std::ostream& operator<<(std::ostream& out, const search_range_t& range)
{
    out<<'('<<range.minValue<<','<<range.numSteps<<','<<range.increment<<')';
    return out;
}

std::ostream& operator<<(std::ostream& out, const search_voxel_t& voxel)
{
    out<<'('<<voxel.x<<','<<voxel.y<<','<<voxel.theta<<','<<voxel.score<<')';
    return out;
}


search_volume_t create_initial_search_volume(const scan_matcher_params_t& params);
search_range_t  create_search_range(float center, float distance, float discretization);


// Transforms is updated with the freshly calculated fine-resolution transforms. Other option involved
// a huge and unnecessary amount of copying transforms around.
// Transform with the highest score is returned
void calculate_scores_for_coarse_resolution_grid(const search_volume_t&                  volume,
                                                 const std::vector<Point<float>>& scan,
                                                 ScanLikelihoodGrid&                     coarseGrid,
                                                 search_volume_scores_t&                 scores);

search_voxel_t search_fine_resolution_grid_for_best_match(search_volume_scores_t&                 coarseScores,
                                                          const search_volume_t&                  coarseVolume,
                                                          const std::vector<Point<float>>& scan,
                                                          ScanLikelihoodGrid&                     fineResolutionGrid,
                                                          const scan_matcher_params_t&            params,
                                                          search_statistics_t&                    statistics);

search_voxel_t  create_voxel_from_volume_indices(uint8_t x, uint8_t y, uint8_t theta, const search_volume_t& volume);

search_volume_t create_search_volume_from_coarse_voxel(const search_voxel_t& coarseVoxel,
                                                       float fineLinearDiscretization,
                                                       float fineAngularDiscretization);

search_voxel_t select_best_voxel(search_volume_scores_t& scores, const search_volume_t& volume);
void           calculate_volume_statistics(const search_volume_scores_t& scores, const search_volume_t& volume, search_statistics_t& stats);

bool have_found_best_possible_transform(float fineResolutionScore, float nextCoarseResolutionScore);

// Helpers for the scan matching
void calculate_volume_scores(const search_volume_t& volume, const std::vector<Point<float>>& scan,
                             ScanLikelihoodGrid& likelihoodGrid, search_volume_scores_t& scores);

void calculate_slice_scores(const search_range_t& xRange, const search_range_t& yRange,
                            const std::vector<Point<float>>& scan, ScanLikelihoodGrid& likelihoodGrid,
                            search_volume_scores_t& scores, uint8_t thetaIndex);

void rotate_scan(std::vector<Point<float>>& rotatedScan, const std::vector<Point<float>>& scan, float deltaTheta);

Vector voxel_to_vector(const search_voxel_t& voxel);
Matrix calculate_transform_covariance(const search_statistics_t& stats);


CorrelativeScanMatcher::CorrelativeScanMatcher(const scan_matcher_params_t& params)
    : fineResolutionGrid(params.fineResolutionGridWidth, params.fineResolutionGridHeight, params.fineResolutionMetersPerCell)
    , coarseResolutionGrid(params.fineResolutionGridWidth, params.fineResolutionGridHeight, params.fineResolutionMetersPerCell)
    , params(params)
{
    fineResolutionGrid.setLaserVariance(0.0025);
}


void CorrelativeScanMatcher::setReferenceScan(const std::vector<Point<float>>& reference, int64_t referenceTime)
{
    fineResolutionGrid.setReferenceScan(reference);

    coarseResolutionMetersPerCell = fineResolutionGrid.createCoarseGrid(params.coarseResolutionMetersPerCell, coarseResolutionGrid);

    referenceScanTimestamp = referenceTime;
}


MultivariateGaussian CorrelativeScanMatcher::findMostLikelyTransform(const std::vector<Point<float>>& scan, int64_t scanTime) const
{
    int64_t timeSinceReference = scanTime - referenceScanTimestamp;

    Vector transformVector;
    Matrix covariance;

    if(timeSinceReference > 250000)
    {
        timeSinceReference = 250000;
    }

    search_volume_t        coarseVolume = create_initial_search_volume(params);
    search_volume_scores_t coarseScores(coarseVolume);

    calculate_scores_for_coarse_resolution_grid(coarseVolume, scan, coarseResolutionGrid, coarseScores);

    search_statistics_t searchStats;
    search_voxel_t bestVoxel = search_fine_resolution_grid_for_best_match(coarseScores, coarseVolume, scan, fineResolutionGrid, params, searchStats);

    transformVector = voxel_to_vector(bestVoxel);
    covariance      = calculate_transform_covariance(searchStats);

    if(timeSinceReference == 250000)
    {
        covariance *= 100;
    }

    return MultivariateGaussian(transformVector, covariance);
}


search_volume_scores_t::search_volume_scores_t(const search_volume_t& volume)
    : numThetaSteps(volume.thetaRange.numSteps)
    , numXSteps(volume.xRange.numSteps)
    , numYSteps(volume.yRange.numSteps)
{
    scores = new uint32_t**[numThetaSteps];

    for(int x = numThetaSteps; --x >= 0;)
    {
        scores[x] = new uint32_t*[numXSteps];

        for(int y = numXSteps; --y >= 0;)
        {
            scores[x][y] = new uint32_t[numYSteps];
        }
    }
}


search_volume_scores_t::~search_volume_scores_t(void)
{
    for(int x = numThetaSteps; --x >= 0;)
    {
        for(int y = numXSteps; --y >= 0;)
        {
            delete [] scores[x][y];
        }

        delete [] scores[x];
    }

    delete [] scores;
}


search_statistics_t::search_statistics_t(void)
    : weightedVoxelSquareSum(3, 3)
    , weightedVoxelSum(3)
    , sumOfVoxelLikelihoods(0)
{
    weightedVoxelSquareSum.zeros();
    weightedVoxelSum.zeros();
}



void calculate_scores_for_coarse_resolution_grid(const search_volume_t&                  volume,
                                                 const std::vector<Point<float>>& scan,
                                                 ScanLikelihoodGrid&                     coarseGrid,
                                                 search_volume_scores_t&                 scores)
{
    calculate_volume_scores(volume, scan, coarseGrid, scores);

    #ifdef DEBUG_COARSE_SEARCH
    std::cout<<"Coarse scores:\n";
    for(uint8_t theta = 0; theta < scores.numThetaSteps; ++theta)
    {
        for(uint8_t x = 0; x < scores.numXSteps; ++x)
        {
            for(uint8_t y = 0; y < scores.numYSteps; ++y)
            {
                search_voxel_t voxel = create_voxel_from_volume_indices(x, y, theta, volume);
                voxel.score = scores.scores[theta][x][y];
                std::cout<<voxel<<' ';

            }
            std::cout<<'\n';
        }
        std::cout<<'\n';
    }
    #endif

}


search_voxel_t search_fine_resolution_grid_for_best_match(search_volume_scores_t&       coarseScores,
                                                          const search_volume_t&        coarseVolume,
                                                          const std::vector<Point<float>>& scan,
                                                          ScanLikelihoodGrid&           fineResolutionGrid,
                                                          const scan_matcher_params_t&  params,
                                                          search_statistics_t&          statistics)
{
    search_voxel_t bestOverallVoxel;
    bestOverallVoxel.score = 0;

    int numCoarseVoxels = coarseScores.numThetaSteps * coarseScores.numXSteps * coarseScores.numYSteps;

    search_voxel_t voxelToSearch = select_best_voxel(coarseScores, coarseVolume);
    search_volume_t voxelVolume = create_search_volume_from_coarse_voxel(voxelToSearch, params.fineResolutionMetersPerCell, params.fineAngularSearchResolution);

    search_volume_scores_t fineScores(voxelVolume);

    int numTransformsConsidered = 0;

    while(--numCoarseVoxels >= 0)
    {
        calculate_volume_scores(voxelVolume, scan, fineResolutionGrid, fineScores);

        /*
        * NOTE: There is an order dependency here! calculate_volume_statistics MUST come before select_best_voxel
        *       because select_best_voxel sets the score for the best voxel to 0! Be aware.
        */
        calculate_volume_statistics(fineScores, voxelVolume, statistics);

        numTransformsConsidered += fineScores.numThetaSteps * fineScores.numXSteps * fineScores.numYSteps;

        search_voxel_t bestVoxelInVolume = select_best_voxel(fineScores, voxelVolume);

        #ifdef DEBUG_FINE_SEARCH
        std::cout<<"DEBUG: Best voxel: Coarse: "<<voxelToSearch<<" Fine: "<<bestVoxelInVolume<<'\n';
        #endif

        voxelToSearch = select_best_voxel(coarseScores, coarseVolume);
        voxelVolume   = create_search_volume_from_coarse_voxel(voxelToSearch, params.fineResolutionMetersPerCell, params.fineAngularSearchResolution);

        if(bestVoxelInVolume.score > bestOverallVoxel.score)
        {
            bestOverallVoxel = bestVoxelInVolume;
        }

        // Need to check some minimum number of transforms in order to generate a good covariance hypothesis
        if(have_found_best_possible_transform(bestOverallVoxel.score, voxelToSearch.score) &&
           (numTransformsConsidered > params.minNumTransformsToConsider))
        {
            break;
        }
    }

    #ifdef DEBUG_FINE_SEARCH
    std::cout<<"INFO: Searched "<<numTransformsConsidered<<" transforms.\n";
    std::cout<<"DEBUG: Best voxel: "<<bestOverallVoxel<<'\n';
    #endif

    return bestOverallVoxel;
}


void calculate_volume_scores(const search_volume_t& volume, const std::vector<Point<float>>& scan,
                             ScanLikelihoodGrid& likelihoodGrid, search_volume_scores_t& scores)
{
    std::vector<Point<float>> scanToScore(scan);

    float theta = volume.thetaRange.minValue;

    for(uint8_t thetaIndex = 0; thetaIndex < volume.thetaRange.numSteps; ++thetaIndex, theta += volume.thetaRange.increment)
    {
        rotate_scan(scanToScore, scan, theta);

        likelihoodGrid.setLikelihoodSearchBaseScan(scanToScore);

        calculate_slice_scores(volume.xRange, volume.yRange, scanToScore, likelihoodGrid, scores, thetaIndex);
    }
}


void calculate_slice_scores(const search_range_t&                  xRange,
                            const search_range_t&                  yRange,
                            const std::vector<Point<float>>& scan,
                            ScanLikelihoodGrid&                    likelihoodGrid,
                            search_volume_scores_t&                scores,
                            uint8_t                                thetaIndex)
{
    float x = xRange.minValue;

    for(uint8_t xStep = 0; xStep < xRange.numSteps; ++xStep, x += xRange.increment)
    {
        float y = yRange.minValue;

        for(uint8_t yStep = 0; yStep < yRange.numSteps; ++yStep, y += yRange.increment)
        {
            scores.scores[thetaIndex][xStep][yStep] = likelihoodGrid.calculateTransformLikelihood(x, y);
        }
    }
}


void rotate_scan(std::vector<Point<float>>& rotatedScan, const std::vector<Point<float>>& scan, float deltaTheta)
{
    float cosTheta = cos(deltaTheta);
    float sinTheta = sin(deltaTheta);

    for(int i = rotatedScan.size(); --i >= 0;)
    {
        rotatedScan[i].x = scan[i].x*cosTheta - scan[i].y*sinTheta;
        rotatedScan[i].y = scan[i].x*sinTheta + scan[i].y*cosTheta;
    }
}


bool have_found_best_possible_transform(float fineResolutionScore, float nextCoarseResolutionScore)
{
    return fineResolutionScore > nextCoarseResolutionScore;
}


Vector voxel_to_vector(const search_voxel_t& voxel)
{
    Vector transformVector(3);

    transformVector(0) = voxel.x;
    transformVector(1) = voxel.y;
    transformVector(2) = voxel.theta;

    return transformVector;
}


Matrix calculate_transform_covariance(const search_statistics_t& stats)
{
    // The numbers here are VERY BIG, so divide out the sum of likelihoods for each weightedVoxelSum otherwise, well it's just ugly.
    Matrix covarianceDouble = (stats.weightedVoxelSquareSum / stats.sumOfVoxelLikelihoods) -
                                           ((stats.weightedVoxelSum / stats.sumOfVoxelLikelihoods) * (trans(stats.weightedVoxelSum) / stats.sumOfVoxelLikelihoods));

    Matrix covariance(3, 3);

    for(int i = 3; --i >= 0;)
    {
        for(int j = 3; --j >= 0;)
        {
            covariance(i, j) = static_cast<float>(covarianceDouble(i, j));
        }
    }

    #ifdef DEBUG_TRANSFORM_STATISTICS
    std::cout<<"INFO: CorrelativeScanMatcher: K:("<<stats.weightedVoxelSquareSum(0,0)<<' '<<stats.weightedVoxelSquareSum(0, 1)<<' '<<stats.weightedVoxelSquareSum(0, 2)<<'\n'
             <<"                                 "<<stats.weightedVoxelSquareSum(1,0)<<' '<<stats.weightedVoxelSquareSum(1, 1)<<' '<<stats.weightedVoxelSquareSum(1, 2)<<'\n'
             <<"                                 "<<stats.weightedVoxelSquareSum(2,0)<<' '<<stats.weightedVoxelSquareSum(2, 1)<<' '<<stats.weightedVoxelSquareSum(2, 2)<<")\n";

    std::cout<<"INFO: CorrelativeScanMatcher: u:("<<stats.weightedVoxelSum(0)<<','<<stats.weightedVoxelSum(1)<<','<<stats.weightedVoxelSum(2)<<")\n";

    std::cout<<"INFO: CorrelativeScanMatcher: s:"<<stats.sumOfVoxelLikelihoods<<'\n';

    std::cout<<"INFO: CorrelativeScanMatcher: cov: ("<<covariance(0,0)<<' '<<covariance(0, 1)<<' '<<covariance(0, 2)<<'\n'
             <<"                                    "<<covariance(1,0)<<' '<<covariance(1, 1)<<' '<<covariance(1, 2)<<'\n'
             <<"                                    "<<covariance(2,0)<<' '<<covariance(2, 1)<<' '<<covariance(2, 2)<<")\n";
    #endif

    return covariance;
}


search_volume_t create_initial_search_volume(const scan_matcher_params_t& params)
{
    /*
    * The actual search volume can be greatly refined to account for robot kinematics. The Olson
    * paper says that having a large search area gives a better determination of the covariance, so
    * the current implementation is conservative.
    */

    search_volume_t volume;

    volume.xRange     = create_search_range(params.minXSearchArea, params.maxXSearchArea-params.minXSearchArea, params.coarseResolutionMetersPerCell);
    volume.yRange     = create_search_range(params.minYSearchArea, params.maxYSearchArea-params.minYSearchArea, params.coarseResolutionMetersPerCell);
    volume.thetaRange = create_search_range(params.minThetaSearchArea, params.maxThetaSearchArea-params.minThetaSearchArea, params.coarseAngularSearchResolution);

    return volume;
}


search_volume_t create_search_volume_from_coarse_voxel(const search_voxel_t& coarseVoxel,
                                                       float fineLinearDiscretization,
                                                       float fineAngularDiscretization)
{
    search_volume_t volume;

    volume.xRange     = create_search_range(coarseVoxel.x,     coarseVoxel.xLength,     fineLinearDiscretization);
    volume.yRange     = create_search_range(coarseVoxel.y,     coarseVoxel.yLength,     fineLinearDiscretization);
    volume.thetaRange = create_search_range(coarseVoxel.theta, coarseVoxel.thetaLength, fineAngularDiscretization);

    #ifdef DEBUG_SEARCH_VOLUME
    std::cout<<"DEBUG: volume_from_coarse: xRange="<<volume.xRange<<" yRange="<<volume.yRange<<" thetaRange="<<volume.thetaRange<<'\n';
    #endif

    return volume;
}


search_range_t create_search_range(float left, float distance, float discretization)
{
    search_range_t range;

    range.minValue  = left;
    range.increment = discretization;
    range.numSteps  = static_cast<uint8_t>(round(distance/discretization)) + 1;

    return range;
}


search_voxel_t select_best_voxel(search_volume_scores_t& scores, const search_volume_t& volume)
{
    // Brute force search to find the best voxel in the remaining scores.
    uint32_t bestScore      = 0;
    uint8_t  bestXIndex     = 0;
    uint8_t  bestYIndex     = 0;
    uint8_t  bestThetaIndex = 0;

    for(uint8_t theta = 0; theta < scores.numThetaSteps; ++theta)
    {
        for(uint8_t x = 0; x < scores.numXSteps; ++x)
        {
            for(uint8_t y = 0; y < scores.numYSteps; ++y)
            {
                if(scores.scores[theta][x][y] > bestScore)
                {
                    bestScore      = scores.scores[theta][x][y];
                    bestXIndex     = x;
                    bestYIndex     = y;
                    bestThetaIndex = theta;
                }
            }
        }
    }

    search_voxel_t voxel = create_voxel_from_volume_indices(bestXIndex, bestYIndex, bestThetaIndex, volume);
    voxel.score          = bestScore;

    // Set the best voxel score to 0 so that it will not be selected in the future
    scores.scores[bestThetaIndex][bestXIndex][bestYIndex] = 0;

    return voxel;
}


search_voxel_t create_voxel_from_volume_indices(uint8_t x, uint8_t y, uint8_t theta, const search_volume_t& volume)
{
    search_voxel_t voxel;

    voxel.x     = volume.xRange.minValue + volume.xRange.increment*x;
    voxel.y     = volume.yRange.minValue + volume.yRange.increment*y;
    voxel.theta = volume.thetaRange.minValue + volume.thetaRange.increment*theta;

    voxel.xLength     = volume.xRange.increment;
    voxel.yLength     = volume.yRange.increment;
    voxel.thetaLength = volume.thetaRange.increment;

    return voxel;
}


void calculate_volume_statistics(const search_volume_scores_t& scores, const search_volume_t& volume, search_statistics_t& stats)
{
    Vector voxelVector(3);

    for(uint8_t theta = 0; theta < scores.numThetaSteps; ++theta)
    {
        for(uint8_t x = 0; x < scores.numXSteps; ++x)
        {
            for(uint8_t y = 0; y < scores.numYSteps; ++y)
            {
                voxelVector(0) = volume.xRange.minValue + volume.xRange.increment*x;
                voxelVector(1) = volume.yRange.minValue + volume.yRange.increment*y;
                voxelVector(2) = volume.thetaRange.minValue + volume.thetaRange.increment*theta;

                stats.weightedVoxelSquareSum += scores.scores[theta][x][y] * (voxelVector * trans(voxelVector));
                stats.weightedVoxelSum       += scores.scores[theta][x][y] * voxelVector;
                stats.sumOfVoxelLikelihoods  += scores.scores[theta][x][y];
            }
        }
    }
}

} // namespace laser
} // namespace vulcan
