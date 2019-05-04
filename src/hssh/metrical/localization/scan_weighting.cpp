/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     scan_weighting.cpp
* \author   Collin Johnson
*
* Definition of functions that calculate various types of weights for scans:
*
*   - calculate_covariance_weights
*/

#include <hssh/metrical/localization/scan_weighting.h>
#include <laser/laser_scan_lines.h>
#include <core/multivariate_gaussian.h>
#include <math/statistics.h>
#include <utils/strided_sequence.h>

#include <gnuplot-iostream.h>

// #define DEBUG_COVARIANCE
// #define DEBUG_PLOTS

const double kMinWeight = 1e-3;
const double kNonLineWeight = 0.1;

namespace vulcan
{
namespace hssh
{

void calculate_line_weights(const laser::laser_scan_lines_t&  scan,
                            const MultivariateGaussian& proposalDistribution,
                            int stride,
                            std::vector<float>& weights);

void calculate_distance_weights(const laser::laser_scan_lines_t&  scan,
                                const MultivariateGaussian& proposalDistribution,
                                int stride,
                                std::vector<float>& weights);


void calculate_covariance_weights(const laser::laser_scan_lines_t&  scan,
                                  const MultivariateGaussian& proposalDistribution,
                                  int stride,
                                  std::vector<float>& weights)
{
    static Gnuplot plot[2];

    assert(stride > 0);

    int numWeights = utils::strided_sequence_length(scan.scan.numRanges, stride);
    weights.resize(numWeights);
    std::fill(weights.begin(), weights.end(), 0.0);

    std::vector<float> lineWeights(numWeights, 0.0);
    std::vector<float> distWeights(numWeights, 0.0);

    double orientationWeight = proposalDistribution(2, 2);
    double positionWeight = proposalDistribution(0, 0) + proposalDistribution(1, 1);
    double totalWeight = orientationWeight + positionWeight;

    orientationWeight /= totalWeight;
    positionWeight /= totalWeight;

    calculate_distance_weights(scan, proposalDistribution, stride, distWeights);
    calculate_line_weights(scan, proposalDistribution, stride, lineWeights);

    std::transform(lineWeights.begin(), lineWeights.end(), lineWeights.begin(), [positionWeight](double w) {
        return w * positionWeight;
    });

    std::transform(distWeights.begin(), distWeights.end(), distWeights.begin(), [orientationWeight](double w) {
        return w * orientationWeight;
    });

    auto distBegin = distWeights.begin();
    std::transform(lineWeights.begin(), lineWeights.end(), weights.begin(), [&distBegin](double d) {
        return d + *distBegin++;
    });

    auto maxWeight = *std::max_element(weights.begin(), weights.end());

    std::transform(weights.begin(), weights.end(), weights.begin(), [maxWeight](double w) {
        return w / maxWeight;
    });

#ifdef DEBUG_PLOTS
    plot[scan.scan.laserId] << "set yrange [-0.05:1.05]\n";
    plot[scan.scan.laserId] << "plot '-' with lines title 'Dist Weights', '-' with lines title 'Line Weights', '-' with lines title 'Final Weights'\n";
    plot[scan.scan.laserId].send1d(distWeights);
    plot[scan.scan.laserId].send1d(lineWeights);
    plot[scan.scan.laserId].send1d(weights);
#endif // DEBUG_PLOTS
}


void calculate_line_weights(const laser::laser_scan_lines_t&  scan,
                            const MultivariateGaussian& proposalDistribution,
                            int stride,
                            std::vector<float>& weights)
{
    Matrix eigenvectors;
    Vector eigenvalues;
    Matrix xyCov = proposalDistribution.getCovariance().submat(arma::span(0,1), arma::span(0,1));
    arma::eig_sym(eigenvalues, eigenvectors, xyCov);
    eigenvalues /= arma::sum(eigenvalues);

    double currentWeight = 0.0;
    int   loadedLineIndex = -1;
    float proposalOrientation = proposalDistribution.getMean()(2);
    arma::rowvec scanLine(2);

    // The line rotation is rotating the rowvec, hence need to take the transpose to make the rotation matrix operate
    // correctly
    Matrix lineRotation(2, 2);
    lineRotation(0, 0) = std::cos(proposalOrientation);
    lineRotation(0, 1) = -std::sin(proposalOrientation);
    lineRotation(1, 0) = std::sin(proposalOrientation);
    lineRotation(1, 1) = std::cos(proposalOrientation);
    lineRotation = arma::trans(lineRotation);

#ifdef DEBUG_COVARIANCE
    std::cout << "DEBUG:calculate_covariance_weights:Num lines:" << scan.lines.size() << " Eigenvectors:\n"
        << eigenvectors << " Eigenvalues:\n" << eigenvalues << '\n';
#endif

    auto weightIt = weights.begin();

    for(std::size_t n = 0; n < scan.scan.numRanges; n += stride, ++weightIt)
    {
        int currentLineIndex = scan.scanPointToLineIndices[n];
        // If there's no line, use the non-line weight
        if(currentLineIndex == -1)
        {
            currentWeight = kNonLineWeight;
        }
        // If the line has changed, determine the weight of the new line
        else if(currentLineIndex != loadedLineIndex)
        {
            float lineLength = length(scan.lines[currentLineIndex]);

            // Lines of length 0 -- yes they happen -- will break the weighting, so throw them out
            if(lineLength > 0.01)
            {
                scanLine(0) = (scan.lines[currentLineIndex].b.x - scan.lines[currentLineIndex].a.x) / lineLength;
                scanLine(1) = (scan.lines[currentLineIndex].b.y - scan.lines[currentLineIndex].a.y) / lineLength;
                scanLine    = scanLine * lineRotation;

                Vector result = (1.0f - arma::abs(scanLine * eigenvectors)) * eigenvalues;
                currentWeight = std::max(result(0), kMinWeight);

                loadedLineIndex = currentLineIndex;

#ifdef DEBUG_COVARIANCE
                std::cout<<"INFO:CovWeight:line:"<<scan.lines[currentLineIndex]<< " Dir:"<<scanLine<<" weight:"<<currentWeight << " Dot:\n" << result <<'\n';
#endif

                assert(currentWeight != NAN);
            }
            else
            {
                currentWeight = kNonLineWeight;
            }
        }

        // Use whichever weight line is currently active
        *weightIt += currentWeight;
    }
}


void calculate_distance_weights(const laser::laser_scan_lines_t&  scan,
                                const MultivariateGaussian& proposalDistribution,
                                int stride,
                                std::vector<float>& weights)
{
    // For each cell that the distance steps out, that increases the weight of the scan
    // this will create a discretized weight based on the orientation uncertainty
    const double thetaSigma = std::sqrt(proposalDistribution(2, 2));
    const double metersPerCell = 0.05;  // TODO: Provide this as a parameter

    auto weightIt = weights.begin();
    for(std::size_t n = 0; n < scan.scan.numRanges; n += stride, ++weightIt)
    {
        // Ignore invalid ranges
        if((scan.scan.ranges[n] > 0.0) && (scan.scan.ranges[n] < 30.0))
        {
            *weightIt += std::max(std::lrint(scan.scan.ranges[n] * thetaSigma / metersPerCell), 1L);
        }
    }

    float max = *std::max_element(weights.begin(), weights.end());
    for(auto& w : weights)
    {
        w /= max;
    }
}

} // namespace hssh
} // namespace vulcan
