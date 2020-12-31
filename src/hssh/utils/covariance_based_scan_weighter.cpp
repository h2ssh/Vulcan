/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     covariance_based_scan_weighter.cpp
* \author   Collin Johnson
*
* Definition of CovarianceBasedScanWeighter.
*/

#include "hssh/utils/covariance_based_scan_weighter.h"
#include "laser/laser_scan_lines.h"
#include "core/multivariate_gaussian.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

CovarianceBasedScanWeighter::CovarianceBasedScanWeighter(scan_weight_type_t type)
{

}


std::vector<float> CovarianceBasedScanWeighter::calculateWeights(const laser::laser_scan_lines_t& scan,
                                                                 const MultivariateGaussian& poseDistribution)
{

}


void CovarianceBasedScanWeighter::calculateDecomposition(const MultivariateGaussian& priorPoseDistribution)
{
    Matrix xyCov = priorPoseDistribution.getCovariance().submat(arma::span(0,1), arma::span(0,1));

    arma::eig_sym(eigenvalues, eigenvectors, xyCov);

    eigenvalues /= arma::sum(eigenvalues);

    #ifdef DEBUG_COVARIANCE
    std::cout<<"INFO:CovWeight:eigval:"<<eigenvalues<<" eigvec:"<<eigenvectors<<std::endl;;
    #endif
}


void CovarianceBasedScanWeighter::calculateScanWeights(const laser::laser_scan_lines_t&  scan,
                                                       const MultivariateGaussian& priorPoseDistribution)
{
    scanWeights.resize(scan.scan.numRanges, 0);

    float defaultWeight   = arma::sum(eigenvalues) / 2.0f;
    float currentWeight   = 0;
    int   loadedLineIndex = -1;
    arma::rowvec scanLine(2);

    for(int n = scan.scan.numRanges; --n >= 0;)
    {
        int currentLineIndex = scan.scanPointToLineIndices[n];
        if(currentLineIndex == -1)
        {
            scanWeights[n] = defaultWeight;
            continue;
        }
        else if(currentLineIndex != loadedLineIndex)
        {
            float lineLength = length(scan.lines[currentLineIndex]);

            // Lines of length 0 -- yes they happen -- will break the weighting, so throw them out
            if(lineLength > 0.01)
            {
                scanLine(0) = (scan.lines[currentLineIndex].b.x-scan.lines[currentLineIndex].a.x) / lineLength;
                scanLine(1) = (scan.lines[currentLineIndex].b.y-scan.lines[currentLineIndex].a.y) / lineLength;

                loadedLineIndex = currentLineIndex;

                Vector result = (1.0f - arma::abs(scanLine*eigenvectors))*eigenvalues;

                currentWeight = result(0);

                assert(currentWeight != NAN);
            }
            else
            {
                currentWeight = defaultWeight;
            }

            #ifdef DEBUG_COVARIANCE
            std::cout<<"INFO:CovWeight:line:"<<scanLine<<" weight:"<<currentWeight<<'\n';
            #endif
        }

        assert(currentWeight >= 0.0);

        scanWeights[n] = currentWeight;
    }
}

} // namespace hssh
} // namespace vulcan
