/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boundary_classifier.cpp
* \author   Collin Johnson
*
* Definition of BoundaryClassifier.
*/

#include <hssh/local_topological/area_detection/labeling/boundary_classifier.h>
#include <hssh/local_topological/area_detection/labeling/boundary.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis_type.h>
#include <hssh/local_topological/training/labeled_boundary_data.h>
#include <hssh/types.h>

namespace vulcan
{
namespace hssh
{

const std::string kBoundaryExtension(".bnd");


std::unique_ptr<BoundaryClassifier> BoundaryClassifier::LearnClassifier(const LabeledBoundaryData& examples)
{
    const int kNumTypes = 4;

    // Initialize all counts to 1 to ensure no probability is 0.
    IntMatrix posCounts(kNumTypes, kNumTypes);
    posCounts.ones();

    auto negCounts = posCounts;

    BoundaryTypeDistribution dist;

    std::array<int, 2> idx;
    for(auto& ex : examples)
    {
        idx[0] = dist.typeIdx(ex.types[0]);
        idx[1] = dist.typeIdx(ex.types[1]);

        if((idx[0] >= 0) && (idx[1] >= 0))
        {
            if(ex.isOn)
            {
                ++posCounts(idx[0], idx[1]);
            }
            else
            {
                ++negCounts(idx[0], idx[1]);
            }
        }
    }

    // Compute the probability as the relative frequency of each type of boundary
    double total = arma::accu(posCounts) + arma::accu(negCounts);
    Matrix posModel(kNumTypes, kNumTypes);
    auto negModel = posModel;
    for(int n = 0; n < kNumTypes; ++n)
    {
        for(int m = 0; m <= n; ++m)
        {
            // If on the diagonal, only need the single counts, otherwise need both sides of symmetric matrix
            int c = (n == m) ? posCounts(n, m) : posCounts(n, m) + posCounts(m, n);
            posModel(n, m) = c / total;

            // Copy the result to the other triangle in the matrix
            posModel(m, n) = posModel(n, m);

            // If on the diagonal, only need the single counts, otherwise need both sides of symmetric matrix
            c = (n == m) ? negCounts(n, m) : negCounts(n, m) + negCounts(m, n);
            negModel(n, m) = c / total;

            // Copy the result to the other triangle in the matrix
            negModel(m, n) = negModel(n, m);
        }
    }

    return std::unique_ptr<BoundaryClassifier>(new BoundaryClassifier(posModel, negModel));
}


BoundaryClassifier::BoundaryClassifier(const std::string& classifierName)
{
    load(classifierName);
}


BoundaryClassifier::BoundaryClassifier(const Matrix& posModel, const Matrix& negModel)
{
    dist_.posModel = posModel;
    dist_.negModel = negModel;

    logDist_.posModel = arma::log(dist_.posModel);
    logDist_.negModel = arma::log(dist_.negModel);
}


double BoundaryClassifier::classifyBoundary(HypothesisType typeA, HypothesisType typeB, bool isOn) const
{
    int idxA = dist_.typeIdx(typeA);
    int idxB = dist_.typeIdx(typeB);

    if(idxA < 0 || idxB < 0)
    {
        std::cout << "TypeA: " << typeA << " TypeB:" << typeB << '\n';
        assert(idxA >= 0 && idxB >= 0);
    }

    const auto& model = isOn ? dist_.posModel : dist_.negModel;

    // If valid types are found, then there's some probability. Otherwise probability is 0.
    return ((idxA >= 0) && (idxB >= 0)) ? model(idxA, idxB) : 0.0;
}


double BoundaryClassifier::classifyBoundaryLog(HypothesisType typeA, HypothesisType typeB, bool isOn) const
{
    int idxA = logDist_.typeIdx(typeA);
    int idxB = logDist_.typeIdx(typeB);

    if(idxA < 0 || idxB < 0)
    {
        std::cout << "TypeA: " << typeA << " TypeB:" << typeB << '\n';
        assert(idxA >= 0 && idxB >= 0);
    }

    const auto& model = isOn ? logDist_.posModel : logDist_.negModel;

    // If valid types are found, then there's some probability. Otherwise probability is 0.
    return ((idxA >= 0) && (idxB >= 0)) ? model(idxA, idxB) : 0.0;
}


BoundaryTypeDistribution BoundaryClassifier::boundaryDistribution(void) const
{
    return dist_;
}


bool BoundaryClassifier::save(const std::string& filename) const
{
    std::ofstream out(filename + kBoundaryExtension);
    if(!dist_.posModel.save(out, arma::arma_ascii))
    {
        std::cout << "ERROR: BoundaryClassifier::save: Failed to save positive model to " << filename
            << kBoundaryExtension << '\n';
        return false;
    }

    if(!dist_.negModel.save(out, arma::arma_ascii))
    {
        std::cout << "ERROR: BoundaryClassifier::save: Failed to save negative model to " << filename
            << kBoundaryExtension << '\n';
        return false;
    }

    return true;
}


bool BoundaryClassifier::load(const std::string& filename)
{
    std::ifstream in(filename + kBoundaryExtension);
    if(!dist_.posModel.load(in, arma::arma_ascii))
    {
        std::cout << "ERROR: BoundaryClassifier::load: Failed to load positive model from " << filename
            << kBoundaryExtension << '\n';
        return false;
    }

    if(!dist_.negModel.load(in, arma::arma_ascii))
    {
        std::cout << "ERROR: BoundaryClassifier::load: Failed to load negative model from " << filename
            << kBoundaryExtension << '\n';
        return false;
    }

    // Renormalize
//     dist_.posModel /= arma::accu(dist_.posModel);
//     dist_.negModel /= arma::accu(dist_.negModel);

    // For now, set the path probabilities to the sums of path end and path to ensure they are correctly
    // accounted for when not separating endpoint vs side
//     dist_.posModel(0, 1) += dist_.posModel(3, 1);
//     dist_.posModel(0, 2) += dist_.posModel(3, 2);
//     dist_.posModel(1, 0) = dist_.posModel(0, 1);
//     dist_.posModel(2, 0) = dist_.posModel(0, 2);
//
//     dist_.posModel(3, 1) = dist_.posModel(0, 1);
//     dist_.posModel(3, 2) = dist_.posModel(0, 2);
//     dist_.posModel(1, 3) = dist_.posModel(0, 1);
//     dist_.posModel(2, 3) = dist_.posModel(0, 2);

    logDist_.posModel = arma::log(dist_.posModel);
    logDist_.negModel = arma::log(dist_.negModel);

    return true;
}

} // namespace hssh
} // namespace vulcan
