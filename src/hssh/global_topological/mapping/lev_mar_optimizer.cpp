/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lev_mar_optimizer.cpp
* \author   Collin Johnson
*
* Definition of LevMarOptimizer.
*/

#include "hssh/global_topological/mapping/lev_mar_optimizer.h"
#include "hssh/global_topological/chi.h"
#include "hssh/global_topological/global_place.h"
#include "hssh/global_topological/global_path.h"
#include "hssh/global_topological/topological_map.h"
#include <levmar.h>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <iostream>

// #define DEBUG_INITIAL_P
// #define DEBUG_X
// #define DEBUG_JACOBIAN
// #define DEBUG_INCREMENTAL_P
// #define DEBUG_LAMBDA
// #define DEBUG_COVARIANCE
// #define DEBUG_FINAL_P
// #define DEBUG_LEV_MAR

namespace vulcan
{
namespace hssh
{

// Callbacks for talking with levmar
void levmar_func    (double* p, double* hx, int m, int n, void* adata);
void levmar_jacobian(double* p, double* j, int m, int n, void* adata);


LevMarOptimizer::LevMarOptimizer(const lev_mar_optimizer_params_t& params)
: params_(params)
{
}


Chi LevMarOptimizer::optimizeMap(const TopologicalMap& map)
{
    // Clear out any previous results
    idToPIndex.clear();
    xLambdaIndices.clear();

    p.clear();
    x.clear();
    variance.clear();
    xError.clear();
    covariance.clear();

    countEdges(map);

    // Only run the optimization if there are at least as many edges as places. Otherwise,
    // the optimization is actually unconstrained and thus won't produce different results
    // from just laying out the places using the lambda values.
    // The first place is fixed at (0, 0, 0), then all places afterward are relative to this initial place.
    if((numEdges + 1 < map.numPlaces()) || (map.numPlaces() == 0))
    {
        return Chi(map);
    }

//     if(numEdges >= map.numPlaces())
//     {
        setupPVector(map);
        setupXVector(map);
//         calculateJacobian();
        runLevmar();

        return createChiFromP();
//     }
//     else
//     {
//         return Chi(map);
//     }
}


void LevMarOptimizer::setupPVector(const TopologicalMap& map)
{
    assert(map.numPlaces() > 0);
    numPlaces = map.numPlaces();

    p.resize(numPlaces * 3);
    covariance.resize(p.size() * p.size());
    std::fill(covariance.begin(), covariance.end(), 0.0);
    std::fill(p.begin(), p.end(), 0.0);

    int pIndex = 0;
    // Skip the first place because it is the free parameter that we fix to be (0, 0, 0).
    auto beginIt = map.places().begin();
    initialId_ = beginIt->first;
    ++beginIt;
    for(auto& place : boost::make_iterator_range(beginIt, map.places().end()))
    {
        pose_t pose = map.referenceFrame(place.second->id());

        p[pIndex*3]     = pose.x;
        p[pIndex*3 + 1] = pose.y;
        p[pIndex*3 + 2] = pose.theta;

        idToPIndex[place.first] = pIndex;

        ++pIndex;
    }

    // Save the initial, fixed at the end of the pVector, so the optimization can safely refer to it without actually
    // trying to overwrite it. The value is always 0, so it should work okay.
    idToPIndex[initialId_] = pIndex;
    p[pIndex*3]     = 0.0;
    p[pIndex*3 + 1] = 0.0;
    p[pIndex*3 + 2] = 0.0;

#ifdef DEBUG_INITIAL_P
    std::cout<<"DEBUG:LevMar:Initial p-vector (initial mean poses):\n";
    for(std::size_t n = 0; n < numPlaces; ++n)
    {
        std::cout<<'('<<p[n*3]<<','<<p[n*3 + 1]<<','<<p[n*3 + 2]<<")\n";
    }
#endif
}


void LevMarOptimizer::setupXVector(const TopologicalMap& map)
{
    const std::size_t kXMinSize = numEdges * 3;

    if(x.capacity() < kXMinSize)
    {
        x.reserve(kXMinSize);
        variance.reserve(kXMinSize);
        xError.reserve(kXMinSize);
    }

    std::size_t xIndex = 0;

    for(auto& segment : boost::adaptors::values(map.segments()))
    {
        // Ignore frontiers the front half of a loop, as the constraint will be added for the end segment
        if(segment->isFrontier())
        {
            continue;
        }

        // Check the indices to see if this is a well-constrained segment, which means both ends have been visited.
        // Don't add lambdas for frontier segments because they aren't actually useful information for constraining
        // the Chi value

        std::pair<int, int> pIndices(idToPIndex[segment->plusPlace().id()],
                                     idToPIndex[segment->minusPlace().id()]);

        assert(xIndex == xLambdaIndices.size());
        assert(xIndex * 3 == variance.size());

#ifdef DEBUG_X
        std::cout << "Segment from " << segment->plusPlace().id() << " to " << segment->minusPlace().id()
            << " idx:" << pIndices.first << " to " << pIndices.second << '\n';
#endif

        const std::vector<Lambda>& lambdas = segment->getAllLambdas();
        for(auto lambdaIt = lambdas.begin(), lambdaEnd = lambdas.end(); lambdaIt != lambdaEnd; ++lambdaIt)
        {
            const Lambda& segmentLambda = *lambdaIt;

            double xVar = segmentLambda.xVariance;
            double yVar = segmentLambda.yVariance;
            double thetaVar = segmentLambda.thetaVariance;

            variance.push_back(xVar);
            variance.push_back(yVar);
            variance.push_back(thetaVar);

            x.push_back(segmentLambda.x / xVar);
            x.push_back(segmentLambda.y / yVar);
            x.push_back(segmentLambda.theta / thetaVar);

            xLambdaIndices.push_back(pIndices);
            ++xIndex;

#ifdef DEBUG_X
            std::cout<<'('<<segmentLambda.x<<','<<segmentLambda.y<<','<<segmentLambda.theta<<")\n";
#endif
        }
    }

#ifdef DEBUG_X
    std::cout<<"DEBUG:LevMar:x-vector (measurements):\n";
    for(std::size_t n = 0; n < numEdges; ++n)
    {
        std::cout<<'('<<x[n*3]<<','<<x[n*3 + 1]<<','<<x[n*3 + 2]<<")\n";
    }

    std::cout<<"DEBUG:LevMar:Measurement variance:\n";
    for(std::size_t i = 0; i < numEdges; ++i)
    {
        for(std::size_t j = 0; j < 3; ++j)
        {
            std::cout<<variance[i*3 + j]<<' ';
        }
        std::cout<<'\n';
    }
#endif
}


void LevMarOptimizer::countEdges(const TopologicalMap& map)
{
    numEdges = 0;

    for(auto& segment : boost::adaptors::values(map.segments()))
    {
        if(!segment->isFrontier())
        {
            numEdges += segment->getAllLambdas().size();
        }
    }
}


void LevMarOptimizer::runLevmar(void)
{
    // Need to have at least as many edges as places in order to optimizer, otherwise graph is tree-structured
//     if(numEdges < numPlaces)
//     {
//         return;
//     }

    assert(xLambdaIndices.size() == numEdges);

    if(work.size() < LM_DIF_WORKSZ(p.size(), x.size()))
    {
        work.resize(LM_DIF_WORKSZ(p.size(), x.size()));
        std::fill(work.begin(), work.end(), 0.0);
    }

    double opts[5];
    double info[LM_INFO_SZ];

    opts[0] = params_.initialMu;
    opts[1] = params_.stopThreshold;
    opts[2] = params_.stopThreshold;
    opts[3] = params_.stopThreshold;
    opts[4] = 1.0;

//     dlevmar_der(levmar_func, levmar_jacobian, p, x, numPlaces*3, numEdges*3, params_.maxIterations, opts, info, work, covariance, this);
//     dlevmar_dif(levmar_func, p, x, numPlaces*3, numEdges*3, params_.maxIterations, opts, info, work, covariance, this);
    dlevmar_dif(levmar_func,
                p.data(),
                x.data(),
                // handle the case where there's a single area, so it needs to have the error computed, which will be large
                (p.size() > 3) ? p.size() - 3 : p.size(), // ignoring the final place, which will always be (0,0,0)
                x.size(),
                params_.maxIterations,
                opts,
                info,
                work.data(),
                covariance.data(),
                this);

    finalError = info[1];

#ifdef DEBUG_JACOBIAN
    std::vector<double> error(x.size());
    dlevmar_chkjac(levmar_func,
                   levmar_jacobian,
                   p.data(),
                   p.size() - 3,
                   x.size(),
                   this,
                   error.data());

    std::cout<<"DEBUG:LevMar:Jacobian check:\n";
    for(std::size_t n = 0; n < error.size(); ++n)
    {
        std::cout << error[n] << '\n';
    }
#endif

#ifdef DEBUG_LEV_MAR
    std::cout<<"DEBUG:LevMar:Optimization info:\n"
             <<"Initial error:"<<info[0]<<'\n'
             <<"Final error:  "<<info[1]<<'\n'
             <<"Iterations:   "<<info[5]<<'\n'
             <<"Termination:  "<<info[6]<<'\n'
             <<"Func evals:   "<<info[7]<<'\n'
             <<"Jacob evals:  "<<info[8]<<'\n'
             <<"Sys solved:   "<<info[9]<<'\n';
#endif
}


Chi LevMarOptimizer::createChiFromP(void)
{
    std::unordered_map<Id, pose_distribution_t> finalPoses;

    for(auto& index : idToPIndex)
    {
        int idx = index.second * 3;

        pose_distribution_t pose;
        pose.x = p[idx];
        pose.y = p[idx + 1];
        pose.theta = wrap_to_pi(p[idx + 2]);
        pose.uncertainty[0] = pose.x;
        pose.uncertainty[1] = pose.y;
        pose.uncertainty[2] = pose.theta;

        // Copy out the covariance off the block diagonal -- don't care about the other cross-correlation terms
        for(int y = 0; y < 3; ++y)
        {
            for(int x = 0; x < 3; ++x)
            {
                pose.uncertainty(x, y) = covariance[((idx + y) * numPlaces * 3) + idx + x];
            }
        }

        finalPoses[index.first] = pose;
    }

#ifdef DEBUG_FINAL_P
    std::cout << "DEBUG::LevMar: Mean poses:\n";
    for(auto& idToPose : finalPoses)
    {
        std::cout << idToPose.first << ":\n" << idToPose.second.uncertainty.getMean() << '\n'
            << idToPose.second.uncertainty.getCovariance();
    }
#endif

#ifdef DEBUG_COVARIANCE
//     std::cout<<"DEBUG:LevMar:Covariance:\n";
//     for(std::size_t i = 0; i < numPlaces*3; ++i)
//     {
//         for(std::size_t j = 0; j < numPlaces*3; ++j)
//         {
//             printf("%.2f ", covariance[i*numPlaces*3 + j]);
// //             std::cout<<covariance[i*numPlaces*3 + j]<<' ';
//         }
//         std::cout<<'\n';
//     }
    std::cout<<"DEBUG:LevMar:Covariance diag:\n";
    for(std::size_t i = 0; i < numPlaces*3; ++i)
    {
        printf("%.2f ", dlevmar_stddev(covariance.data(), numPlaces * 3, i));//covariance[i*numPlaces*3 + i*3]);
        std::cout<<'\n';
    }
#endif

    // Error minimizes (lambda_chi - lambda_obs)^T cov_obs^-1 (lambda_chi - lambda_obs)
    // To get log-likelihood, just multiply by -0.5, which gives the Gaussian log-likelihood
    return Chi(finalPoses, -0.5*finalError);
}


void LevMarOptimizer::calculateLambdas(double* newP, double* hx, int m, int n)
{
    pose_t start;
    pose_t end;

    for(std::size_t i = 0; i < xLambdaIndices.size(); ++i)
    {
        // Only consider the new poses if they have valid indices. If out of range, that means it should be 0,0,0
        // per the design of the p matrix
        if(xLambdaIndices[i].first * 3 < m)
        {
            int startIdx = xLambdaIndices[i].first*3;
            start.x = newP[startIdx];
            start.y = newP[startIdx + 1];
            start.theta = wrap_to_pi(newP[startIdx + 2]);
        }
        // At the origin
        else
        {
            start.x = 0.0f;
            start.y = 0.0f;
            start.theta = 0.0f;
        }

        if(xLambdaIndices[i].second * 3 < m)
        {
            int endIdx = xLambdaIndices[i].second*3;
            end.x = newP[endIdx];
            end.y = newP[endIdx + 1];
            end.theta = wrap_to_pi(newP[endIdx + 2]);
        }
        // At the origin
        else
        {
            end.x = 0.0f;
            end.y = 0.0f;
            end.theta = 0.0f;
        }

        auto diff = end.transformToNewFrame(start);

        int xIdx = i * 3;
        int yIdx = xIdx + 1;
        int thetaIdx = xIdx + 2;

        hx[xIdx] = diff.x / variance[xIdx];
        hx[yIdx] = diff.y / variance[yIdx];
        hx[thetaIdx] = diff.theta / variance[thetaIdx];

        xError[xIdx] = x[xIdx] - hx[xIdx];
        xError[yIdx] = x[yIdx] - hx[yIdx];
        xError[thetaIdx] = wrap_to_pi(x[thetaIdx] - hx[thetaIdx]);
    }

#ifdef DEBUG_INCREMENTAL_P
    std::cout<<"DEBUG:LevMar:Incremental p-vector:\n";
    for(int i = 0; i*3 < m; ++i)
    {
        std::cout<<'('<<newP[i*3]<<','<<newP[i*3 + 1]<<','<<newP[i*3 + 2]<<")\n";
    }
#endif

#ifdef DEBUG_LAMBDA
    std::cout<<"DEBUG:LevMar:Incremental lambdas (hx-vector):\n";
    for(std::size_t i = 0; i < numEdges; ++i)
    {
        std::cout<<'('<<hx[i*3]<<','<<hx[i*3 + 1]<<','<<hx[i*3 + 2]<<")\n";
    }

    std::cout<<"DEBUG:LevMar:Residuals (x-hx):\n";
    for(std::size_t i = 0; i < numEdges; ++i)
    {
        std::cout<<'('<<(x[i*3]-hx[i*3])<<','<<(x[i*3 + 1] - hx[i*3 + 1])<<','<<(x[i*3 + 2] - hx[i*3 + 2])<<")\n";
    }
#endif
}


void LevMarOptimizer::calculateJacobian(double* j, int m, int n)
{
    // The residuals are (p_i - p_j - x_n)^2. Thus the Jacobian is +1 for J_n_i and -1 for J_n_j.

    std::size_t rowStart = 0;
    std::size_t rowStep  = numPlaces*3;

    memset(j, 0, m*m*sizeof(double));

    // The Jacobian is two diagonal blocks for (x,y,theta).  second = end place, first = start place. so lambda = p_end - p_start
    for(std::size_t i = 0; i < numEdges; ++i)
    {
        if(xLambdaIndices[i].second*3 < n)
        {
            j[rowStart + xLambdaIndices[i].second*3] = xError[i*3];
        }
        if(xLambdaIndices[i].first*3 < n)
        {
            j[rowStart + xLambdaIndices[i].first*3]  = -xError[i*3];
        }

        rowStart += rowStep;

        if(xLambdaIndices[i].second*3 < n)
        {
            j[rowStart + xLambdaIndices[i].second*3 + 1] = xError[i*3 + 1];
        }
        if(xLambdaIndices[i].first*3 < n)
        {
            j[rowStart + xLambdaIndices[i].first*3 + 1]  = -xError[i*3 + 1];
        }

        rowStart += rowStep;

        if(xLambdaIndices[i].second*3 < n)
        {
            j[rowStart + xLambdaIndices[i].second*3 + 2] = xError[i*3 + 2];
        }
        if(xLambdaIndices[i].first*3 < n)
        {
            j[rowStart + xLambdaIndices[i].first*3 + 2]  = -xError[i*3 + 2];
        }

        rowStart += rowStep;
    }

#ifdef DEBUG_JACOBIAN
    rowStart = 0;
    std::cout<<"DEBUG:LevMar:Jacobian:\n";
    for(std::size_t n = 0; n < numEdges*3; ++n)
    {
        for(std::size_t i = 0; i < numPlaces*3; ++i)
        {
            std::cout<<j[rowStart + i]<<' ';
        }
        std::cout<<'\n';
        rowStart += rowStep;
    }
#endif
}

// Create friend functions to issue as the callbacks used by levmar. The adata will point to this, which will then
// be used to call the
void levmar_func(double* p, double* hx, int m, int n, void* adata)
{
    LevMarOptimizer* optimizer = static_cast<LevMarOptimizer*>(adata);
    optimizer->calculateLambdas(p, hx, m, n);
}


void levmar_jacobian(double* p, double* j, int m, int n, void* adata)
{
    LevMarOptimizer* optimizer = static_cast<LevMarOptimizer*>(adata);
    optimizer->calculateJacobian(j, m, n);
}

} // namespace hssh
} // namespace vulcan
