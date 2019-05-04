/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_stump.cpp
* \author   Collin Johnson
*
* Definition of DecisionStump.
*/

#include <utils/decision_stump.h>
#include <utils/algorithm_ext.h>
#include <utils/float_io.h>
#include <iostream>
#include <cassert>

// #define DEBUG_BEST_SPLIT

namespace vulcan
{
namespace utils
{

std::unique_ptr<DecisionStump> DecisionStump::LearnStump(const IntVector& labels,
                                                         const Vector& weights,
                                                         const Matrix& features,
                                                         const std::vector<int>& ignoreFeatures)
{
    double totalWeightPos = arma::accu(weights.elem(find(labels > 0)));
    double totalWeightNeg = arma::accu(weights.elem(find(labels < 0)));

    std::cout << "Pos weight: " << totalWeightPos << " Neg weight:" << totalWeightNeg << '\n';

    double bestEntropy = HUGE_VAL;
    double bestThreshold = 0.0;
    int bestFeature = -1;
    int bestPolarity = 1;

    for(arma::uword feat = 0; feat < features.n_rows; ++feat)
    {
        // If this is a feature to ignore, keep going
        if(contains(ignoreFeatures, feat))
        {
            continue;
        }

        arma::uvec sortIdx = arma::sort_index(features.row(feat), "descend");

        double sumAbovePos = 0.0;
        double sumAboveNeg = 0.0;
        double sumBelowPos = totalWeightPos;
        double sumBelowNeg = totalWeightNeg;

        // All std::max(var, 0.0) are to deal with small floating point numbers going slightly negative.
        for(arma::uword n = 0; n < sortIdx.n_elem; ++n)
        {
            auto idx = sortIdx[n];

            if(labels(idx) > 0)
            {
                sumAbovePos += weights(idx);
                sumBelowPos -= weights(idx);
            }
            else
            {
                sumAboveNeg += weights(idx);
                sumBelowNeg -= weights(idx);
            }

            double abovePosRatio = sumAbovePos;// / totalWeightPos;
            double aboveNegRatio = sumAboveNeg;// / totalWeightNeg;

            double belowPosRatio = sumBelowPos;// / totalWeightPos;
            double belowNegRatio = sumBelowNeg;// / totalWeightNeg;

            double posError = aboveNegRatio + belowPosRatio;  // error if pos examples are > thresh
            double negError = abovePosRatio + belowNegRatio;  // error if pos examples are < thresh

            double error = std::min(posError, negError);

            if(error < bestEntropy)
            {
                auto nextIdx = n + 1 < sortIdx.n_elem ? sortIdx[n + 1] : sortIdx[n];

                bestEntropy = error;
                bestThreshold = (features.row(feat)(idx) + features.row(feat)(nextIdx)) / 2.0;
                bestFeature = feat;
                bestPolarity = (posError > negError) ? -1 : 1;

#ifdef DEBUG_BEST_SPLIT
                std::cout << "DEBUG: DecisionStump: New best split: Total: " << error
                    << " Feat:" << bestFeature << " Thresh:" << bestThreshold
                    << " Above: " << posError << " Below: " << negError
                    << " Pol:" << bestPolarity;
                if(bestPolarity > 0)
                {
                    std::cout << " pos TP:" << abovePosRatio << " neg TP: " << belowNegRatio << '\n';
                }
                else // if(bestPolarity < 0)
                {
                    std::cout << " pos TP:" << belowPosRatio << " neg TP: " << aboveNegRatio << '\n';
                }
#endif

//             double weightAbove = sumAbovePos + sumAboveNeg;
//             weightAbove = std::max(weightAbove, 0.0);
//             double aboveEntropy = (weightAbove > 1e-6) ? 0.0 : HUGE_VAL;
//
//             if(sumAbovePos > 1e-6)
//             {
//                 aboveEntropy -= (sumAbovePos / weightAbove) * std::log(sumAbovePos / weightAbove);
//             }
//             if(sumAboveNeg > 1e-6)
//             {
//                 aboveEntropy -= (sumAboveNeg / weightAbove) * std::log(sumAboveNeg / weightAbove);
//             }
//             aboveEntropy = std::max(aboveEntropy, 0.0);
//
//             double weightBelow = sumBelowPos + sumBelowNeg;
//             weightBelow = std::max(weightBelow, 0.0);
//             double belowEntropy = (weightBelow > 1e-6) ? 0.0 : HUGE_VAL;
//
//             if(sumBelowPos > 1e-6)
//             {
//                 belowEntropy -= (sumBelowPos / weightBelow) * std::log(sumBelowPos / weightBelow);
//             }
//             if(sumBelowNeg > 1e-6)
//             {
//                 belowEntropy -= (sumBelowNeg / weightBelow) * std::log(sumBelowNeg / weightBelow);
//             }
//             belowEntropy = std::max(0.0, belowEntropy);
//
//             double totalEntropy = (weightAbove * aboveEntropy) + (weightBelow * belowEntropy);
//
//             if(totalEntropy < bestEntropy)
//             {
//                 bestEntropy = totalEntropy;
//                 bestThreshold = features.row(feat)(idx);
//                 bestFeature = feat;
//                 bestPolarity = (sumAbovePos + sumBelowNeg) > (sumAboveNeg + sumBelowPos) ? 1 : -1;
//
// #ifdef DEBUG_BEST_SPLIT
//                 std::cout << "DEBUG: DecisionStump: New best split: Total: " << totalEntropy
//                     << " Feat:" << bestFeature << " Thresh:" << bestThreshold
//                     << " Above: " << aboveEntropy << " w:" << weightAbove
//                     << " Below: " << belowEntropy << " w:" << weightBelow
//                     << " Pol:" << bestPolarity;
//                 if(bestPolarity > 0)
//                 {
//                     std::cout << " pos TP:" << (sumAbovePos / totalWeightPos) << " neg TP: "
//                         << (sumBelowNeg / totalWeightNeg) << '\n';
//                 }
//                 else // if(bestPolarity < 0)
//                 {
//                     std::cout << " pos TP:" << (sumBelowPos / totalWeightPos) << " neg TP: "
//                         << (sumAboveNeg / totalWeightNeg) << '\n';
//                 }
// #endif
            }
        }
    }

    if(bestFeature != -1)
    {
        std::cout << "Created stump! Feature:" << bestFeature << " Threshold:" << bestThreshold << " Polarity: "
            << bestPolarity << " Error:" << bestEntropy << '\n';

        auto stump = std::make_unique<DecisionStump>();
        stump->index_ = bestFeature;
        stump->threshold_ = bestThreshold;
        stump->trueWeight_ = (bestPolarity > 0) ? 1.0 : -1.0;
        stump->falseWeight_ = (bestPolarity < 0) ? 1.0 : -1.0;
        stump->polarity_ = bestPolarity;
        return stump;
    }
    else
    {
        return nullptr;
    }
}


double DecisionStump::classify(const Vector& features) const
{
    return (features(index_) >= threshold_) ? trueWeight_ : falseWeight_;
}


void DecisionStump::setWeight(double weight)
{
    assert(weight > 0.0);

    if(polarity_ > 0)
    {
        trueWeight_ = weight;
        falseWeight_ = -weight;
    }
    else
    {
        trueWeight_ = -weight;
        falseWeight_ = weight;
    }
}


double DecisionStump::weight(void) const
{
    return std::max(trueWeight_, falseWeight_);
}


bool DecisionStump::load(std::istream& in)
{
    in >> index_;
    threshold_ = load_floating_point(in);
    trueWeight_ = load_floating_point(in);
    falseWeight_ = load_floating_point(in);
    in >> polarity_;
    return in.good();
}


bool DecisionStump::save(std::ostream& out) const
{
    out << index_ << ' ';
    save_floating_point(out, threshold_);
    out << ' ';
    save_floating_point(out, trueWeight_);
    out << ' ';
    save_floating_point(out, falseWeight_);
    out << ' ' << polarity_ << ' ';
    return out.good();
}

} // namespace utils
} // namespace vulcan
