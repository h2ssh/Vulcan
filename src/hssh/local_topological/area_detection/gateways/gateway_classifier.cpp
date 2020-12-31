/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_classifier.cpp
* \author   Collin Johnson
*
* Definition of GatewayClassifier.
*/

#include "hssh/local_topological/area_detection/gateways/gateway_classifier.h"
#include "hssh/local_topological/area_detection/gateways/feature_extraction.h"
#include "hssh/local_topological/training/labeled_gateway_data.h"
#include "utils/histogram.h"
#include "utils/plot2d.h"
#include "math/statistics.h"
#include "utils/boosting.h"
#include "utils/feature_vector.h"
#include "utils/timestamp.h"
#include <random>
#include <cassert>

namespace vulcan
{
namespace hssh
{

namespace
{
const int kNumClasses = 2;
const int kGatewayClass = 1;
const int kNotGatewayClass = -1;

const double kGwyThreshold = 0.15;

const std::string kGatewayExtension(".gwy");
const std::string kAdaBoostExtension(".ada");

std::string adaboost_name(std::string filename) { return filename + kGatewayExtension + kAdaBoostExtension; }

void assign_labels_for_examples(const LabeledGatewayData& examples, IntVector& labels);

Vector estimate_gateway_distribution(const LabeledGatewayData& examples);
LabeledGatewayFeatures sample_gateway_vector(const LabeledGatewayFeatures& features, const Vector& stddev);
}


std::unique_ptr<GatewayClassifier> GatewayClassifier::LearnClassifier(const LabeledGatewayData& examples)
{
    assert(!examples.empty());

    // Put the number of gateways on-par with the number of negative examples
    int numGateways = std::count_if(examples.begin(), examples.end(), [](const auto& e) { return e.isGateway; });
    assert(numGateways > 0);
    int gatewayDuplicateCount = std::max((examples.size() - numGateways) / numGateways, std::size_t(1));
    auto gatewayStddev = estimate_gateway_distribution(examples);

    LabeledGatewayData filteredExamples;
    for(auto& e : examples)
    {
        filteredExamples.addExample("filtered", e);

        if(e.isGateway)
        {
            for(int n = 0; n < gatewayDuplicateCount; ++n)
            {
                filteredExamples.addExample("filtered", sample_gateway_vector(e, gatewayStddev));
            }
        }
    }

    IntVector labels(filteredExamples.size());
    assign_labels_for_examples(filteredExamples, labels);

    // Put all features into a single matrix
    Matrix features(filteredExamples[0].features.numFeatures(), filteredExamples.size());
    for(std::size_t n = 0; n < filteredExamples.size(); ++n)
    {
        features.col(n) = filteredExamples[n].features.features();
    }

    auto boostingClassifier = utils::AdaBoostClassifier::LearnClassifier(labels, features, 500);
    if(!boostingClassifier)
    {
        std::cerr << "ERROR: Failed to learn boosting classifier.\n";
    }
    else
    {
        std::cout << "SUCCESS! Learned boosting classifier. Quality is:";
        int numPosCorrect = 0;
        int numNegCorrect = 0;
        int totalPos = 0;
        int totalNeg = 0;

        for(arma::uword n = 0; n < labels.n_elem; ++n)
        {
            double result = boostingClassifier->classify(features.col(n));
            if(labels(n) > 0)
            {
                if(result > 0.5)
                {
                    ++numPosCorrect;
                }

                ++totalPos;
            }
            else // if(boostingLabels(n) < 0)
            {
                if(result < 0.5)
                {
                    ++numNegCorrect;
                }

                ++totalNeg;
            }
        }

        std::cout << numPosCorrect << " of " << totalPos << " positives and " << numNegCorrect << " of " << totalNeg
            << " negatives.\n";
    }

    // Can't use make_unique because using a private constructor
    return std::unique_ptr<GatewayClassifier>(new GatewayClassifier(std::move(boostingClassifier)));
}


GatewayClassifier::GatewayClassifier(const std::string& filename)
: threshold_(kGwyThreshold)
{
    load(filename);
}


GatewayClassification GatewayClassifier::classifyGateway(const utils::FeatureVector& features,
                                                         ClassifierType type) const
{
    return adaboostClassification(features.features());
}


bool GatewayClassifier::save(const std::string& filename) const
{
    if(adaboostModel_ && !adaboostModel_->save(adaboost_name(filename)))
    {
        std::cerr << "ERROR: GatewayClassifier::save: Failed to save AdaBoost model to " << adaboost_name(filename)
            << '\n';
        return false;
    }

    return true;
}


bool GatewayClassifier::load(const std::string& filename)
{
    adaboostModel_ = std::make_shared<utils::AdaBoostClassifier>();

    if(adaboostModel_ && !adaboostModel_->load(adaboost_name(filename)))
    {
        std::cerr << "ERROR: GatewayClassifier::load: Failed to load AdaBoost model to " << adaboost_name(filename)
            << '\n';
        return false;
    }
    else
    {
        std::cout << "INFO: GatewayClassifier: Loaded AdaBoost classifier " << adaboost_name(filename) << ":\n";
        for(auto& stump : *adaboostModel_)
        {
            std::cout << gateway_feature_name(stump.featureIndex()) << " w:" << stump.weight() << '\n';
        }
    }

    return true;
}


GatewayClassifier::GatewayClassifier(std::unique_ptr<utils::AdaBoostClassifier>&& adaboostModel)
: threshold_(kGwyThreshold)
, adaboostModel_(std::move(adaboostModel))
{
}


GatewayClassification GatewayClassifier::adaboostClassification(const Vector& features) const
{
    if(!adaboostModel_)
    {
        return GatewayClassification{false, 0.0};
    }

    double prob = adaboostModel_->classify(features);
    return GatewayClassification{prob >= threshold_, prob};
}


namespace
{
void assign_labels_for_examples(const LabeledGatewayData& examples, IntVector& labels)
{
    auto labelIt = labels.begin();
    for(auto& e : examples)
    {
        *labelIt++ = e.isGateway ? kGatewayClass : kNotGatewayClass;
    }
}


Vector estimate_gateway_distribution(const LabeledGatewayData& examples)
{
    Vector stddev(examples.begin()->features.numFeatures());
    std::vector<double> values;
    for(arma::uword n = 0; n < stddev.n_rows; ++n)
    {
        values.clear();
        for(auto& e : examples)
        {
            if(e.isGateway)
            {
                values.push_back(e.features[n]);
            }
        }

        stddev(n) = std::sqrt(math::variance(values.begin(), values.end()));
    }

    stddev /= 1000000.0;
    return stddev;
}


LabeledGatewayFeatures sample_gateway_vector(const LabeledGatewayFeatures& features, const Vector& stddev)
{
    LabeledGatewayFeatures sampled = features;
    sampled.features = features.features.sample(stddev);
    return sampled;
}

} // anonymous

} // namespace hssh
} // namespace vulcan
