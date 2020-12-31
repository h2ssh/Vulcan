/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     likelihood_only_classifier.cpp
 * \author   Collin Johnson
 *
 * Definition of HypothesisClassifier.
 */

#include "hssh/local_topological/area_detection/labeling/hypothesis_classifier.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "hssh/local_topological/area_detection/labeling/type_distribution.h"
#include "hssh/local_topological/training/labeled_area_data.h"
#include "utils/timestamp.h"

// #define DEBUG_LIKELIHOODS

namespace vulcan
{
namespace hssh
{

const bool kTrainIsovists = false;
const bool kTrainAreas = true;
const int kMaxStumps = 500;

const std::string kLikelihoodExtension(".likelihood");
const std::string kPathExtension(".path");
const std::string kDestExtension(".dest");
const std::string kDecisionExtension(".decision");
const std::string kIsovistExtension(".iso");
const std::string kBoostingExtension(".ada");

namespace
{
std::string boosting_name(std::string filename)
{
    return filename + kBoostingExtension;
}
std::string boosting_isovist_name(std::string filename)
{
    return filename + kIsovistExtension + kBoostingExtension;
}
}   // namespace

void assign_labels_for_examples(const LabeledAreaData& examples,
                                HypothesisType type,
                                IntVector& labels,
                                IntVector& isovistLabels);
utils::AdaBoostModelPtr learn_feature_classifier(std::string classifierName,
                                                 const Matrix& features,
                                                 const IntVector& labels,
                                                 bool learnAdaBoost);
HypothesisTypeDistribution probs_to_dist(const Vector& probs);
void print_boosting_features(const utils::AdaBoostClassifier& classifier);
void print_isovist_features(const utils::AdaBoostClassifier& classifier);


std::unique_ptr<HypothesisClassifier> HypothesisClassifier::LearnClassifier(const LabeledAreaData& examples)
{
    assert(!examples.empty());

    const int numFeatures = examples[0].features.numFeatures();
    const int numExamples = examples.size();

    Matrix features(numFeatures, numExamples);

    for (std::size_t n = 0; n < examples.size(); ++n) {
        features.col(n) = examples[n].features.featureVector();
    }

    int numIsovists = 0;
    int numIsovistFeatures = examples[0].features.numIsovistFeatures();

    for (std::size_t n = 0; n < examples.size(); ++n) {
        numIsovists += examples[n].features.numIsovists();
    }

    Matrix isovistFeatures(numIsovistFeatures, numIsovists);

    int isovistIndex = 0;
    for (auto& e : examples) {
        int numExampleIsovists = e.features.numIsovists();
        isovistFeatures.cols(isovistIndex, isovistIndex + numExampleIsovists - 1) = e.features.isovistFeatures();
        isovistIndex += numExampleIsovists;
    }

    // Creating a classifier requiries:
    //  - Assigning the labels for the features, both regular and isovist
    //  - Learning the classifier

    IntVector labels(numExamples);
    IntVector isovistLabels(numIsovists);

    utils::AdaBoostModelPtr areaClassifier;

    // Create the path classifier
    type_classifier_t pathClassifier;
    pathClassifier.type = HypothesisType::kPath;
    assign_labels_for_examples(examples, HypothesisType::kPath, labels, isovistLabels);

    pathClassifier.areaClassifier = learn_feature_classifier("path classifier", features, labels, kTrainAreas);
    pathClassifier.isovistClassifier =
      learn_feature_classifier("path isovist classifier", isovistFeatures, isovistLabels, kTrainIsovists);

    // Create the dest classifier
    type_classifier_t destClassifier;
    destClassifier.type = HypothesisType::kDest;
    assign_labels_for_examples(examples, HypothesisType::kDest, labels, isovistLabels);

    destClassifier.areaClassifier = learn_feature_classifier("dest classifier", features, labels, kTrainAreas);
    destClassifier.isovistClassifier =
      learn_feature_classifier("dest isovist classifier", isovistFeatures, isovistLabels, kTrainIsovists);

    // Create the decision classifier
    type_classifier_t decisionClassifier;
    decisionClassifier.type = HypothesisType::kDecision;
    assign_labels_for_examples(examples, HypothesisType::kDecision, labels, isovistLabels);

    decisionClassifier.areaClassifier = learn_feature_classifier("decision classifier", features, labels, kTrainAreas);
    decisionClassifier.isovistClassifier =
      learn_feature_classifier("decision isovist classifier", isovistFeatures, isovistLabels, kTrainIsovists);

    return std::unique_ptr<HypothesisClassifier>(
      new HypothesisClassifier(std::move(pathClassifier), std::move(destClassifier), std::move(decisionClassifier)));
}


HypothesisClassifier::HypothesisClassifier(const std::string& filename)
{
    bool success = load(filename);
    assert(success);
}


HypothesisType HypothesisClassifier::classify(const HypothesisFeatures& features) const
{
    return calculateDistribution(features).mostAppropriate();
}


HypothesisTypeDistribution HypothesisClassifier::calculateDistribution(const HypothesisFeatures& features) const
{
    auto boosting = boostingVotes(features.featureVector());
    auto isovist = isovistVotes(features.isovistFeatures());

    boosting /= accu(boosting);
    isovist /= accu(isovist);

    Vector dist = boosting % isovist;

    HypothesisTypeDistribution distribution = probs_to_dist(dist);
    distribution.normalize();
    return distribution;
}


HypothesisTypeDistribution HypothesisClassifier::calculateRawDistribution(const HypothesisFeatures& features) const
{
    auto boosting = boostingVotes(features.featureVector());
    auto isovist = isovistVotes(features.isovistFeatures());
    return probs_to_dist(boosting % isovist);
}


HypothesisTypeDistribution
  HypothesisClassifier::calculateRawBoostingDistribution(const HypothesisFeatures& features) const
{
    return probs_to_dist(boostingVotes(features.featureVector()));
}


HypothesisTypeDistribution
  HypothesisClassifier::calculateRawIsovistDistribution(const HypothesisFeatures& features) const
{
    return probs_to_dist(isovistVotes(features.isovistFeatures()));
}


bool HypothesisClassifier::save(const std::string& filename) const
{
    HypothesisFeatures version;
    std::ostringstream baseStr;
    baseStr << filename << '_' << version.version() << kLikelihoodExtension;
    std::string baseName = baseStr.str();

    std::ostringstream isoStr;
    isoStr << filename << '_' << version.isovistVersion() << kLikelihoodExtension;
    std::string isoBaseName = isoStr.str();

    return saveClassifier(pathClassifier_, baseName, isoBaseName, kPathExtension)
      && saveClassifier(destClassifier_, baseName, isoBaseName, kDestExtension)
      && saveClassifier(decisionClassifier_, baseName, isoBaseName, kDecisionExtension);
}


bool HypothesisClassifier::load(const std::string& filename)
{
    HypothesisFeatures version;
    std::ostringstream baseStr;
    baseStr << filename << '_' << version.version() << kLikelihoodExtension;
    std::string baseName = baseStr.str();

    std::ostringstream isoStr;
    isoStr << filename << '_' << version.isovistVersion() << kLikelihoodExtension;
    std::string isoBaseName = isoStr.str();

    bool pathSuccess = loadClassifier(baseName, isoBaseName, kPathExtension, pathClassifier_);
    pathClassifier_.type = HypothesisType::kPath;

    if (!pathSuccess) {
        std::cerr << "ERROR: HypothesisClassifier: Failed to load path classifier at:" << baseName << kPathExtension
                  << '\n';
    } else {
        if (pathClassifier_.areaClassifier) {
            std::cout << "INFO: HypothesisClassifier: Path boosting features:\n";
            print_boosting_features(*pathClassifier_.areaClassifier);
        }

        if (pathClassifier_.isovistClassifier) {
            std::cout << "INFO: HypothesisClassifier: Path isovist boosting features:\n";
            print_isovist_features(*pathClassifier_.isovistClassifier);
        }
    }

    bool destSuccess = loadClassifier(baseName, isoBaseName, kDestExtension, destClassifier_);
    destClassifier_.type = HypothesisType::kDest;

    if (!destSuccess) {
        std::cerr << "ERROR: HypothesisClassifier: Failed to load destination classifier at:" << baseName
                  << kDestExtension << '\n';
    } else {
        if (destClassifier_.areaClassifier) {
            std::cout << "INFO: HypothesisClassifier: Destination boosting features:\n";
            print_boosting_features(*destClassifier_.areaClassifier);
        }

        if (destClassifier_.isovistClassifier) {
            std::cout << "INFO: HypothesisClassifier: Destination isovist features:\n";
            print_isovist_features(*destClassifier_.isovistClassifier);
        }
    }

    bool decisionSuccess = loadClassifier(baseName, isoBaseName, kDecisionExtension, decisionClassifier_);
    decisionClassifier_.type = HypothesisType::kDecision;

    if (!decisionSuccess) {
        std::cerr << "ERROR: HypothesisClassifier: Failed to load decision classifier at:" << baseName
                  << kDecisionExtension << '\n';
    } else {
        if (decisionClassifier_.areaClassifier) {
            std::cout << "INFO: HypothesisClassifier: Decision boosting features:\n";
            print_boosting_features(*decisionClassifier_.areaClassifier);
        }

        if (decisionClassifier_.isovistClassifier) {
            std::cout << "INFO: HypothesisClassifier: Decision isovist boosting features:\n";
            print_isovist_features(*decisionClassifier_.isovistClassifier);
        }
    }

    return pathSuccess && destSuccess && decisionSuccess;
}


HypothesisClassifier::HypothesisClassifier(type_classifier_t&& pathClassifier,
                                           type_classifier_t&& destClassifier,
                                           type_classifier_t&& decisionClassifier)
: pathClassifier_(std::move(pathClassifier))
, destClassifier_(std::move(destClassifier))
, decisionClassifier_(std::move(decisionClassifier))
{
}


Vector HypothesisClassifier::boostingVotes(const Vector& features) const
{
    Vector prob(3);
    prob.zeros();

    prob(0) = pathClassifier_.areaClassifier->classify(features);
    prob(1) = decisionClassifier_.areaClassifier->classify(features);
    prob(2) = destClassifier_.areaClassifier->classify(features);

    return prob;
}


Vector HypothesisClassifier::isovistVotes(const Matrix& isovistFeatures) const
{
    Vector logProb(3);
    logProb.zeros();

    for (arma::uword n = 0; n < isovistFeatures.n_cols; ++n) {
        logProb(0) += pathClassifier_.isovistClassifier->classify(isovistFeatures.col(n));
        logProb(1) += decisionClassifier_.isovistClassifier->classify(isovistFeatures.col(n));
        logProb(2) += destClassifier_.isovistClassifier->classify(isovistFeatures.col(n));
    }

    logProb /= isovistFeatures.n_cols;

    //     logProb -= max(logProb);
    //     logProb = exp(logProb);

    return logProb;
}


bool HypothesisClassifier::saveClassifier(const type_classifier_t& classifier,
                                          const std::string& baseName,
                                          const std::string& isovistBaseName,
                                          const std::string& typeName) const
{
    std::string filename = baseName + typeName;

    if (classifier.areaClassifier && !classifier.areaClassifier->save(boosting_name(filename))) {
        std::cout << "ERROR: HypothesisClassifier::save: Failed to save boosting area classifier model\n";
        return false;
    }

    std::string isoFilename = isovistBaseName + typeName;

    if (classifier.isovistClassifier && !classifier.isovistClassifier->save(boosting_isovist_name(isoFilename))) {
        std::cout << "ERROR: HypothesisClassifier::save: Failed to save boosting isovist classifier model\n";
        return false;
    }

    return true;
}


bool HypothesisClassifier::loadClassifier(const std::string& baseName,
                                          const std::string& isovistBaseName,
                                          const std::string& typeName,
                                          type_classifier_t& classifier)
{
    std::string filename = baseName + typeName;
    std::string isoFilename = isovistBaseName + typeName;

    classifier.areaClassifier = std::make_shared<utils::AdaBoostClassifier>(boosting_name(filename));
    classifier.isovistClassifier = std::make_shared<utils::AdaBoostClassifier>(boosting_isovist_name(isoFilename));
    return true;
}


void assign_labels_for_examples(const LabeledAreaData& examples,
                                HypothesisType type,
                                IntVector& labels,
                                IntVector& isovistLabels)
{
    // Each example either of the specified type or an other type. There are just two classes to divide them into.

    for (std::size_t n = 0; n < examples.size(); ++n) {
        labels(n) = (examples[n].type == type) ? 1 : -1;
    }

    int isovistIndex = 0;
    for (auto& e : examples) {
        int label = (e.type == type) ? 1 : -1;
        int numExampleIsovists = e.features.numIsovists();
        isovistLabels.rows(isovistIndex, isovistIndex + numExampleIsovists - 1).fill(label);
        isovistIndex += numExampleIsovists;
    }
}


utils::AdaBoostModelPtr learn_feature_classifier(std::string classifierName,
                                                 const Matrix& features,
                                                 const IntVector& labels,
                                                 bool learnAdaBoost)
{
    const double kStdDev = 1e-10;

    arma::uvec posIdx = arma::find(labels > 0);
    arma::uvec negIdx = arma::find(labels < 0);

    Matrix finalFeatures = features;
    IntVector finalLabels = labels;

    // If there are more positives than negatives, increase the weight for negative examples
    if (posIdx.n_elem > negIdx.n_elem) {
        int numRandom = posIdx.n_elem - negIdx.n_elem;

        finalLabels.resize(labels.n_rows + numRandom);
        finalLabels.tail(numRandom).fill(1);

        finalFeatures.resize(features.n_rows, features.n_cols + numRandom);
        int nextCol = features.n_cols;

        int numGen = 0;
        while (numGen < numRandom) {
            for (auto idx : negIdx) {
                if ((drand48() > 0.5) && (numGen < numRandom)) {
                    finalFeatures.col(nextCol++) = features.col(idx) + (arma::randn(features.n_rows) * kStdDev);
                    ++numGen;
                }
            }
        }
    }
    // If there are more negatives than positives, increase the weight for positive examples
    else if (negIdx.n_elem > posIdx.n_elem) {
        int numRandom = negIdx.n_elem - posIdx.n_elem;

        finalLabels.resize(labels.n_rows + numRandom);
        finalLabels.tail(numRandom).fill(1);

        finalFeatures.resize(features.n_rows, features.n_cols + numRandom);
        int nextCol = features.n_cols;

        int numGen = 0;
        while (numGen < numRandom) {
            for (auto idx : posIdx) {
                if ((drand48() > 0.5) && (numGen < numRandom)) {
                    finalFeatures.col(nextCol++) = features.col(idx) + (arma::randn(features.n_rows) * kStdDev);
                    ++numGen;
                }
            }
        }
    }

    posIdx = arma::find(finalLabels > 0);
    negIdx = arma::find(finalLabels < 0);

    std::cout << "INFO: HypothesisClassifier::LearnClassifier: Learning classifiers with " << posIdx.n_elem
              << " pos and " << negIdx.n_elem << " neg examples.\n";

    utils::AdaBoostModelPtr boostingModel;

    if (learnAdaBoost) {
        std::cout << "INFO: HypothesisClassifier::LearnClassifier: Starting AdaBoost training for " << classifierName
                  << "...\n";

        int64_t boostingStart = utils::system_time_us();
        boostingModel = utils::AdaBoostClassifier::LearnClassifier(finalLabels, finalFeatures, kMaxStumps);

        std::cout << "INFO: HypothesisClassifier::LearnClassifier: Finished AdaBoost training for " << classifierName
                  << "...Time:" << ((utils::system_time_us() - boostingStart) / 1000) << "ms\n";
    }

    return boostingModel;
}


HypothesisTypeDistribution probs_to_dist(const Vector& probs)
{
    HypothesisTypeDistribution distribution;

    distribution.path = probs(0);
    distribution.decision = probs(1);
    distribution.destination = probs(2);

    return distribution;
}


void print_boosting_features(const utils::AdaBoostClassifier& classifier)
{
    for (auto& stump : classifier) {
        std::cout << HypothesisFeatures::featureName(stump.featureIndex()) << " w:" << stump.weight()
                  << " thresh:" << stump.threshold() << '\n';
    }
}


void print_isovist_features(const utils::AdaBoostClassifier& classifier)
{
    for (auto& stump : classifier) {
        std::cout << HypothesisFeatures::isovistFeatureName(stump.featureIndex()) << " w:" << stump.weight()
                  << " thresh:" << stump.threshold() << '\n';
    }
}

}   // namespace hssh
}   // namespace vulcan
