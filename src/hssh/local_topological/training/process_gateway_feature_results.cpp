/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     process_gateway_feature_results.cpp
 * \author   Collin Johnson
 *
 * process_gateway_feature_results is a program that parses the results of a gateway_feature_search program. It then
 * supplies various metrics to help search for the best combination of gateway features.
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

struct ClassifierResult
{
    double threshold;
    double error;
    int numFn;
    int numTp;
    int numFp;
    double precision;
    double recall;
};

struct FeatureSet
{
    int id;
    std::vector<ClassifierResult> results;
    double bestResult;

    bool operator<(const FeatureSet& rhs) const { return bestResult < rhs.bestResult; }
};

std::ifstream& operator>>(std::ifstream& in, FeatureSet& features);
std::istream& operator>>(std::istream& in, ClassifierResult& result);

std::ostream& operator<<(std::ostream& out, const FeatureSet& features);

void assign_max_sum_precision_recall(std::vector<FeatureSet>& sets);
void assign_fn_neg(std::vector<FeatureSet>& sets);


int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Expected command-line: process_gateway_feature_results <gateway results file>\n";
        return -1;
    }

    std::ifstream in(argv[1]);
    if (!in.is_open()) {
        std::cerr << "ERROR: Failed to open results file: " << argv[1] << '\n';
        return -1;
    }

    std::vector<FeatureSet> sets;
    std::string line;
    while (std::getline(in, line)) {
        if (line.length() < 10) {
            continue;
        }

        if (line.find("Feature") != std::string::npos) {
            FeatureSet newSet;

            // Line is: Feature set <id>
            std::istringstream featureIn(line);
            std::string tmp;
            featureIn >> tmp >> tmp >> newSet.id;

            in >> newSet;
            sets.emplace_back(std::move(newSet));
        }
    }

    std::cout << "Read " << sets.size() << " feature sets.\n";

    assign_max_sum_precision_recall(sets);
    std::sort(sets.begin(), sets.end());

    std::ofstream prOut("pr_gateway_results.txt");
    prOut << "PR sum is: " << '\n';
    std::copy(sets.begin(), sets.end(), std::ostream_iterator<FeatureSet>(prOut, "\n"));

    assign_fn_neg(sets);
    std::sort(sets.begin(), sets.end());

    std::ofstream fnOut("fn_gateway_results.txt");
    fnOut << "Num Fn is: " << '\n';
    std::copy(sets.begin(), sets.end(), std::ostream_iterator<FeatureSet>(fnOut, "\n"));

    return 0;
}


std::ifstream& operator>>(std::ifstream& in, FeatureSet& features)
{
    std::string line;
    while (std::getline(in, line)) {
        if (line.length() < 10) {
            break;
        }

        // If we aren't reading the header, then
        if (line.find("Thresh") == std::string::npos) {
            std::istringstream resIn(line);
            ClassifierResult result;
            resIn >> result;
            features.results.push_back(result);
        }
    }

    return in;
}


std::istream& operator>>(std::istream& in, ClassifierResult& result)
{
    in >> result.threshold >> result.error >> result.numFn >> result.numTp >> result.numFp >> result.precision
      >> result.recall;
    return in;
}


std::ostream& operator<<(std::ostream& out, const FeatureSet& features)
{
    out << features.id << " : " << features.bestResult << " Threshold:" << features.results.back().threshold << '\n';
    return out;
}


void assign_max_sum_precision_recall(std::vector<FeatureSet>& sets)
{
    for (auto& s : sets) {
        std::sort(s.results.begin(), s.results.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.precision + lhs.recall < rhs.precision + rhs.recall;
        });

        s.bestResult = s.results.back().precision + s.results.back().recall;
    }
}


void assign_fn_neg(std::vector<FeatureSet>& sets)
{
    for (auto& s : sets) {
        std::sort(s.results.begin(), s.results.end(), [](const auto& lhs, const auto& rhs) {
            return (lhs.numFn > rhs.numFn) || (lhs.numFn == rhs.numFn && lhs.threshold < rhs.threshold);
        });

        s.bestResult = s.results.back().numFn;
    }
}
