/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "utils/boosting.h"
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>


void output_features_for_type(const std::string& type, const std::string& directory);


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "Expected command-line: boosting_feature_selection <classifier dir>\n";
        return -1;
    }

    output_features_for_type("path", argv[1]);
    output_features_for_type("decision", argv[1]);
    output_features_for_type("dest", argv[1]);

    return 0;
}


void output_features_for_type(const std::string& type, const std::string& directory)
{
    using namespace boost::filesystem;
    using namespace vulcan;
    using namespace vulcan::hssh;

    // Iterate through the directory provided by the first command-line argument and load all classifiers
    // containing 'likelihood' and 'ada'
    // Create a set of all features used print them out, along with features not used

    std::map<int, double> weights;
    std::set<int> unusedFeatures;

    HypothesisFeatures allFeatures;
    for(int n = 0, end = allFeatures.numFeatures(); n < end; ++n)
    {
        unusedFeatures.insert(n);
    }

    utils::AdaBoostClassifier classifier;
    for(auto dirIt = directory_iterator(path(directory)), endIt = directory_iterator(); dirIt != endIt; ++dirIt)
    {
        auto entry = dirIt->path().generic_string();

        if((entry.find("likelihood") != std::string::npos)
            && (entry.find(type) != std::string::npos)
            && (entry.find("ada") != std::string::npos))
        {
            if(!classifier.load(entry))
            {
                std::cerr << "ERROR: Failed to load classifier: " << entry << '\n';
                continue;
            }

//             std::cout << "Loading classifier: " << entry << '\n';

            for(auto& stump : classifier)
            {
                if(weights.find(stump.featureIndex()) == weights.end())
                {
                    weights[stump.featureIndex()] = stump.weight();
                }
                else
                {
                    weights[stump.featureIndex()] += stump.weight();
                }
                unusedFeatures.erase(stump.featureIndex());
            }
        }
    }

    std::vector<std::pair<int, double>> usedFeatures;
    for(auto feature : weights)
    {
        usedFeatures.emplace_back(feature.first, feature.second);
    }

    std::sort(usedFeatures.begin(), usedFeatures.end(), [](auto lhs, auto rhs) {
        return lhs.second > rhs.second;
    });

    std::cout << type << " features used by AdaBoost:\nIdx:\tName:\n";
    for(auto feature : usedFeatures)
    {
        std::cout << feature.first << '\t'
            << feature.second << '\t'
            << HypothesisFeatures::featureName(feature.first) << '\n';
    }

    std::cout << "\n\n" << type << " unused features:\nIdx:\tName:\n";
    for(int idx : unusedFeatures)
    {
        std::cout << idx << '\t' << HypothesisFeatures::featureName(idx) << '\n';
    }
}
