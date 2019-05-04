/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_graph_feature.cpp
* \author   Collin Johnson
*
* Definition of VisibilityGraphFeature.
*/

#include <utils/visibility_graph_feature.h>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <algorithm>

namespace vulcan
{
namespace utils
{

std::string feature_type_to_string(VisibilityGraphFeatureType type)
{
    switch(type)
    {
    case VisibilityGraphFeatureType::none:
        return "none";

    case VisibilityGraphFeatureType::mean_edge_count:
        return "mean edge count";

    case VisibilityGraphFeatureType::clustering_coeff:
        return "clustering coeff";

    case VisibilityGraphFeatureType::degree_centrality:
        return "degree centrality";

    case VisibilityGraphFeatureType::closeness_centrality:
        return "closeness centrality";

    case VisibilityGraphFeatureType::betweenness_centrality:
        return "betweenness centrality";

    case VisibilityGraphFeatureType::pagerank:
        return "pagerank";

    case VisibilityGraphFeatureType::num_features:
    default:
        break;
    }

    return "";
}


VisibilityGraphFeature::VisibilityGraphFeature(void)
: type_(VisibilityGraphFeatureType::none)
{
}


VisibilityGraphFeature::VisibilityGraphFeature(VisibilityGraphFeatureType type,
                                               const std::vector<value_type>& values)
: type_(type)
, values_(values)
{
}


feature_stats_t VisibilityGraphFeature::stats(std::vector<VisGraphVertex>::const_iterator begin,
                                              std::vector<VisGraphVertex>::const_iterator end) const
{
    using namespace boost::accumulators;
    accumulator_set<double, boost::accumulators::stats<tag::mean, tag::variance, tag::min, tag::max>> statsAcc;

    for(auto& vertToValue : values_)
    {
        if((begin == end) || (std::find(begin, end, vertToValue.first) != end))
        {
            statsAcc(vertToValue.second);
        }
    }

    return { min(statsAcc), mean(statsAcc), max(statsAcc), std::sqrt(variance(statsAcc)) };
}

} // namespace utils
} // namespace vulcan
