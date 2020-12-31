/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "math/clustering.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

TEST(NoiseFreeClustering, DBScanFindsClusters)
{
    using namespace vulcan::math;
    
    std::vector<double> noiseFreeData(32);
    std::fill(noiseFreeData.begin(), noiseFreeData.begin() + 10, 0);
    std::fill(noiseFreeData.begin() + 10, noiseFreeData.begin() + 20, 5);
    std::fill(noiseFreeData.begin() + 20, noiseFreeData.begin() + 30, 10);
    std::fill(noiseFreeData.begin() + 30, noiseFreeData.end(), 15);
    
    std::random_shuffle(noiseFreeData.begin(), noiseFreeData.end());
    
    clustering_result_t results = dbscan_1d_linear(noiseFreeData.begin(), noiseFreeData.end(), 1, 3);
    
    std::cout << "DBScan Results for Noise-Free: Num clusters:" << results.numClusters << " Cluster sizes:\n";
    std::copy(results.clusterSizes.begin(), results.clusterSizes.end(), std::ostream_iterator<int>(std::cout, ","));
    
    EXPECT_EQ(3, results.numClusters);
}


TEST(NoiseFreeClustering, KMeansFindsClusters)
{
    using namespace vulcan::math;
    
    std::vector<double> noiseFreeData(32);
    std::fill(noiseFreeData.begin(), noiseFreeData.begin() + 10, 0);
    std::fill(noiseFreeData.begin() + 10, noiseFreeData.begin() + 20, 5);
    std::fill(noiseFreeData.begin() + 20, noiseFreeData.begin() + 30, 10);
    std::fill(noiseFreeData.begin() + 30, noiseFreeData.end(), 15);
    
    std::random_shuffle(noiseFreeData.begin(), noiseFreeData.end());
    
    clustering_result_t results = kmeans_1d_linear(noiseFreeData.begin(), noiseFreeData.end(), 10, 200);
    
    std::cout << "DBScan Results for Noise-Free: Num clusters:" << results.numClusters << " Cluster sizes:\n";
    std::copy(results.clusterSizes.begin(), results.clusterSizes.end(), std::ostream_iterator<int>(std::cout, ","));
    
    EXPECT_EQ(3, results.numClusters);
}
