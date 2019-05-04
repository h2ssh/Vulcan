/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose_trace_test.cpp
* \author   Collin Johnson
* 
* pose_trace_test is a test program that tests the functionality of PoseTrace.
*/

#include <utils/pose_trace.h>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

using namespace vulcan;

const std::string kTestTraceFile("pose_trace_test.trc");


TEST(PoseTraceAddTest, CanAddData)
{
    utils::PoseTrace trace;
    
    for(int n = 0; n < 100; ++n)
    {
        pose_t pose(100 + n*1000, 0.1 * n, 0.1 * n, 0.1 * n);
        EXPECT_TRUE(trace.addPose(pose));
    }
}


TEST(PoseTraceIOTest, CanSaveData)
{
    utils::PoseTrace trace;
    
    for(int n = 0; n < 100; ++n)
    {
        pose_t pose(100 + n*1000, 0.1 * n, 0.1 * n, 0.1 * n);
        trace.addPose(pose);
    }
    
    EXPECT_TRUE(trace.saveToFile(kTestTraceFile));
}


TEST(PoseTraceIOTest, CanLoadData)
{
    utils::PoseTrace trace;
    
    for(int n = 0; n < 100; ++n)
    {
        pose_t pose(100 + n*1000, 0.1 * n, 0.1 * n, 0.1 * n);
        trace.addPose(pose);
    }
    
    EXPECT_TRUE(trace.saveToFile(kTestTraceFile));
    
    utils::PoseTrace loaded(kTestTraceFile);
    ASSERT_EQ(trace.size(), loaded.size());
    
    for(std::size_t n = 0; n < loaded.size(); ++n)
    {
        EXPECT_EQ(trace[n], loaded[n]);
    }
}


TEST(PoseTraceIOTest, TimestampsAreSaved)
{
    utils::PoseTrace trace;

    for(int n = 0; n < 100; ++n)
    {
        pose_t pose(100 + n*1000, 0.1 * n, 0.1 * n, 0.1 * n);
        trace.addPose(pose);
    }

    EXPECT_TRUE(trace.saveToFile(kTestTraceFile));

    utils::PoseTrace loaded(kTestTraceFile);
    ASSERT_EQ(trace.size(), loaded.size());

    for(std::size_t n = 1; n < loaded.size(); ++n)
    {
        EXPECT_GT(loaded[n].timestamp, loaded[n-1].timestamp);
    }
}