/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     transition_cycle_test.cpp
* \author   Collin Johnson
* 
* transition_cycle_test is a test program that tests the functionality of GlobalTransitionCycle.
*/

#include "hssh/global_topological/transition_cycle.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

using namespace vulcan;
using namespace vulcan::hssh;


const AreaType kEvenType = AreaType::path_segment;
const AreaType kOddType = AreaType::destination;
const GlobalArea kPlusArea(next_id(), AreaType::decision_point);


SmallScaleStar create_junction(int numNavigable)
{
    std::vector<local_path_fragment_t> fragments;
    for(int n = 0; n < 4; ++n)
    {
        local_path_fragment_t frag;
        frag.fragmentId = n;
        frag.pathId = n % 2;
        frag.navigable = n < numNavigable;
        frag.type = n % 2 ? kOddType : kEvenType;
        fragments.push_back(frag);
    }
    
    return SmallScaleStar(fragments);
}


SmallScaleStar create_l_junction(void)
{
    return create_junction(2);
}

SmallScaleStar create_t_junction(void)
{
    return create_junction(3);
}


SmallScaleStar create_plus_junction(void)
{
    return create_junction(4);
}


TEST(GlobalTransitionCycleTest, CorrectNavigableTest)
{
    for(int n = 2; n < 4; ++n)
    {
        auto star = create_junction(n);
        int sumNavigable = 0;
        GlobalTransitionCycle cycle(kPlusArea, star);
        
        for(auto& t : cycle)
        {
            if(t.isNavigable())
            {
                ++sumNavigable;
            }
        }
        
        EXPECT_EQ(n, sumNavigable);
    }
}

TEST(GlobalTransitionCycleTest, CorrectTypeTest)
{
    for(int n = 2; n < 4; ++n)
    {
        auto star = create_junction(n);
        GlobalTransitionCycle cycle(kPlusArea, star);
        
        // All transitions up to n have a non-null other area
        for(int i = 0; i < n; ++i)
        {
            EXPECT_EQ((i % 2 ? kOddType : kEvenType), cycle[i].otherArea(kPlusArea).type());
        }
    }
}


TEST(GlobalTransitionCycleTest, LJunctionEqualityTest)
{
    auto lStar = create_l_junction();
    
    GlobalTransitionCycle cycle(kPlusArea, lStar);
    
    for(int n = 1; n < 4; ++n)
    {
        std::vector<local_path_fragment_t> rotated(lStar.getAllFragments());
        std::rotate(rotated.begin(), rotated.begin() + n, rotated.end());
        SmallScaleStar rotatedStar(rotated);
        GlobalTransitionCycle rotatedCycle(kPlusArea, rotatedStar);
        
        // Confirm that the equality operation is correctly symmetric
        EXPECT_EQ(cycle, rotatedCycle);
        EXPECT_EQ(rotatedCycle, cycle);
    }
}


TEST(GlobalTransitionCycleTest, TJunctionEqualityTest)
{
    auto tStar = create_t_junction();
    
    GlobalTransitionCycle cycle(kPlusArea, tStar);
    
    for(int n = 1; n < 4; ++n)
    {
        std::vector<local_path_fragment_t> rotated(tStar.getAllFragments());
        std::rotate(rotated.begin(), rotated.begin() + n, rotated.end());
        SmallScaleStar rotatedStar(rotated);
        GlobalTransitionCycle rotatedCycle(kPlusArea, rotatedStar);
        
        // Confirm that the equality operation is correctly symmetric
        EXPECT_EQ(cycle, rotatedCycle);
        EXPECT_EQ(rotatedCycle, cycle);
    }
}


TEST(GlobalTransitionCycleTest, PlusJunctionEqualityTest)
{
    auto pStar = create_plus_junction();
    
    GlobalTransitionCycle cycle(kPlusArea, pStar);
    
    for(int n = 1; n < 4; ++n)
    {
        std::vector<local_path_fragment_t> rotated(pStar.getAllFragments());
        std::rotate(rotated.begin(), rotated.begin() + n, rotated.end());
        SmallScaleStar rotatedStar(rotated);
        GlobalTransitionCycle rotatedCycle(kPlusArea, rotatedStar);
        
        // Confirm that the equality operation is correctly symmetric
        EXPECT_EQ(cycle, rotatedCycle);
        EXPECT_EQ(rotatedCycle, cycle);
    }
}


TEST(GlobalTransitionCycleTest, NotEqualTest)
{
    auto lStar = create_l_junction();
    auto tStar = create_t_junction();
    auto pStar = create_plus_junction();
    
    GlobalTransitionCycle lCycle(kPlusArea, lStar);
    GlobalTransitionCycle tCycle(kPlusArea, tStar);
    GlobalTransitionCycle pCycle(kPlusArea, pStar);
    
    EXPECT_NE(lCycle, tCycle);
    EXPECT_NE(lCycle, pCycle);
    EXPECT_NE(tCycle, pCycle);
    EXPECT_NE(tCycle, lCycle);
    EXPECT_NE(pCycle, lCycle);
    EXPECT_NE(pCycle, tCycle);
}


TEST(GlobalTransitionCycleTest, NotEqualNotCompatibleTest)
{
    auto lStar = create_l_junction();
    auto tStar = create_t_junction();
    auto pStar = create_plus_junction();
    
    GlobalTransitionCycle lCycle(kPlusArea, lStar);
    GlobalTransitionCycle tCycle(kPlusArea, tStar);
    GlobalTransitionCycle pCycle(kPlusArea, pStar);
    
    // All cycles with different topologies won't be equal
    
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            EXPECT_FALSE(are_cycles_compatible(lCycle, lCycle[n], tCycle, tCycle[i]));
            EXPECT_FALSE(are_cycles_compatible(tCycle, tCycle[n], lCycle, lCycle[i]));
        }
    }
    
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            EXPECT_FALSE(are_cycles_compatible(lCycle, lCycle[n], pCycle, pCycle[i]));
            EXPECT_FALSE(are_cycles_compatible(pCycle, pCycle[n], lCycle, lCycle[i]));
        }
    }
    
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            EXPECT_FALSE(are_cycles_compatible(pCycle, pCycle[n], tCycle, tCycle[i]));
            EXPECT_FALSE(are_cycles_compatible(tCycle, tCycle[n], pCycle, pCycle[i]));
        }
    }
}

TEST(GlobalTransitionCycleTest, NotCompatibleTest)
{
    auto lStar = create_l_junction();
    auto tStar = create_t_junction();
    auto pStar = create_plus_junction();
    
    GlobalTransitionCycle lCycle(kPlusArea, lStar);
    GlobalTransitionCycle tCycle(kPlusArea, tStar);
    GlobalTransitionCycle pCycle(kPlusArea, pStar);
    
    
    
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            EXPECT_FALSE(are_cycles_compatible(lCycle, lCycle[n], tCycle, tCycle[i]));
            EXPECT_FALSE(are_cycles_compatible(tCycle, tCycle[n], lCycle, lCycle[i]));
        }
    }
    
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            EXPECT_FALSE(are_cycles_compatible(lCycle, lCycle[n], pCycle, pCycle[i]));
            EXPECT_FALSE(are_cycles_compatible(pCycle, pCycle[n], lCycle, lCycle[i]));
        }
    }
    
    //
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            EXPECT_FALSE(are_cycles_compatible(pCycle, pCycle[n], tCycle, tCycle[i]));
            EXPECT_FALSE(are_cycles_compatible(tCycle, tCycle[n], pCycle, pCycle[i]));
        }
    }
}

TEST(GlobalTransitionCycleTest, TCompatibilityTest)
{
    auto tStar = create_t_junction();
    GlobalTransitionCycle tCycle(kPlusArea, tStar);
    
    // Not rotations of a T are compatible
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            if(n == i)
            {
                EXPECT_TRUE(are_cycles_compatible(tCycle, tCycle[n], tCycle, tCycle[i]));
            }
            else // if(tCycle[n].isNavigable() && tCycle[i].isNavigable())
            {
                EXPECT_FALSE(are_cycles_compatible(tCycle, tCycle[n], tCycle, tCycle[i]));
            }
        }
    }
}

TEST(GlobalTransitionCycleTest, PlusCompatibilityTest)
{
    auto pStar = create_plus_junction();
    GlobalTransitionCycle pCycle(kPlusArea, pStar);
    
    // Only even rotations of a plus are compatible for this case because of the alternating path-segment/dest
    // construction of pStar
    for(int n = 0; n < 4; ++n)
    {
        for(int i = 0; i < 4; ++i)
        {
            if((n % 2) == (i % 2))
            {
                EXPECT_TRUE(are_cycles_compatible(pCycle, pCycle[n], pCycle, pCycle[i]));
            }
            else // if((n % 2) != (i % 2))
            {
                EXPECT_FALSE(are_cycles_compatible(pCycle, pCycle[n], pCycle, pCycle[i]));
            }
        }
    }
}
