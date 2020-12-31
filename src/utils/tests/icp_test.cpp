/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     icp_test.cpp
* \author   Collin Johnson
* 
* A test program for the ICP algorithm to make sure it passes a basic sanity check for
* functionality.
*/

#include "utils/icp.h"
#include "core/point.h"
#include "core/angle_functions.h"
#include "core/pose.h"
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace vulcan;

std::vector<Point<float>> generate_scan(int numPoints);
std::vector<Point<float>> transform_points(const std::vector<Point<float>>& points, const pose_t& transform);
std::vector<Point<float>> randomize_points(const std::vector<Point<float>>& points, float maxNoise);

void test_rotation(void);
void test_translation(void);
void test_rotation_and_translation(void);
bool check_icp_result(const std::vector<Point<float>>& scan,
                      const std::vector<Point<float>>& transformed,
                      const pose_t&                   transform);

/**
* icp_test is a simple test program for the icp_2d algorithm. It generates random range scans,
* applies various transforms or noise, and runs ICP to see how good the result is.
*/
int main(int argc, char** argv)
{
    srand48(std::time(0));
    
    test_rotation();
    test_translation();
    test_rotation_and_translation();
    
    return 0;
}


std::vector<Point<float>> generate_scan(int numPoints)
{
    assert(numPoints > 0);
    
    const float MAX_RANGE = 15.0f;
    
    std::vector<Point<float>> points(numPoints);
    float angle     = 0.0f;
    float increment = 2*M_PI / numPoints;
    
    for(int n = 0; n < numPoints; ++n, angle += increment)
    {
        float range = drand48() * MAX_RANGE;
        
        points[n] = Point<float>(std::cos(angle)*range, std::sin(angle)*range);
    }
    
    return points;
}


std::vector<Point<float>> transform_points(const std::vector<Point<float>>& points, const pose_t& transform)
{
    std::vector<Point<float>> transformed(points.size());
    
    for(std::size_t n = 0; n < points.size(); ++n)
    {
        transformed[n] = transform.toPoint() + rotate(points[n], transform.theta);
    }
    
    return transformed;
}


std::vector<Point<float>> randomize_points(const std::vector<Point<float>>& points, float maxNoise)
{
    std::vector<Point<float>> noisy(points.size());
    
    for(std::size_t n = 0; n < points.size(); ++n)
    {
        // drand48 range is [0,1], so offset by 0.5 to allow the noise to be a negative value
        noisy[n].x = points[n].x + (drand48()-0.5)*maxNoise;
        noisy[n].y = points[n].y + (drand48()-0.5)*maxNoise;
    }
    
    return noisy;
}


void test_rotation(void)
{
    int   NUM_SAMPLES = 20;
    float MIN_ANGLE   = -0.2f;
    float MAX_ANGLE   = 0.2f;
    
    std::vector<Point<float>> scan = generate_scan(360);
    
    int numTests  = 0;
    int numPassed = 0;
    
    for(int n = 0; n < NUM_SAMPLES; ++n)
    {
        pose_t                   transform(0, 0, MIN_ANGLE + ((MAX_ANGLE-MIN_ANGLE)/NUM_SAMPLES) * n);
        std::vector<Point<float>> transformed = transform_points(scan, transform);
        
        if(check_icp_result(scan, transformed, transform))
        {
            ++numPassed;
        }
        
        ++numTests;
    }
    
    if(numPassed == numTests)
    {
        std::cout<<"SUCCESS: Passed all rotation tests!\n";
    }
    else
    {
        std::cout<<"FAILURE: Passed "<<numPassed<<'/'<<numTests<<" rotation tests.\n";
    }
}


void test_translation(void)
{
    int   NUM_SAMPLES = 20;
    float MIN_TRANS   = -0.2f;
    float MAX_TRANS   = 0.2f;
    
    std::vector<Point<float>> scan = generate_scan(360);
    
    int numTests  = 0;
    int numPassed = 0;
    
    for(int n = 0; n < NUM_SAMPLES; ++n)
    {
        pose_t                   transform(MIN_TRANS + ((MAX_TRANS-MIN_TRANS)/NUM_SAMPLES) * n, 0, 0);
        std::vector<Point<float>> transformed = transform_points(scan, transform);
        
        if(check_icp_result(scan, transformed, transform))
        {
            ++numPassed;
        }
        
        ++numTests;
    }
    
    for(int n = 0; n < NUM_SAMPLES; ++n)
    {
        pose_t                   transform(0, MIN_TRANS + ((MAX_TRANS-MIN_TRANS)/NUM_SAMPLES) * n, 0);
        std::vector<Point<float>> transformed = transform_points(scan, transform);
        
        if(check_icp_result(scan, transformed, transform))
        {
            ++numPassed;
        }
        
        ++numTests;
    }
    
    if(numPassed == numTests)
    {
        std::cout<<"SUCCESS: Passed all translation tests!\n";
    }
    else
    {
        std::cout<<"FAILURE: Passed "<<numPassed<<'/'<<numTests<<" translation tests.\n";
    }
}


void test_rotation_and_translation(void)
{
    
}


bool check_icp_result(const std::vector<Point<float>>& scan,
                      const std::vector<Point<float>>& transformed,
                      const pose_t&                   transform)
{
    pose_t icpResult = utils::icp_2d(scan, transformed, pose_t(0, 0, 0));
    
    float xError     = std::abs(icpResult.x-transform.x);
    float yError     = std::abs(icpResult.y-transform.y);
    float thetaError = std::abs(angle_diff(icpResult.theta,transform.theta));
    
    bool doesPassTest = (xError < 0.01) && (yError < 0.01) && (thetaError < 0.01);
    
    if(!doesPassTest)
    {
        std::cout<<"Failed: Trans:"<<transform<<" ICP:"<<icpResult<<" Error:"<<xError<<','<<yError<<','<<thetaError<<'\n';
    }
    
    return doesPassTest;
}
