/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_utils_test.cpp
* \author   Collin Johnson
*
* Unit tests for ensuring proper functionality of the functions defined in gateway_utils.h.
*/

#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include <gtest/gtest.h>

using namespace vulcan;
using namespace vulcan::hssh;

const double kAngleThreshold    = M_PI / 4.0;
const double kEndpointThreshold = 0.25;

const double kBoundaryLength = 5.0;


Gateway create_gateway(const Line<double>& boundary);


TEST(GatewayAnglesCloseTest, IdenticalGatewaysAreClose)
{
    Line<double> boundary       (0, 0, kBoundaryLength, 0);
    Line<double> reverseBoundary(kBoundaryLength, 0, 0, 0);
    
    Gateway g  = create_gateway(boundary);
    Gateway rg = create_gateway(reverseBoundary);
    
    EXPECT_TRUE(are_gateway_angles_close(g,  rg, kAngleThreshold, kEndpointThreshold));
    EXPECT_TRUE(are_gateway_angles_close(g,  rg, kAngleThreshold, kEndpointThreshold));
    EXPECT_TRUE(are_gateway_angles_close(rg, g,  kAngleThreshold, kEndpointThreshold));
    EXPECT_TRUE(are_gateway_angles_close(rg, rg, kAngleThreshold, kEndpointThreshold));
}


TEST(GatewayAnglesCloseTest, CloseEndpointsAreClose)
{
    Line<double> boundary              (0,                      0, kBoundaryLength,                        0.0);
    Line<double> belowThresholdBoundary(kEndpointThreshold/2.0, 0, kBoundaryLength+kEndpointThreshold/2.0, 0.0);
    Line<double> onThresholdBoundary   (kEndpointThreshold,     0, kBoundaryLength+kEndpointThreshold,     0.0);
    Line<double> aboveThresholdBoundary(kEndpointThreshold*2.0, 0, kBoundaryLength+kEndpointThreshold*2.0, 0.0);
    
    Gateway g     = create_gateway(boundary);
    Gateway below = create_gateway(belowThresholdBoundary);
    Gateway on    = create_gateway(onThresholdBoundary);
    Gateway above = create_gateway(aboveThresholdBoundary);
    
    EXPECT_TRUE(are_gateway_angles_close (g, below, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, on,    kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, above, kAngleThreshold, kEndpointThreshold));
}


TEST(GatewayAnglesCloseTest, CloseAnglesAreClose)
{
    const double closeAngle = kAngleThreshold * 0.5;
    const double onAngle    = kAngleThreshold;
    const double aboveAngle = kAngleThreshold  * 1.5;
    
    Line<double> boundary                 (0, 0, kBoundaryLength,                       0);
    Line<double> belowPosThresholdBoundary(0, 0, kBoundaryLength*std::cos(closeAngle),  kBoundaryLength*std::sin(closeAngle));
    Line<double> belowNegThresholdBoundary(0, 0, kBoundaryLength*std::cos(-closeAngle), kBoundaryLength*std::sin(-closeAngle));
    Line<double> onPosThresholdBoundary   (0, 0, kBoundaryLength*std::cos(onAngle),     kBoundaryLength*std::sin(onAngle));
    Line<double> onNegThresholdBoundary   (0, 0, kBoundaryLength*std::cos(-onAngle),    kBoundaryLength*std::sin(-onAngle));
    Line<double> abovePosThresholdBoundary(0, 0, kBoundaryLength*std::cos(aboveAngle),  kBoundaryLength*std::sin(aboveAngle));
    Line<double> aboveNegThresholdBoundary(0, 0, kBoundaryLength*std::cos(-aboveAngle), kBoundaryLength*std::sin(-aboveAngle));
    
    Gateway g        = create_gateway(boundary);
    Gateway belowPos = create_gateway(belowPosThresholdBoundary);
    Gateway belowNeg = create_gateway(belowNegThresholdBoundary);
    Gateway onPos    = create_gateway(onPosThresholdBoundary);
    Gateway onNeg    = create_gateway(onNegThresholdBoundary);
    Gateway abovePos = create_gateway(abovePosThresholdBoundary);
    Gateway aboveNeg = create_gateway(aboveNegThresholdBoundary);
    
    EXPECT_TRUE(are_gateway_angles_close (g, belowPos, kAngleThreshold, kEndpointThreshold));
    EXPECT_TRUE(are_gateway_angles_close (g, belowNeg, kAngleThreshold, kEndpointThreshold));
    EXPECT_TRUE(are_gateway_angles_close (belowPos, g, kAngleThreshold, kEndpointThreshold));
    EXPECT_TRUE(are_gateway_angles_close (belowNeg, g, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, onPos,    kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, onNeg,    kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(onPos, g,    kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(onNeg, g,    kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, abovePos, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, aboveNeg, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(abovePos, g, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(aboveNeg, g, kAngleThreshold, kEndpointThreshold));
}


TEST(GatewayAnglesCloseTest, FarAnglesAreNotClose)
{
    const double closeAngle = kAngleThreshold * 0.5;
    const double farAngle   = angle_sum(closeAngle, M_PI);
    
    Line<double> boundary      (0, 0, kBoundaryLength,                       0);
    Line<double> farBoundary   (0, 0, kBoundaryLength*std::cos(farAngle),  kBoundaryLength*std::sin(farAngle));
    Line<double> farNegBoundary(0, 0, kBoundaryLength*std::cos(-farAngle), kBoundaryLength*std::sin(-farAngle));
    
    Gateway g      = create_gateway(boundary);
    Gateway farPos = create_gateway(farBoundary);
    Gateway farNeg = create_gateway(farNegBoundary);
    
    EXPECT_FALSE(are_gateway_angles_close(g, farPos, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(g, farNeg, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(farPos, g, kAngleThreshold, kEndpointThreshold));
    EXPECT_FALSE(are_gateway_angles_close(farNeg, g, kAngleThreshold, kEndpointThreshold));
}


Gateway create_gateway(const Line<double>& boundary)
{
    // TODO: Fix this test!
    return Gateway();
//     return Gateway(0,
//                    boundary,
//                    Point<double>((boundary.a.x + boundary.b.x) / 2,
//                                        (boundary.a.y + boundary.b.y) / 2),
//                    angle_to_point(boundary.a, boundary.b) + M_PI_2);
}
