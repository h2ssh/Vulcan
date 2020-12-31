/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     endpoint_model_test.cpp
 * \author   Collin Johnson
 *
 * Unit tests for EndpointModel. Tests that stationary, rotating, and linear motion endpoints all work properly.
 */

#include "math/univariate_gaussian.h"
#include "tracker/objects/endpoint_model.h"
#include <gtest/gtest.h>
#include <iostream>

using namespace vulcan;
using namespace vulcan::tracker;

////// Constants for all models ////////////
const int kNumMeasurements = 20;

////// Constants for linear tests /////////////
const Position kEndpointPosition(0.5f, 0.5f);
const double kLinearDistance = 0.5;

///// Constants for rotating tests //////////////
const double kSmallRotation = M_PI / 9.0;
const double kMediumRotation = M_PI / 4.0;
const double kLargeRotation = M_PI / 2.0;
const double kSmallRadius = 0.25;
const double kMediumRadius = 0.5;
const double kLargeRadius = 1.0;

const std::array<double, 3> kRotations = {kSmallRotation, kMediumRotation, kLargeRotation};
const std::array<double, 3> kRadii = {kSmallRadius, kMediumRadius, kLargeRadius};

// These create models for evaluating the different types of endpoints that are expected
EndpointModel create_single_position_endpoint(double variance);
EndpointModel create_linear_endpoint(double length, double variance);
EndpointModel create_rotating_endpoint(double angleRange, double radius, double radiusVariance, double angleVariance);

double sampleValue(double mean, double variance);


TEST(StationaryEndpointTest, OnePositionIsUndetermined)
{
    // Test that initial estimate is a stationary position
    Position initialPosition(1.0f, 1.0f);
    EndpointModel singlePosition(initialPosition);
    EXPECT_EQ(EndpointType::undetermined, singlePosition.type());
}


TEST(StationaryEndpointTest, AllSamePositionIsStationary)
{
    // Test that estimate with lots of same position is stationary
    auto noVariance = create_single_position_endpoint(0.0);

    EXPECT_EQ(EndpointType::stationary, noVariance.type());

    // Test that estimate with lots of same position isn't rotating, sliding, or undetermined
    EXPECT_NE(EndpointType::rotating, noVariance.type());
    EXPECT_NE(EndpointType::sliding, noVariance.type());
    EXPECT_NE(EndpointType::undetermined, noVariance.type());
}


TEST(StationaryEndpointTest, SmallPositionVarianceIsStationary)
{
    // Test that estimate with small random sample is stationary
    auto smallVariance = create_single_position_endpoint(0.0001);
    EXPECT_EQ(EndpointType::stationary, smallVariance.type());
}


TEST(StationaryEndpointTest, LargePositionVarianceIsNotStationary)
{
    // Test that estimate with large random sample is stationary -- still fits stationary the best
    auto largeVariance = create_single_position_endpoint(0.5);
    EXPECT_NE(EndpointType::stationary, largeVariance.type());
}


TEST(StationaryEndpointTest, ConstructedPositionIsInitialPosition)
{
    // Test that position with only one position is same as input
    Position initialPosition(1.0f, 1.0f);
    EndpointModel singlePosition(initialPosition);
    EXPECT_EQ(initialPosition.x, singlePosition.position().x);
    EXPECT_EQ(initialPosition.y, singlePosition.position().y);
}


TEST(StationaryEndpointTest, NoVariancePositionMatches)
{
    // Test that position with many inputs of same position yields the correct position
    auto noVariance = create_single_position_endpoint(0.0);
    EXPECT_EQ(kEndpointPosition.x, noVariance.position().x);
    EXPECT_EQ(kEndpointPosition.y, noVariance.position().y);
}


TEST(LinearEndpointTest, NoErrorLineIsSliding)
{
    auto noError = create_linear_endpoint(kLinearDistance, 0.0);
    EXPECT_EQ(EndpointType::sliding, noError.type());
    EXPECT_NE(EndpointType::stationary, noError.type());
    EXPECT_NE(EndpointType::rotating, noError.type());
    EXPECT_NE(EndpointType::undetermined, noError.type());
}


TEST(LinearEndpointTest, SmallVarianceLineIsSliding)
{
    // Test that motion along a line is neither stationary nor rotating
    auto linear = create_linear_endpoint(kLinearDistance, 0.0001);
    EXPECT_EQ(EndpointType::sliding, linear.type());
    EXPECT_NE(EndpointType::stationary, linear.type());
    EXPECT_NE(EndpointType::rotating, linear.type());
    EXPECT_NE(EndpointType::undetermined, linear.type());
}


TEST(LinearEndpointTest, NoErrorLineDistanceIsCorrect)
{
    // Test that distance with no error is correct
    auto linear = create_linear_endpoint(kLinearDistance, 0.0);
    EXPECT_FLOAT_EQ(linear.distanceRange(), kLinearDistance);
}


TEST(RotatingEndpointTest, NoErrorCircleIsRotating)
{
    // Test rotating with no error is good
    for (auto rotation : kRotations) {
        for (auto radius : kRadii) {
            std::cout << "Rotating test (no error): (" << rotation << ',' << radius << ")\n";
            auto noError = create_rotating_endpoint(rotation, radius, 0.0, 0.0);
            if (noError.distanceRange() > EndpointModel::kMaxStationaryExtremaDistance) {
                EXPECT_EQ(EndpointType::rotating, noError.type());
                EXPECT_NE(EndpointType::stationary, noError.type());
                EXPECT_NE(EndpointType::sliding, noError.type());
            } else {
                EXPECT_EQ(EndpointType::stationary, noError.type());
                EXPECT_NE(EndpointType::rotating, noError.type());
                EXPECT_NE(EndpointType::sliding, noError.type());
            }
        }
    }
}


TEST(RotatingEndpointTest, SmallVarianceCircleIsRotating)
{
    // Test rotating with error is good
    for (auto rotation : kRotations) {
        for (auto radius : kRadii) {
            std::cout << "Rotating test (error): (" << rotation << ',' << radius << ")\n";
            auto error = create_rotating_endpoint(rotation, radius, 0.0001, 0.0001);
            if (error.distanceRange() > EndpointModel::kMaxStationaryExtremaDistance) {
                EXPECT_EQ(EndpointType::rotating, error.type());
                EXPECT_NE(EndpointType::stationary, error.type());
                EXPECT_NE(EndpointType::sliding, error.type());
            } else {
                EXPECT_EQ(EndpointType::stationary, error.type());
                EXPECT_NE(EndpointType::rotating, error.type());
                EXPECT_NE(EndpointType::sliding, error.type());
            }
        }
    }
}


TEST(RotatingEndpointTest, RotatingModelIsCorrect)
{
    // TODO
}


EndpointModel create_single_position_endpoint(double variance)
{
    EndpointModel model(kEndpointPosition);

    for (int n = 0; n < kNumMeasurements; ++n) {
        model.addPositionMeasurement(
          Position(sampleValue(kEndpointPosition.x, variance), sampleValue(kEndpointPosition.y, variance)));
    }

    return model;
}


EndpointModel create_linear_endpoint(double length, double variance)
{
    const double kLengthStep = length / (kNumMeasurements - 1);

    EndpointModel model(kEndpointPosition);

    std::cout << "Line points:\n";
    for (int n = 0; n < kNumMeasurements; ++n) {
        auto point = Position(sampleValue(kEndpointPosition.x + kLengthStep * n, variance),
                              sampleValue(kEndpointPosition.y, variance));
        model.addPositionMeasurement(point);

        std::cout << point << '\n';
    }

    return model;
}


EndpointModel create_rotating_endpoint(double angleRange, double radius, double radiusVariance, double angleVariance)
{
    const double kAngleStep = angleRange / (kNumMeasurements - 1);

    EndpointModel model(Position(kEndpointPosition.x + radius, kEndpointPosition.y));

    for (int n = 0; n < kNumMeasurements; ++n) {
        double radiusSample = sampleValue(radius, radiusVariance);
        double angleSample = sampleValue(kAngleStep * n, angleVariance);
        model.addPositionMeasurement(Position(kEndpointPosition.x + std::cos(angleSample) * radiusSample,
                                              kEndpointPosition.y + std::sin(angleSample) * radiusSample));
    }

    return model;
}


double sampleValue(double mean, double variance)
{
    static math::UnivariateGaussianDistribution dist;

    if (variance > 0.0) {
        dist.setMean(mean);
        dist.setVariance(variance);
        return dist.sample();
    } else {
        return mean;
    }
}
