/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_axes_calculator.cpp
* \author   Collin Johnson
*
* Definition of LPMAxesCalculator.
*/

#include <hssh/utils/lpm_axes_calculator.h>
#include <laser/laser_scan_lines.h>
#include <cmath>
#include <cassert>

namespace vulcan
{
namespace hssh
{

inline double line_direction(const Line<float>& line, const pose_t& pose)
{
    double direction = angle_sum(std::atan(slope(line)), pose.theta);

    // Vertical lines will have slope -pi
    if(direction == -M_PI_2)
    {
        direction = M_PI_2;
    }

    return direction;
}


LPMAxesCalculator::LPMAxesCalculator(const lpm_axes_calculator_params_t& params)
    : params(params)
{
    // Line directions go from (-pi/2, pi/2]
    histogram.resize(2*M_PI/params.binWidth + 1);
}


lpm_axes_t LPMAxesCalculator::calculateAxes(const LocalPerceptualMap& lpm, const laser::laser_scan_lines_t& lines)
{
    clearHistogram();
    createLineHistogram(lines);

    int bestBin = findBestBin();
    
    if(bestBin >= 0)
    {
        verifyBestBin(histogram[bestBin]);

        return calculateAxes(histogram[bestBin]);
    }
    else
    {
        return {0.0f, 0.1f};
    }
}


void LPMAxesCalculator::clearHistogram(void)
{
    for(size_t n = 0; n < histogram.size(); ++n)
    {
        histogram[n].lines.clear();
        histogram[n].length = 0;
    }
}


void LPMAxesCalculator::createLineHistogram(const laser::laser_scan_lines_t& lines)
{
    double startAngle = findStartBin(lines);

    histogram_line_t histLine;

    for(auto lineIt = lines.lines.begin(), lineEnd = lines.lines.end(); lineIt != lineEnd; ++lineIt)
    {
        histLine.direction = line_direction(*lineIt, lines.pose);
        histLine.length    = length(*lineIt);

        std::size_t bin = static_cast<std::size_t>(wrap_to_2pi(histLine.direction - startAngle) / params.binWidth);

        assert(bin < histogram.size());

        histogram[bin].lines.push_back(histLine);
        histogram[bin].length += histLine.length;
    }
}


double LPMAxesCalculator::findStartBin(const laser::laser_scan_lines_t& lines) const
{
    // The starting bin is the bin associated with the longest line in the set of lines

    double maxDirection = 0.0;
    double maxLength    = 0.0;

    for(auto lineIt = lines.lines.begin(), lineEnd = lines.lines.end(); lineIt != lineEnd; ++lineIt)
    {
        if(length(*lineIt) > maxLength)
        {
            maxLength    = length(*lineIt);
            maxDirection = line_direction(*lineIt, lines.pose);
        }
    }

    // The maxDirection is the center of the first bin, so subtract off half the width to get the actual start angle
    return angle_diff(maxDirection, params.binWidth/2.0);
}


int LPMAxesCalculator::findBestBin(void) const
{
    // The best bin is the one with the longest cumulative length of lines contained inside
    int    maxBin    = -1;
    double maxLength = 0.0;

    for(size_t n = 0; n < histogram.size(); ++n)
    {
        if(histogram[n].length > maxLength)
        {
            maxBin    = n;
            maxLength = histogram[n].length;
        }
    }

    return maxBin;
}


void LPMAxesCalculator::verifyBestBin(const histogram_bin_t& bin) const
{
    // Don't worry about verification for now.
}


lpm_axes_t LPMAxesCalculator::calculateAxes(const histogram_bin_t& bin) const
{
    lpm_axes_t axes = {0.0, 0.0};
    
    double xSum = 0.0;
    double ySum = 0.0;

    // Use a simple weighted mean to find the orientation where longer lines have a higher weight
    // because they are probably more accurate
    for(auto lineIt = bin.lines.begin(), lineEnd = bin.lines.end(); lineIt != lineEnd; ++lineIt)
    {
        xSum += std::cos(lineIt->direction) * lineIt->length;
        ySum += std::sin(lineIt->direction) * lineIt->length;
    }

    axes.xAxisOrientation = std::atan2(ySum, xSum);

    // Use an unbiased weighted covariance
    double sigma      = 0.0;
    double sumWeights = 0.0;

    for(auto lineIt = bin.lines.begin(), lineEnd = bin.lines.end(); lineIt != lineEnd; ++lineIt)
    {
        double weight = lineIt->length / bin.length;

        sigma      += std::pow(angle_diff(axes.xAxisOrientation, lineIt->direction), 2) * weight;
        sumWeights += weight * weight;
    }

    if(sumWeights < 1.0)
    {
        axes.uncertainty = sigma / (1.0 - sumWeights);
    }
    else // There was only a single really long line in the bin
    {
        axes.uncertainty = 0.01;
    }

    // The orientation should be in the range (-pi/2, pi/2]. If outside that range, then just take the angle pi away.
    if(axes.xAxisOrientation > M_PI / 2.0)
    {
        axes.xAxisOrientation -= M_PI;
    }
    else if(axes.xAxisOrientation < -M_PI / 2.0)
    {
        axes.xAxisOrientation += M_PI;
    }

    // If the orientation is closer to the y-axis than the x-axis, rotate the orientation by pi/2 to get the x-axis orientation
    if(axes.xAxisOrientation > M_PI / 4.0)
    {
        axes.xAxisOrientation -= M_PI/2.0;
    }
    else if(axes.xAxisOrientation < -M_PI / 4.0)
    {
        axes.xAxisOrientation += M_PI/2.0;
    }

    return axes;
}

} // namespace hssh
} // namespace vulcan
