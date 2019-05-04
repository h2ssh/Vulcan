/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     histogram.cpp
* \author   Collin Johnson
*
* Definition of Histogram.
*/

#include <utils/histogram.h>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <cmath>

namespace vulcan
{
namespace utils
{

std::vector<HistogramBin> split_data_into_bins(const std::vector<double>& data,
                                               double minValue,
                                               double maxValue,
                                               int numBins);


/////////////// Operators /////////////////////
std::ostream& operator<<(std::ostream& out, const Histogram& hist)
{
    for(auto& bin : hist)
    {
        out << '[' << bin.minValue << ',' <<  bin.maxValue << "]: " << bin.count << '\n';
    }

    return out;
}

////////////// Histogram /////////////////////

Histogram::Histogram(int numBins)
: numBins_(numBins)
, bins_(numBins)
, binsAreDirty_(true)
, minValue_(NAN)
, maxValue_(NAN)
{
    assert(numBins_ > 0);
}


Histogram::Histogram(double minValue, double maxValue, int numBins)
: numBins_(numBins)
, bins_(numBins)
, binsAreDirty_(true)
, minValue_(minValue)
, maxValue_(maxValue)
{
    assert(minValue_ < maxValue_);
    assert(numBins_ > 0);
}


void Histogram::addValue(double value)
{
    values_.push_back(value);
    binsAreDirty_ = true;
}


int Histogram::findValueBinIndex(double value) const
{
    computeHistogramIfNeeded();

    auto binIt = std::find_if(bins_.begin(), bins_.end(), [value](const HistogramBin& bin) {
        return (bin.minValue <= value) && (value <= bin.maxValue);
    });

    return (binIt == bins_.end()) ? -1 : std::distance(bins_.begin(), binIt);
}


HistogramBin Histogram::bin(int binIndex) const
{
    computeHistogramIfNeeded();

    assert(binIndex >= 0);
    assert(binIndex < numBins_);

    return bins_[binIndex];
}


void Histogram::normalize(void)
{
    computeHistogramIfNeeded();

    double total = 0.0;

    for(auto& b : bins_)
    {
        total += b.count;
    }

    if(total > 0.0)
    {
        for(auto& b : bins_)
        {
            b.count /= total;
        }
    }
}


void Histogram::clear(void)
{
    values_.clear();
    binsAreDirty_ = true;
}


Histogram::const_iterator Histogram::begin(void) const
{
    computeHistogramIfNeeded();
    return bins_.begin();
}


Histogram::const_iterator Histogram::end(void) const
{
    assert(!binsAreDirty_);
    return bins_.end();
}


math::UnivariateGaussianDistribution Histogram::toGaussian(void) const
{
    return math::UnivariateGaussianDistribution(values_.begin(), values_.end());
}


void Histogram::computeHistogramIfNeeded(void) const
{
    if(binsAreDirty_ && !values_.empty())
    {
        if(std::isnan(minValue_) || std::isnan(maxValue_))
        {
            minValue_ = *std::min_element(values_.begin(), values_.end());
            maxValue_ = *std::max_element(values_.begin(), values_.end());
        }

        bins_ = split_data_into_bins(values_, minValue_, maxValue_, numBins_);
        binsAreDirty_ = false;
    }
}


std::vector<HistogramBin> split_data_into_bins(const std::vector<double>& data,
                                               double minValue,
                                               double maxValue,
                                               int numBins)
{
    if(data.empty() || (numBins == 0))
    {
        return std::vector<HistogramBin>();
    }

    double range = maxValue - minValue;
    double binWidth = range / numBins;

    // All bins can't be the same
    if(binWidth == 0.0)
    {
        return std::vector<HistogramBin>();
    }

    // Construct the bins
    std::vector<HistogramBin> bins(numBins);
    bins[0].minValue = minValue;
    bins[0].maxValue = minValue + binWidth;
    bins[0].count = 0;

    for(int n = 1; n < numBins; ++n)
    {
        bins[n].minValue = bins[n - 1].maxValue;
        bins[n].maxValue = bins[n].minValue + binWidth;
        bins[n].count = 0;
    }

    // Populate the bins
    for(auto value : data)
    {
        int binIdx = (value - minValue) / binWidth;
        binIdx = boost::algorithm::clamp(binIdx, 0, numBins - 1);
        bins[binIdx].count++;
    }

    return bins;
}

} // namespace utils
} // namespace vulcan
