/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     kld_sample_size_calculator.cpp
* \author   Collin Johnson
*
* Definition of KLDSampleSizeCalculator.
*/


#include <hssh/metrical/localization/kld_sample_size_calculator.h>
#include <core/angle_functions.h>
#include <boost/algorithm/clamp.hpp>
#include <iostream>

// #define DEBUG_KLD

namespace vulcan
{
namespace hssh
{

int calculate_samples_needed(int numFilledBins, float z, float epsilon);

inline bool valid_bin(int binIndex, int numBins) { return (binIndex >= 0) && (binIndex < numBins); }


KLDSampleSizeCalculator::KLDSampleSizeCalculator(const kld_sampling_params_t& params)
: sampleBins(0)
, calculationID(0)
, params(params)
{
    sampleBins = new char[params.numXBins * params.numYBins * params.numThetaBins];
}


KLDSampleSizeCalculator::~KLDSampleSizeCalculator(void)
{
    delete [] sampleBins;
}


void KLDSampleSizeCalculator::startNewCalculation(const particle_t& meanSample)
{
    centerSample  = meanSample;
    numFilledBins = 0;
    samplesNeeded = 0;
    ++calculationID;
}


void KLDSampleSizeCalculator::addSample(const particle_t& sample)
{
    if(addSampleToBins(sample))
    {
        ++numFilledBins;
        samplesNeeded = calculate_samples_needed(numFilledBins, params.z, params.epsilon);

#ifdef DEBUG_KLD
        std::cout<<"INFO: KLDSampleSizeCalculator: k="<<numFilledBins<<" samples="<<samplesNeeded<<'\n';
#endif
    }
}


std::size_t KLDSampleSizeCalculator::currentNumberOfSamplesNeeded(void) const
{
    return (samplesNeeded < params.minSamples) ? params.minSamples : samplesNeeded;
}


bool KLDSampleSizeCalculator::haveDrawnEnoughSamples(std::size_t numSamples) const
{
    return (numSamples >= samplesNeeded) && (numSamples > params.minSamples);
}


bool KLDSampleSizeCalculator::addSampleToBins(const particle_t& sample)
{
    int binX     = (sample.pose.x - centerSample.pose.x) / params.xBinWidth + params.numXBins/2;
    int binY     = (sample.pose.y - centerSample.pose.y) / params.yBinWidth + params.numYBins/2;
    int binTheta = (angle_diff(sample.pose.theta, centerSample.pose.theta) / params.thetaBinWidth) + params.numThetaBins/2;

    bool binWasEmpty = false;
    
    binX     = boost::algorithm::clamp(binX,     0, params.numXBins-1);
    binY     = boost::algorithm::clamp(binY,     0, params.numYBins-1);
    binTheta = boost::algorithm::clamp(binTheta, 0, params.numThetaBins-1);

    int index         = binIndex(binX, binY, binTheta);
    binWasEmpty       = sampleBins[index] != calculationID;
    sampleBins[index] = calculationID;

    return binWasEmpty;
}


int calculate_samples_needed(int numFilledBins, float z, float epsilon)
{
    /*
    * KL-distance = (k-1)/(2*epsilon) * {1 - 2/(9(k-1)) + z*sqrt(2/(9(k-1)))}^3
    *
    * k == numFilledBins
    */

    int samplesNeeded = 0;

    if(numFilledBins > 1)
    {
        float coefficient = (numFilledBins-1) / (2*epsilon);
        float insidePow   = 1.0f - 2/(9*(numFilledBins-1)) + z*sqrt(2/(9*(numFilledBins-1)));

        samplesNeeded = static_cast<std::size_t>(coefficient * pow(insidePow, 3));
    }

    return samplesNeeded;
}

} // namespace hssh
} // namespace vulcan
