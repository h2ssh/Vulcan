/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     kld_sample_size_calculator.h
* \author   Collin Johnson
*
* Declaration of KLDSampleSizeCalculator.
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_KLD_SAMPLE_SIZE_CALCULATOR_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_KLD_SAMPLE_SIZE_CALCULATOR_H

#include "hssh/metrical/localization/params.h"
#include "hssh/metrical/localization/particle.h"

namespace vulcan
{
namespace hssh
{

/**
* KLDSampleSizeCalculator is used to determine the necessary sample size of a particle filter
* using KLD-sampling.

    void addSampleToBins(const particle_t arg1);*
* kld_sampling_params_t controls the number of bins, the size of the bins, and the two parameters,
* delta and epsilon, responsible for determining the sample size needed.
*
* To use the KLDSampleSizeCalculator, call startNewCalculation() at the beginning of a particle
* filter update. The provided sample should be near the mean of the prior and hopefully posterior,
* as the bins for the sampler will be centered around this sample. Next, for each generated sample
* in the proposal distribution, call addSample() which will then calculate the new sample size.
* isFinishedSampling() can then be used to check if enough samples have been generated.
*/
class KLDSampleSizeCalculator
{
public:

    /**
    * Constructor for KLDSampleSizeCalculator.
    */
    KLDSampleSizeCalculator(const kld_sampling_params_t& params);

    /**
    * Destructor for KLDSampleSizeCalculator.
    */
    ~KLDSampleSizeCalculator(void);

    /**
    * startNewCalculation resets the state of the calculator for a new
    * sample size calculation.
    *
    * The bins are centered around the provided sample. The sample should be the
    * mean of the prior or some value located close to the middle of the expected
    * posterior distribution in order to make sure the samples don't fall outside
    * the volume of bins that are maintained.
    *
    * \param    meanSample          Sample near the mean of the prior
    */
    void startNewCalculation(const particle_t& meanSample);

    /**
    * addSample adds a new sample to the sampling bins. The sample size is adjusted
    * accordingly.
    *
    * \param    sample              New sample in the proposal distribution
    */
    void addSample(const particle_t& sample);

    /**
    * currentNumberOfSamplesNeeded retrieves the current value for the number of samples
    * needed, given the previous samples drawn.
    */
    std::size_t currentNumberOfSamplesNeeded(void) const;

    /**
    * haveDrawnEnoughSamples checks to see if enough samples have been generated to
    * satisfactorially cover the posterior distribution.
    *
    * \param    numSamples          Number of samples generated so far
    * \return   True if sampling can be stopped. False if sampling needs to continue.
    */
    bool haveDrawnEnoughSamples(std::size_t numSamples) const;

private:

    // No copying allowed!
    KLDSampleSizeCalculator(const KLDSampleSizeCalculator& copy) = delete;
    void operator=(const KLDSampleSizeCalculator& rhs) = delete;

    bool addSampleToBins(const particle_t& sample);
    
    int binIndex(int xBin, int yBin, int thetaBin)
    {
        return xBin + yBin*params.numXBins + thetaBin*params.numXBins*params.numYBins;
    }

    // the bins in the current calculation have calculationID. this is used
    // to avoid the need to reset the sampleBins at the start of each calculation,
    // which would be necessary if a bool flag was used
    char* sampleBins;
    char  calculationID;
    int   numFilledBins;

    particle_t centerSample;

    std::size_t samplesNeeded;

    kld_sampling_params_t params;
};

}
}

#endif // HSSH_UTILS_METRICAL_LOCALIZATION_KLD_SAMPLE_SIZE_CALCULATOR_H
