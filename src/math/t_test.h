/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file
* \author   Collin Johnson
*
* Declaration of t_test_results_t and independent_t_test.
*/

#ifndef MATH_T_TEST_H
#define MATH_T_TEST_H

namespace vulcan
{
namespace math
{

/**
* Results for running an t-test on two sets of samples.
*
* Based on a provided confidence threshold, the summary provided details:
*
*   - Are means significantly different? (Two-tailed test)
*   - Is sample A mean significantly less than sample B mean? (One-tailed test)
*   - Is sample A mean significantly greater than sample B mean? (One-tailed test)
*/
struct t_test_results_t
{
    double tValue;          ///< T-statistic computed for the samples
    double confidence;      ///< Confidence value provided by caller

    // P-Values giving probability of difference between the distributions
    double pValueDifferent;
    double pValueLess;
    double pValueGreater;

    // Conditions based on the p-values
    bool areDifferent;      ///< Don't reject hypothesis they are different
    bool isLess;            ///< Don't reject hypothesis that sample A mean is less than sample B mean
    bool isGreater;         ///< Don't reject hypothesis that Sample A mean is greater than sample B mean
};

/**
* Description of samples for a t-test.
*/
struct t_test_sample_t
{
    double mean;
    double variance;
    int numSamples;
};


/**
* Run a t-test on independent samples.
*
* \pre  sampleA.numSamples > 1 && sampleB.numSamples > 1
* \pre  sampleA.variance > 0
* \pre  sampleB.variance > 0
* \param    sampleA         One of the samples
* \param    sampleB         The other sample
* \param    confidence      Confidence value for asserting a condition to be true or false
* \return   Results of the t-test as a t_test_results_t that summarizes the analysis.
*/
t_test_results_t independent_t_test(const t_test_sample_t& sampleA, const t_test_sample_t& sampleB, double confidence);

} // namespace math
} // namespace vulcan

#endif // MATH_T_TEST_H
