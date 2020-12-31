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
 * Declaration of z_test_results_t and binomial_proportion_z_test.
 */

#ifndef MATH_Z_TEST_H
#define MATH_Z_TEST_H

namespace vulcan
{
namespace math
{

/**
 * Results for running an z-test on two sets of samples from a binomial distribution.
 *
 * Based on a provided confidence threshold, the summary provided details:
 *
 *   - Are means significantly different? (Two-tailed test)
 *   - Is sample A mean significantly less than sample B mean? (One-tailed test)
 *   - Is sample A mean significantly greater than sample B mean? (One-tailed test)
 */
struct z_test_results_t
{
    double zValue;       ///< Z-statistic computed for the samples
    double confidence;   ///< Confidence value provided by caller

    // P-Values giving probability of difference between the distributions
    double pValueDifferent;
    double pValueLess;
    double pValueGreater;

    // Conditions based on the p-values
    bool areDifferent;   ///< Don't reject hypothesis they are different
    bool isLess;         ///< Don't reject hypothesis that sample A mean is less than sample B mean
    bool isGreater;      ///< Don't reject hypothesis that Sample A mean is greater than sample B mean
};

/**
 * Description of samples for a z-test.
 */
struct z_test_sample_t
{
    int numTrue;   // Number of true samples (must be <= numSamples)
    int numSamples;
};

/**
 * Run a binomial proportion Z-test on two samples.
 *
 * See: http://www.itl.nist.gov/div898/software/dataplot/refman1/auxillar/binotest.htm
 *
 * \pre  sampleA.numSamples > 0
 * \pre  sampleB.numSamples > 0
 * \pre  sampleA.numTrue >= 0 && sampleA.numTrue <= numSamples
 * \pre  sampleB.numTrue >= 0 && sampleB.numTrue <= numSamples
 * \param    sampleA             Collection of samples
 * \param    sampleB             Collection of samples to compare against
 * \param    confidence          Confidence level for the prediction
 * \return   A z_test_results_t with the results of all possible hypotheses.
 */
z_test_results_t
  binomial_proportion_z_test(const z_test_sample_t& sampleA, const z_test_sample_t& sampleB, double confidence);

}   // namespace math
}   // namespace vulcan

#endif   // MATH_Z_TEST_H
