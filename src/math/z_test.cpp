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
 * Definition of binomial_proportion_z_test.
 */

#include "math/z_test.h"
#include <boost/math/distributions/normal.hpp>

namespace vulcan
{
namespace math
{

z_test_results_t
  binomial_proportion_z_test(const z_test_sample_t& sampleA, const z_test_sample_t& sampleB, double confidence)
{
    // Code closely related to Boost.Math example at:
    // http://www.boost.org/doc/libs/1_65_1/libs/math/doc/html/math_toolkit/stat_tut/weg/st_eg/two_sample_students_t.html

    using namespace boost::math;

    assert(sampleA.numSamples > 1);
    assert(sampleB.numSamples > 1);
    assert(sampleA.numTrue >= 0);
    assert(sampleB.numTrue >= 0);

    double successA = static_cast<double>(sampleA.numTrue) / sampleA.numSamples;
    double successB = static_cast<double>(sampleB.numTrue) / sampleB.numSamples;

    double totalSuccess =
      static_cast<double>(sampleA.numTrue + sampleB.numTrue) / (sampleA.numSamples + sampleB.numSamples);

    double zDenom =
      std::sqrt(totalSuccess * (1.0 - totalSuccess) * ((1.0 / sampleA.numSamples) + (1.0 / sampleB.numSamples)));

    z_test_results_t results;
    results.zValue = (successA - successB) / zDenom;
    results.confidence = confidence;

    normal dist;

    results.pValueDifferent = cdf(complement(dist, std::abs(results.zValue)));
    results.areDifferent = results.pValueDifferent < (confidence / 2.0);

    results.pValueGreater = cdf(complement(dist, results.zValue));
    results.isGreater = results.pValueGreater < confidence;

    results.pValueLess = cdf(dist, results.zValue);
    results.isLess = results.pValueLess < confidence;

    return results;
}

}   // namespace math
}   // namespace vulcan
