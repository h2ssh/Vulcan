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
* Definition of independent_t_test.
*/

#include "math/t_test.h"
#include <boost/math/distributions/students_t.hpp>

namespace vulcan
{
namespace math
{

t_test_results_t independent_t_test(const t_test_sample_t& sampleA, const t_test_sample_t& sampleB, double confidence)
{
    // Code closely related to Boost.Math example at:
    // http://www.boost.org/doc/libs/1_65_1/libs/math/doc/html/math_toolkit/stat_tut/weg/st_eg/two_sample_students_t.html

    using namespace boost::math;

    assert(sampleA.numSamples > 1);
    assert(sampleB.numSamples > 1);
    assert(sampleA.variance > 0.0);
    assert(sampleB.variance > 0.0);

    int dofA = sampleA.numSamples - 1;
    int dofB = sampleB.numSamples - 1;
    int dof  = dofA + dofB;

    double pooledVariance = std::sqrt(((dofA * sampleA.variance) + (dofB * sampleB.variance)) / dof);
    double tDenom         = pooledVariance * std::sqrt((1.0 / sampleA.numSamples) + (1.0 / sampleB.numSamples));

    t_test_results_t results;
    results.tValue     = (sampleA.mean - sampleB.mean) / tDenom;
    results.confidence = confidence;

    students_t dist(dof);

    results.pValueDifferent = cdf(complement(dist, std::abs(results.tValue)));
    results.areDifferent    = results.pValueDifferent < (confidence / 2.0);

    results.pValueGreater = cdf(complement(dist, results.tValue));
    results.isGreater     = results.pValueGreater < confidence;

    results.pValueLess = cdf(dist, results.tValue);
    results.isLess     = results.pValueLess < confidence;

    return results;
}

} // namespace math
} // namespace vulcan
