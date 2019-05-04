/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     uncertain_value.h
* \author   Collin Johnson
*
* Definition of UncertainValue template.
*/

#ifndef MATH_UNCERTAIN_VALUE_H
#define MATH_UNCERTAIN_VALUE_H

#include <iomanip>
#include <iostream>
#include <limits>
#include <type_traits>
#include <cereal/access.hpp>

namespace vulcan
{
namespace math
{

/**
* UncertainValue
*/
template <typename T = double>
class UncertainValue
{
public:

    UncertainValue(void)
    : min_(std::numeric_limits<T>::max())
    , max_(std::numeric_limits<T>::lowest())
    , mean_(0)
    , variance_(0)
    , m2_(0)
    , numSamples_(0)
    {
        static_assert(std::is_arithmetic<T>::value, "UncertainValue parameter must be arithmetic");
    }

    T      min(void) const { return min_; }
    T      max(void) const { return max_; }
    double mean(void) const { return mean_; }
    double variance(void) const { return variance_; }
    int numSamples(void) const { return numSamples_; }

    void addSample(T sample)
    {
        ++numSamples_;

        min_ = std::min(sample, min_);
        max_ = std::max(sample, max_);

        // Calculate online mean and variance
        auto delta = sample - mean_;
        mean_  = mean_ + (delta / numSamples_);
        m2_   += (delta * (sample - mean_));

        if(numSamples_ > 1)
        {
            variance_ = m2_ / (numSamples_ - 1);
        }
    }

private:

    T      min_;
    T      max_;
    double mean_;
    double variance_;
    T      m2_;            ///< Moment used for inline calculation of variance
    int    numSamples_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( min_,
            max_,
            mean_,
            variance_,
            m2_,
            numSamples_);
    }
};

// Output operator for UncertainValue
template <typename T>
std::ostream& operator<<(std::ostream& out, const UncertainValue<T>& value)
{
    out << std::setprecision(3) << "Min:" << std::setw(6) << value.min() << " Max:" << value.max() << " Mean:"
        << value.mean() << " Var:" << value.variance() << " Num samples:" << value.numSamples();
    return out;
}

}
}

#endif // MATH_UNCERTAIN_VALUE_H
