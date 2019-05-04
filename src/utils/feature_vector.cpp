/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_features.cpp
 * \author   Collin Johnson
 * 
 * Definition of FeatureVector.
 */

#include <utils/feature_vector.h>
#include <core/float_comparison.h>
#include <utils/float_io.h>
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <cmath>
#include <functional>

namespace vulcan
{
namespace utils
{

////////////////////  Operators ///////////////////////////

std::ostream& operator<<(std::ostream& out, const FeatureVector& features)
{
    out << features.version() << ' ' << features.numFeatures() << ' ';
    for(auto f : features)
    {
        utils::save_floating_point(out, f);
    }
    
    return out;
}


std::istream& operator>>(std::istream& in, FeatureVector& features)
{
    in >> features.version_;
    
    int numFeatures = 0;
    in >> numFeatures;
    
    features.features_.resize(numFeatures);
    for(int n = 0; n < numFeatures; ++n)
    {
        features.features_[n] = utils::load_floating_point(in);
    }
    
    return in;
}


bool operator==(const FeatureVector& lhs, const FeatureVector& rhs)
{
    if(lhs.numFeatures() != rhs.numFeatures())
    {
        return false;
    }
    
    for(std::size_t n = 0; n < lhs.numFeatures(); ++n)
    {
        if(!absolute_fuzzy_equal(lhs.featureAt(n), rhs.featureAt(n)))
        {
            return false;
        }
    }
    
    return true;
}


bool operator!=(const FeatureVector& lhs, const FeatureVector& rhs)
{
    return !(lhs == rhs);
}


//////////////////// FeatureVector /////////////////////////////

FeatureVector::FeatureVector(const Features& features, int version)
: version_(version)
, features_(features)
{
}


FeatureVector::FeatureVector(std::size_t numFeatures, int version)
: version_(version)
, features_(numFeatures)
{
}


FeatureVector FeatureVector::sample(const Vector& stddev) const
{
    // Add zero-mean noise for the sampling
    return FeatureVector(features_ + (arma::randn(numFeatures()) % stddev), version());
}

} // namespace utils
} // namespace vulcan
