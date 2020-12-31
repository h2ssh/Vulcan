/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     feature_vector.h
 * \author   Collin Johnson
 *
 * Declaration of FeatureVector.
 */

#ifndef UTILS_FEATURE_VECTOR_H
#define UTILS_FEATURE_VECTOR_H

#include "core/vector.h"

namespace vulcan
{
namespace utils
{

/**
 * FeatureVector represents a simple set of features with versioning information and I/O support.
 */
class FeatureVector
{
public:
    using Features = Vector;
    using const_iterator = Features::const_iterator;
    using iterator = Features::iterator;

    /**
     * Constructor for FeatureVector.
     *
     * \param    features        Features associated with the gateway
     * \param    version         Version number for the features
     */
    FeatureVector(const Features& features, int version);

    /**
     * Constructor for FeatureVector.
     *
     * This constructor is intended for cases where the features will be incrementally constructed for efficiency.
     * Obviously, care must be taken when building the features.
     *
     * \param    numFeatures     Number of features to allocate space for
     * \param    version         Version number of the features
     */
    FeatureVector(std::size_t numFeatures, int version);

    // Can default all constructors
    FeatureVector(void) = default;
    FeatureVector& operator=(FeatureVector&& rhs) = default;
    FeatureVector& operator=(const FeatureVector& rhs) = default;
    FeatureVector(FeatureVector&& rhs) = default;
    FeatureVector(const FeatureVector& rhs) = default;

    /**
     * sample creates a new FeatureVector drawn from a distribution with the same mean and stddev as provided.
     *
     * \param    stddev      Standard deviation of the distribution sampled from
     * \return   A new FeatureVector sampled from this feature vector.
     *
     * \pre  stddev.n_rows == numFeatures()
     */
    FeatureVector sample(const Vector& stddev) const;

    /**
     * numFeatures retrieves the number of features currently being used.
     */
    std::size_t numFeatures(void) const { return features_.size(); }

    /**
     * version retrieves the version number for the features. Every time these features are changed, the version
     * increases by 1. Thus, the version number is closely associated with the number of different attempts at made at
     * finding a good feature set.
     */
    int version(void) const { return version_; }

    double featureAt(std::size_t index) const { return features_[index]; }
    const Features& features(void) const { return features_; }

    const_iterator begin(void) const { return features_.begin(); }
    const_iterator end(void) const { return features_.end(); }
    double operator[](int index) const { return features_[index]; }

    iterator begin(void) { return features_.begin(); }
    iterator end(void) { return features_.end(); }
    double& operator[](int index) { return features_[index]; }

    // I/O operators for FeatureVector
    /**
     * The format for FeatureVector output is:
     *
     *   num_features feature_0 ... feature_n-1 num_isovists num_isovist_features iso_feats_00 iso_feats_01 ...
     * exploration
     */
    friend std::ostream& operator<<(std::ostream& out, const FeatureVector& features);
    friend std::istream& operator>>(std::istream& in, FeatureVector& features);

private:
    int version_ = -1;
    Features features_;
};

// Equality operators for FeatureVector
bool operator==(const FeatureVector& lhs, const FeatureVector& rhs);
bool operator!=(const FeatureVector& lhs, const FeatureVector& rhs);

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_FEATURE_VECTOR_H
