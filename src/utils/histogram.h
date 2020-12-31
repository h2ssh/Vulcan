/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     histogram.h
 * \author   Collin Johnson
 *
 * Declaration of Histogram class and supporting POD HistogramBin.
 */

#ifndef UTILS_HISTOGRAM_H
#define UTILS_HISTOGRAM_H

#include "math/univariate_gaussian.h"
#include <algorithm>
#include <iosfwd>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
 * HistogramBin represents the values stored in one bin in the histogram. The count and range represented by the
 * histogram are provided.
 */
struct HistogramBin
{
    double minValue;
    double maxValue;
    double count;
};

/**
 * Histogram stores a
 */
class Histogram
{
public:
    using const_iterator = std::vector<HistogramBin>::const_iterator;
    using size_type = std::vector<Histogram>::size_type;

    /**
     * Constructor for Histogram.
     *
     * \param    numBins         The number of bins in which to split the data (optional, default = 100)
     * \pre  numBins > 0
     */
    explicit Histogram(int numBins = 100);

    /**
     * Create a histogram with a fixed range of values.
     *
     * \pre  minValue < maxValue
     * \pre  numBins > 0
     * \param    minValue        Minimum value that can go in the histogram
     * \param    maxValue        Maximum value for the histogram
     * \param    numBins         Numbers of bins for the histogram
     */
    Histogram(double minValue, double maxValue, int numBins);

    /**
     * addValue adds another value to the stored histogram data.
     */
    void addValue(double value);

    /**
     * addRange adds a range of values to the stored histogram data.
     */
    template <class Iterator>
    void addRange(Iterator begin, Iterator end)
    {
        std::for_each(begin, end, [this](double value) {
            addValue(value);
        });
    }

    /**
     * findValueBinIndex finds the bin index associated with a given value. The bin index is the index of the histogram
     * bin into which a particular value falls. If the value isn't in a bin, the index will be -1.
     *
     * \param    value           Value for which to search for the bin
     * \return   Index of the bin if value is in the histogram. -1 if the value isn't in the histogram's range.
     */
    int findValueBinIndex(double value) const;

    /**
     * bin retrieves the bin with the associated index.
     *
     * \param    binIndex        Index of the bin to retrieve
     * \pre binIndex >= 0 && binIndex < size
     * \return   Bin associated with the provided index.
     */
    HistogramBin bin(int binIndex) const;

    /**
     * Normalize the count of bins to the range [0, 1].
     */
    void normalize(void);

    /**
     * clear clears all data from the histogram.
     */
    void clear(void);

    // Iterator support for iterating through the bins
    const_iterator begin(void) const;
    const_iterator end(void) const;

    /**
     * size retrieves the number bins in the histogram.
     */
    size_type size(void) const { return numBins_; }

    /**
     * Retrieve the number of values added to the histogram.
     */
    size_type numValues(void) const { return values_.size(); }

    /**
     * Estimate a Gaussian distribution from the example data.
     */
    math::UnivariateGaussianDistribution toGaussian(void) const;

private:
    int numBins_;
    std::vector<double> values_;
    mutable std::vector<HistogramBin> bins_;
    mutable bool binsAreDirty_;
    mutable double minValue_;
    mutable double maxValue_;

    void computeHistogramIfNeeded(void) const;
};

// Operators
/**
 * Output operator for Histogram. The output format is such that is can be easily read to produce a histogram in
 * GNUplot.
 *
 * The format is: (ne bin per line)
 *
 *   (minValue+(minValue + maxValue)/2) count (maxValue-minValue)
 *
 * This format provides the center of the bin, the height in raw count, and the width in histogram units, which is the
 * data needed by the boxes style in GNUplot.
 *
 * \param    out         Stream to write to
 * \param    hist        Histogram to write
 * \return   Out after histogram is written
 */
std::ostream& operator<<(std::ostream& out, const Histogram& hist);

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_HISTOGRAM_H
