/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_HISTOGRAM_2D_H
#define UTILS_HISTOGRAM_2D_H

#include "utils/cell_grid.h"
#include <vector>

namespace vulcan
{
namespace utils
{

/**
*
*/
class Histogram2D
{
public:

    /**
    * Create a histogram with a fixed range of values.
    *
    * \pre  minValue < maxValue
    * \pre  numBins > 0
    * \param    minValue        Minimum value that can go in the histogram
    * \param    maxValue        Maximum value for the histogram
    * \param    numBins         Numbers of bins for the histogram
    */
    Histogram2D(double minX, double maxX, double minY, double maxY, int numBins);

    /**
    * addValue adds another value to the stored histogram data.
    */
    void addValue(double x, double y);

    /**
    * Normalize the count of bins to the range [0, 1].
    *
    * NOTE: Normalization is destructive and can only occur once. Add all values and then normalize right now.
    *
    * Attempts to normalize multiple times will fail with a warning.
    *
    * \return   True if normalization was performed. False otherwise.
    */
    bool normalize(void);

    /**
    * Find the maximum value stored in a histogram bin.
    */
    double max(void) const;

    /**
    * clear clears all data from the histogram.
    */
    void clear(void);

    /**
    * Retrieve the number of values added to the histogram.
    */
    std::size_t numValues(void) const { return values_.size(); }

    /**
     * Draw the histogram using Gnuplot.
     */
    void plot(const std::string& title,
              const std::string& xLabel,
              const std::string& yLabel,
              double maxCbrange);

private:

    utils::CellGrid<double> bins_;
    std::vector<Point<double>> values_;
    bool isNormalized_ = false;
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_HISTOGRAM_2D_H
