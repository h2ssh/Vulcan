/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <utils/histogram_2d.h>
#include <gnuplot-iostream.h>

namespace vulcan
{
namespace utils
{

Histogram2D::Histogram2D(double minX, double maxX, double minY, double maxY, int numBins)
: bins_(Point<float>(minX, minY), maxX - minX, maxY - minY, numBins, 0.0)
{
}


void Histogram2D::addValue(double x, double y)
{
    values_.emplace_back(x, y);
    auto bin = utils::global_point_to_grid_cell(Point<double>(x, y), bins_);
    if(bins_.isCellInGrid(bin))
    {
        ++bins_(bin.x, bin.y);
    }
}


bool Histogram2D::normalize(void)
{
    // Ignore if there are no values yet
    if(values_.empty() || isNormalized_)
    {
        std::cerr << "WARNING: Histogram2D: Ignore request for normalization. Already normalized? " << isNormalized_
            << '\n';
        return false;
    }

    // Divide all bins by the total number of values to get the percent of time a particular value is
    // occupied
    for(std::size_t y = 0; y < bins_.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < bins_.getWidthInCells(); ++x)
        {
            bins_(x, y) /= values_.size();
        }
    }

    isNormalized_ = true;
    return true;
}


double Histogram2D::max(void) const
{
    double max = 0.0;

    for(std::size_t y = 0; y < bins_.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < bins_.getWidthInCells(); ++x)
        {
            max = std::max(bins_(x, y), max);
        }
    }

    return max;
}


void Histogram2D::clear(void)
{
    bins_.reset(0.0);
    values_.clear();
}


void Histogram2D::plot(const std::string& title,
                       const std::string& xLabel,
                       const std::string& yLabel,
                       double maxCbrange)
{
//     std::vector<std::vector<boost::tuple<double, double, double>>> data2d(bins_.getHeightInCells());
    std::vector<std::vector<double>> data2d(bins_.getHeightInCells());
    // Convert the bins to appropriate 2d data
    for(std::size_t y = 0; y < bins_.getHeightInCells(); ++y)
    {
        data2d[y].resize(bins_.getWidthInCells());
        auto& row = data2d[y];
        for(std::size_t x = 0; x < bins_.getWidthInCells(); ++x)
        {
            row[x] = bins_(x, y);
//             row[x] = boost::make_tuple(double(x), double(y), bins_(x, y));
        }
    }

    double scale = bins_.metersPerCell();

    double yMin = bins_.getBottomLeft().y - (scale / 2.0);
    double yMax = bins_.getBottomLeft().y + bins_.getHeightInMeters() - (scale / 2.0);
    double xMin = bins_.getBottomLeft().x - (scale / 2.0);
    double xMax = bins_.getBottomLeft().x + bins_.getWidthInMeters() - (scale / 2.0);

    Gnuplot plot;
    plot << "set title '" << title << "'\n";
    plot << "set x2label '" << xLabel << "'\n";
    plot << "set ylabel '" << yLabel << "'\n";
    plot << "set yrange [" << yMax << ":" << yMin << "]\n"; // intentionally flip y-axis
    plot << "set xrange [" << xMin << ":" << xMax << "]\n";
    plot << "set x2range [" << xMin << ":" << xMax << "]\n";
    plot << "set cbrange [0:" << maxCbrange << "]\n";
    plot << "unset xtics\n";
    plot << "set x2tics\n";

    plot << "plot '-' binary "
        << " dx=" << bins_.metersPerCell() << " dy=" << bins_.metersPerCell()
        << plot.binFmt2d(data2d, "array") << " with image notitle"
        << std::endl;
    plot.sendBinary2d(data2d);

    sleep(1);
}

} // namespace utils
} // namepace vulcan
