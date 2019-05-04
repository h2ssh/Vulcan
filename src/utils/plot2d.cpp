/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     plot2d.cpp
* \author   Collin Johnson
*
* Definition of Plot2D.
*/

#include <utils/plot2d.h>
#include <gnuplot-iostream.h>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>

namespace vulcan
{
namespace utils
{

std::string style_string(PlotStyle style);


Plot2D::Plot2D(const std::string& title, const std::string& xLabel, const std::string& yLabel)
: title_(title)
, xLabel_(xLabel)
, yLabel_(yLabel)
{
}

void Plot2D::addData(double x, double y, int type)
{
    data_[type].emplace_back(x, y);
}


void Plot2D::addData(const std::vector<double>& x, const std::vector<double>& y, int type)
{
    assert(x.size() == y.size());
    for(std::size_t n = 0; n < x.size(); ++n)
    {
        addData(x[n], y[n], type);
    }
}


void Plot2D::setTypeName(int type, const std::string& name)
{
    names_[type] = name;
}


void Plot2D::setXRange(double min, double max)
{
    xRange_ = std::make_pair(min, max);
}


void Plot2D::setYRange(double min, double max)
{
    yRange_ = std::make_pair(min, max);
}


void Plot2D::plot(PlotStyle style)
{
    Gnuplot plot;

    plot << "set title '" << title_ << "'\n"
        << "set xlabel '" << xLabel_ << "'\n"
        << "set ylabel '" << yLabel_ << "'\n";

    if(xRange_)
    {
        plot << "set xrange [" << xRange_->first << ':' << xRange_->second << "]\n";
    }

    if(yRange_)
    {
        plot << "set yrange [" << yRange_->first << ':' << yRange_->second << "]\n";
    }

    std::string styleStr = style_string(style);

    // Setup the plot command
    int numAdded = 0;
    int total = data_.size();
    plot << "plot ";
    for(auto& d : data_)
    {
        plot << "'-' with " << styleStr;

        if(names_.find(d.first) != names_.end())
        {
            plot << " title '" << names_[d.first] << "'";
        }

        ++numAdded;
        if(numAdded != total)
        {
            plot << ',';
        }
        else
        {
            plot << '\n';
        }
    }

    // Pass all the data in -- plot created with same iteration over hash map, so same ordering will be guaranteed
    for(auto& d : data_)
    {
        plot.send1d(d.second);
    }
}


std::string style_string(PlotStyle style)
{
    switch(style)
    {
    case PlotStyle::points:
        return "points";

    case PlotStyle::lines:
        return "lines";

    default:
        return "points";
    }
}

} // namespace math
} // namespace vulcan
