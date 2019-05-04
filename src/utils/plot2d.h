/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     plot2d.h
* \author   Collin Johnson
*
* Declaration of Plot2D.
*/

#ifndef UTILS_PLOT2D_H
#define UTILS_PLOT2D_H

#include <boost/optional.hpp>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
* PlotStyle sets the style to use for drawing the data.
*/
enum class PlotStyle
{
    points,
    lines,
};

/**
* Plot2D will store and display information for a simple 2D plot. It is a simple abstraction on top of gnuplot-iostream.
*/
class Plot2D
{
public:

    /**
    * Constructor for Plot2D.
    *
    * \param    title           Title of the plot
    * \param    xLabel          Label for x-axis
    * \param    yLabel          Label for y-axis
    */
    Plot2D(const std::string& title, const std::string& xLabel, const std::string& yLabel);

    /**
    * addData adds a new piece of data to the plot.
    *
    * \param    x       x-value
    * \param    y       y-value
    * \param    type    Optional type information in case multiple types of data will appear in the plot
    */
    void addData(double x, double y, int type = 0);

    /**
    * addData adds a big chunk of data to the plot.
    *
    * \param    x       x-values
    * \param    y       y-values
    * \param    type    Optional type information in case multiple types of data will appear in the plot
    *
    * \pre x.size() == y.size()
    */
    void addData(const std::vector<double>& x, const std::vector<double>& y, int type = 0);

    /**
    * setTypeName sets the name to use for a given type that will be plotted.
    */
    void setTypeName(int type, const std::string& name);

    /**
    * setXRange sets the range to display for the x-axis.
    *
    * \pre  min < max
    */
    void setXRange(double min, double max);

    /**
    * setYRange sets the range to display for the y-axis.
    *
    * \pre  min < max
    */
    void setYRange(double min, double max);

    /**
    * plot will draw the plot.
    *
    * \param    style           Style of plot to be drawn
    */
    void plot(PlotStyle style);

private:

    using Data = std::vector<std::tuple<double, double>>;
    using Range = std::pair<double, double>;

    std::map<int, Data> data_;
    std::map<int, std::string> names_;
    std::string title_;
    std::string xLabel_;
    std::string yLabel_;
    boost::optional<Range> xRange_;
    boost::optional<Range> yRange_;
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_PLOT2D_H
