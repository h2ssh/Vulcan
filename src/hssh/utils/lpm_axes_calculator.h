/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_axes_calculator.h
* \author   Collin Johnson
*
* Declaration of lpm_axes_t and LPMAxesCalculator.
*/

#ifndef HSSH_UTILS_LPM_AXES_CALCULATOR_H
#define HSSH_UTILS_LPM_AXES_CALCULATOR_H

#include <memory>
#include <vector>

namespace vulcan
{
namespace laser { struct laser_scan_lines_t; }

namespace hssh
{

class  LocalPerceptualMap;

/**
* lpm_axes_calculator_params_t defines the parameters for the LPMAxesCalculator.
*/
struct lpm_axes_calculator_params_t
{
    double binWidth;            ///< Width of a bin in the histogram in radians
};

/**
* lpm_axes_t defines the axes of an LPM. The orientation of the x-axis within the current LPM
* reference frame and the uncertainty of this measurement are provided.
*/
struct lpm_axes_t
{
    double xAxisOrientation;        ///< Orientation of the x-axis of the local reference frame within the current LPM
    double uncertainty;             ///< Uncertainty measure for the orientation
};

/**
* LPMAxesCalculator locates the major axis of the environment around the robot. The axis is calculated in the current
* LPM reference frame. The approach for calculating the axes uses a histogram of line orientations in the current laser
* scan:
*
*   1) Find the longest line and assign it to be the center of bin 0.
*   2) Place each line in a bin based on the direction.
*   3) Find the bin with the longest cumulative line length.
*   4) For this bin, calculate weighted average direction use line length.
*   5) Can sanity check to see where the other high bins fall. Hopefully, they are about pi/2 away.
*   6) If direction is closer to pi/2 then 0, then orientation of x-axis is dir-pi/2.
*
*/
class LPMAxesCalculator
{
public:

    /**
    * Constructor for LPMAxesCalculator.
    *
    * \param    params          Parameters for this configuration of the axes calculator
    */
    LPMAxesCalculator(const lpm_axes_calculator_params_t& params);

    /**
    * calculateAxes calculates the location of the LPM axes in the current map. The provided data are the
    * lines extracted from the most recent laser scan and the LPM itself.
    *
    * \param    lpm             LPM in which to find the axes
    * \param    lines           Lines extracted from the most recent laser scan used to update the map
    * \return   Axes found in the LPM.
    */
    lpm_axes_t calculateAxes(const LocalPerceptualMap& lpm, const laser::laser_scan_lines_t& lines);

private:

    struct histogram_line_t
    {
        double direction;
        double length;
    };

    struct histogram_bin_t
    {
        double                        length;
        std::vector<histogram_line_t> lines;
    };

    void       clearHistogram     (void);
    void       createLineHistogram(const laser::laser_scan_lines_t& lines);
    double     findStartBin       (const laser::laser_scan_lines_t& lines) const;
    int        findBestBin        (void) const;
    void       verifyBestBin      (const histogram_bin_t& bin) const;
    lpm_axes_t calculateAxes      (const histogram_bin_t& bin) const;

    std::vector<histogram_bin_t> histogram;

    lpm_axes_calculator_params_t params;
};

}
}

#endif // HSSH_UTILS_LPM_AXES_CALCULATOR_H
