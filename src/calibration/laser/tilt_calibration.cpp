/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tilt_calibration.cpp
 * \author   Collin Johnson
 *
 * Definition of calibrate_laser_pitch.
 */

#include "calibration/laser/tilt_calibration.h"
#include "math/regression.h"
#include <boost/range/iterator_range.hpp>
#include <iostream>

namespace vulcan
{
namespace calibration
{

using Measurements = std::vector<Point<double>>;


tilt_calibration_results_t calibrate_laser_tilt(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                                std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                                int lineStartIndex,
                                                int lineEndIndex,
                                                double pitchStepSize,
                                                double maxPitch,
                                                double rollStepSize,
                                                double maxRoll);
Measurements extract_line_measurements(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                       std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                       int lineStartIndex,
                                       int lineEndIndex);
Point<double> apply_tilt(const Point<double>& point, double pitch, double roll);


tilt_calibration_results_t calibrate_laser_pitch(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                                 std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                                 int lineStartIndex,
                                                 int lineEndIndex,
                                                 double minStepSize,
                                                 double maxPitch)
{
    return calibrate_laser_tilt(beginLaser, endLaser, lineStartIndex, lineEndIndex, minStepSize, maxPitch, 0.0, 0.0);
}


tilt_calibration_results_t calibrate_laser_roll(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                                std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                                int lineStartIndex,
                                                int lineEndIndex,
                                                double minStepSize,
                                                double maxRoll)
{
    return calibrate_laser_tilt(beginLaser, endLaser, lineStartIndex, lineEndIndex, 0.0, 0.0, minStepSize, maxRoll);
}


tilt_calibration_results_t calibrate_laser_tilt(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                                std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                                int lineStartIndex,
                                                int lineEndIndex,
                                                double pitchStepSize,
                                                double maxPitch,
                                                double rollStepSize,
                                                double maxRoll)
{
    assert(lineEndIndex > lineStartIndex + 5);
    assert(endLaser > beginLaser);

    std::cout << "Beginning tilt calibration for " << std::distance(beginLaser, endLaser) << " laser scans using "
              << "indices " << lineStartIndex << "->" << lineEndIndex << " for fitting the line.\n";

    tilt_calibration_results_t results;

    double minError = std::numeric_limits<double>::max();

    Measurements measurements = extract_line_measurements(beginLaser, endLaser, lineStartIndex, lineEndIndex);
    Measurements pitchMeasurements = measurements;

    auto pitchLine = math::total_least_squares(measurements.begin(), measurements.end());

    for (double pitch = 0.0, roll = 0.0; pitch >= -maxPitch && roll >= -maxRoll;
         pitch -= pitchStepSize, roll -= rollStepSize) {
        std::transform(measurements.begin(),
                       measurements.end(),
                       pitchMeasurements.begin(),
                       [pitch, roll](Point<double> point) {
                           return apply_tilt(point, pitch, roll);
                       });

        double error = 0;

        for (auto point : pitchMeasurements) {
            error += distance_to_line(point, pitchLine);
        }

        tilt_t tilt(pitch, roll);
        error /= pitchMeasurements.size();
        results.tiltErrors.emplace_back(tilt, error);

        std::cout << "Pitch: " << pitch << " Roll:" << roll << " Line: " << pitchLine << " Error:" << error << '\n';

        if (error < minError) {
            results.bestTilt = tilt;
            minError = error;
        }
    }

    std::cout << "Finished tilt calibration: Pitch:" << results.bestTilt.pitch << " Roll:" << results.bestTilt.roll
              << '\n';

    return results;
}


Measurements extract_line_measurements(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                       std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                       int lineStartIndex,
                                       int lineEndIndex)
{
    Measurements measurements;
    measurements.reserve(std::distance(beginLaser, endLaser) * (lineEndIndex - lineStartIndex));
    cartesian_laser_scan_t cartesian;

    for (auto& scan : boost::make_iterator_range(beginLaser, endLaser)) {
        polar_scan_to_cartesian_scan(scan, cartesian);

        std::copy(cartesian.scanPoints.begin() + lineStartIndex,
                  cartesian.scanPoints.begin() + lineEndIndex,
                  std::back_inserter(measurements));
    }

    std::cout << "INFO: tilt_calibration: Using " << measurements.size() << " measurements for fitting.\n";

    return measurements;
}


Point<double> apply_tilt(const Point<double>& point, double pitch, double roll)
{
    return Point<double>(point.x * std::cos(pitch), point.y * std::cos(roll));
}

}   // namespace calibration
}   // namespace vulcan
