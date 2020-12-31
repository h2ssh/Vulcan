/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_io.h
 * \author   Collin Johnson
 *
 * Declaration of I/O functions for saving/loading laser scans.
 */

#ifndef LASER_LASER_IO_H
#define LASER_LASER_IO_H

#include <string>

namespace vulcan
{
struct polar_laser_scan_t;

namespace laser
{

/**
 * save_laser_scan_to_file saves a polar_laser_scan_t to disk in a plain text format. Plain text is used
 * to allow the data to be easily read by Matlab.
 *
 * The format is:
 *
 * timestamp scan_id laser_model start_angle resolution max_range x_offset y_offset theta_offset intensity_flag
 * num_ranges range_data intensity_data
 *
 *   - Each value is separated by whitespace.
 *   - intensity_flag is 1 if intensity data was captured in the scan.
 *   - (x_offset, y_offset, theta_offset) is transform from robot center to laser center.
 *
 * \param    scan            Scan to be saved
 * \param    file            File in which to save the scan
 * \return   True if scan was saved successfully. False otherwise.
 */
bool save_laser_scan_to_file(const polar_laser_scan_t& scan, std::ofstream& file);

/**
 * load_laser_scan_from_file loads a polar_laser_scan_t saved to disk in the format described in the
 * save_laser_scan_to_file function.
 *
 * \param    file            File where the scan is saved
 * \param    scan            Scan loaded from the file        (output)
 * \return   True if the scan was loaded successfully. False otherwise.
 */
bool load_scan_from_file(std::ifstream& file, polar_laser_scan_t& scan);

}   // namespace laser
}   // namespace vulcan

#endif   // LASER_LASER_IO_H
