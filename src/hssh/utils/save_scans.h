/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     save_scans.h
* \author   Paul Foster
* 
* Functions to allow saving the scans used in mapping to file
* 
*   - accumulate_scan  :  stores a copy of the scan to a list of scans used
*   - save_scans       :  dumps the stored scans to disk
*/

#ifndef HSSH_UTILS_SAVE_SCANS_H
#define HSSH_UTILS_SAVE_SCANS_H

#include <string>


namespace vulcan
{
namespace hssh
{

struct map_update_data_t;


/**
* accumulate_scan
* 
* \param    scan      Maximum number of features to consider
*/
void accumulate_scan(const map_update_data_t& scan);

/**
* save_scans
* 
* \param    filename File to save to
*/
void save_scans(const std::string& filename);

}
}

#endif // HSSH_UTILS_SAVE_SCANS_H