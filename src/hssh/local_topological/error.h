/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     error.h
 * \author   Collin Johnson
 *
 * Definition of types pertaining to errors that can occur while constructing the local topological map:
 *
 *   - LabelingError : types of errors that can cause the failure of the labeling process
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_ERROR_H
#define HSSH_LOCAL_TOPOLOGICAL_ERROR_H

namespace vulcan
{
namespace hssh
{

/**
 * LabelingError provides a descriptor for the different types of errors that can occur during the place detection
 * and labeling process.
 */
enum class LabelingError
{
    success,                      ///< no error occurred
    no_labeling_solution,         ///< no solution can be found to the labeling problem
    invalid_transition_gateway,   ///< the gateway through which a transition occurred doesn't exist or is invalid
    exit_area_not_found,          ///< the exit area for the last transition no longer exists in the map, but it should
    invalid_exit_area,            ///< the exit area for the last transition fails constraints on area labels
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_ERROR_H
