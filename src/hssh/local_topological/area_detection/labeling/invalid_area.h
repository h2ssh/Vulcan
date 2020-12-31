/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     invalid_area.h
 * \author   Collin Johnson
 *
 * Definition of InvalidAreaException.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_INVALID_AREA_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_INVALID_AREA_H

#include <exception>
#include <string>

namespace vulcan
{
namespace hssh
{

/**
 * InvalidAreaException is thrown whenever an attempt to create a LocalArea or AreaProposal fails because a constraint
 * from the topological representation has been violated.
 */
class InvalidAreaException : public std::exception
{
public:
    InvalidAreaException(std::string description) : error_(description) { }

    // std::exception interface
    const char* what() const noexcept override { return error_.c_str(); }

private:
    std::string error_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_INVALID_AREA_H
