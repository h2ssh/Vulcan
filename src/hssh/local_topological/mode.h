/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file
 * \author   Collin Johnson
 *
 * Declaration of possible modes for local_topo_hssh.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_MODE_H
#define HSSH_LOCAL_TOPOLOGICAL_MODE_H

namespace vulcan
{
namespace hssh
{

/**
 * Possible states for the local_topo_hssh module:
 */
enum class LocalTopoMode
{
    label_only,        ///< Only run the labeling process because the robot is in an area of perceptual uncertainty
    event_detection,   ///< Run the labeling and event detection
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_MODE_H
