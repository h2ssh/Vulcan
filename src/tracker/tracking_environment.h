/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tracking_environment.h
 * \author   Collin Johnson
 *
 * Definition of tracking_environment_t.
 */

#ifndef TRACKER_TRACKING_ENVIRONMENT_H
#define TRACKER_TRACKING_ENVIRONMENT_H

namespace vulcan
{
namespace hssh
{
class LocalPerceptualMap;
}
namespace hssh
{
class LocalTopoMap;
}
namespace tracker
{

/**
 * tracking_environment_t contains representations of the environment for use with goal estimation.
 */
struct tracking_environment_t
{
    int64_t timestamp;
    const hssh::LocalPerceptualMap* lpm;
    const hssh::LocalTopoMap* ltm;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_TRACKING_ENVIRONMENT_H
