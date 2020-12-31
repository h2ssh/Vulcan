/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     service.h
 * \author   Collin Johnson
 *
 * Declaration of vgo_service_t.
 */

#ifndef ROBOT_VGO_SERVICE_H
#define ROBOT_VGO_SERVICE_H

#include <cstdint>
#include <string>

namespace vulcan
{
namespace robot
{

/**
 * vgo_service_task_t enumerates the possible tasks for a service.
 */
enum vgo_service_task_t
{
    VGO_SERVICE_SET,   ///< Set the service to the value
    VGO_SERVICE_GET    ///< Get is the result of the service
};

/**
 * vgo_service_type_t enumerates all types of services on the VGo.
 */
enum vgo_service_type_t
{
    VGO_AUX_SENSORS_POWER,
    VGO_HEAD_POWER,
    VGO_SHIRT_COLOR,
    VGO_PASSTHROUGH_STATUS
};


/**
 * vgo_service_t describes a service performed on the VGo. The type of service, the task (get or set), and the
 * corresponding value are stored.
 *
 * For get services, the initial request should have an empty content string. THe returned service value will
 * have data in the content string.
 */
struct vgo_service_t
{
    int64_t timestamp;

    vgo_service_type_t type;
    vgo_service_task_t task;

    std::string content;
};

}   // namespace robot
}   // namespace vulcan

#endif   // ROBOT_VGO_SERVICE_H
