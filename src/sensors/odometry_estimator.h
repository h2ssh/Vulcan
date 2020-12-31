/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     odometry_estimator.h
 * \author   Collin Johnson
 *
 * Declaration of OdometryEstimator interface and create_odometry_estimator factory.
 */

#ifndef SENSORS_ODOMETRY_ESTIMATOR_H
#define SENSORS_ODOMETRY_ESTIMATOR_H

#include <memory>
#include <string>

namespace vulcan
{
struct odometry_t;

namespace system
{
class ModuleCommunicator;
}
namespace utils
{
class ConfigFile;
}

namespace sensors
{

class OdometryEstimator;

/**
 * create_odometry_estimator creates a new instance of OdometryEstimator using the parameters loaded from the
 * configuration file.
 *
 * The type descriptions for the odometry estimators are defined in the headers for the implementations of
 * the OdometryEstimator interface.
 *
 * NOTE: If type is invalid, the program will be brought down with an assertion
 *
 * \param    type            Type of OdometryEstimator to make
 * \param    config          Configuration file with the parameters for the estimator
 * \return   A pointer to an instance of OdometryEstimator.
 */
std::unique_ptr<OdometryEstimator> create_odometry_estimator(const std::string& type, const utils::ConfigFile& config);

/**
 * OdometryEstimator is an interface for a class that estimates odometry for the robot using some sort of dead-reckoning
 * approach. The traditional method of estimating odometry is using wheel encoders, though scan matching or visual
 * feature matching can also be used. This interface is minimal and simply provides the method for any of these odometry
 * estimation approaches to be integrated with Vulcan.
 *
 * The initialization should subscribe to any necessary data and initialize internal state. The update should provide
 * the most recently updated odometry. The method can either copy a previously calculated value, or do the calculation
 * at that point. Finally, the send method will send out the odometry and any other calculated state using the normal
 * data transmitter. At bare minimum, odometry data should be sent out.
 */
class OdometryEstimator
{
public:
    virtual ~OdometryEstimator(void) { }

    /**
     * initialize initializes the state of the estimator. A DataReceiver is provided so a subclass can subscribe to any
     * necessary sensor data for calculating the robot's odometry.
     *
     * \param    communicator        Producer from which to subscribe to data
     */
    virtual void initialize(system::ModuleCommunicator& communicator) = 0;

    /**
     * update provides an updated estimate of the robot's odometry. The provided odometry estimate needs to be new data.
     * If no new data has been calculated, the call to update should block until data is available.
     *
     * \return   Newly calculated odometry data.
     */
    virtual odometry_t update(void) = 0;

    /**
     * send sends out the calculated odometry data, along with any other state being estimated, like wheel encoder
     * information.
     *
     * \param    communicator    Communicator instance to use for sending out the new data
     */
    virtual void send(system::ModuleCommunicator& communicator) = 0;
};

}   // namespace sensors
}   // namespace vulcan

#endif   // SENSORS_ODOMETRY_ESTIMATOR_H
