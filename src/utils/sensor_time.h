/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     sensor_time.h
 * \author   Collin Johnson
 *
 * Declaration of SensorTime.
 */

#ifndef UTILS_SENSOR_TIME_H
#define UTILS_SENSOR_TIME_H

#include <cstdint>

namespace vulcan
{
namespace utils
{

/**
 * SensorTime calculates the time at which a piece of sensor data was measured in the system/robot time frame. The basic
 * idea is that there's an underlying offset between the sensor clock and the system clock, A. Accounting for clock
 * drift, this offset varies with time, A_i. In addition to the offset between the clocks, there's an additional unknown
 * latency, e_i, that is the transmission delay, plus the time data spends in buffers. Occasionally, a good, low-latency
 * measurement arrives. We can use this measurement to determine the offset A_i.
 *
 * With s_i = sensor time, t_i = system time of measurement, q_i = time measurement is received, e_i = system latency,
 * and A_i = offset between the sensor and system clocks, in the no-drift case we get:
 *
 *       s_i = A_i + t_i
 *       q_i = t_i + e_i
 *
 * and   s_i - q_i = A_i - e_i
 *
 * Now, this means that latency is minimized with s_i - q_i is maximized. If we save the A_i associated with this
 * maximum offset, we can use it to convert the sensor's time into the system time and get a more accurate estimate of
 * the time at which a measurement was taken. This estimation of A_i can continue for the duration of the sensor's run,
 * meaning that the estimate will improve over time.
 *
 * The situation becomes more complicated when clock drift is accounted for. To get the full details on the
 * implementation as used in SensorTime, see the paper:
 *
 *   "A Passive Solution to the Sensor Synchronization Problem" by Edwin Olson
 *
 */
class SensorTime
{
public:
    /**
     * Constructor for SensorTime.
     *
     * \param    slowDriftRate           Drift rate if sensor clock is slower (alpha_2 in the paper) (default = 0.01)
     * \param    fastDriftRate           Drift rate if sensor clock is faster (alpha_1 in the paper) (default = 0.01)
     */
    explicit SensorTime(double slowDriftRate = 0.001, double fastDriftRate = 0.001);

    /**
     * timestamp retrieves the system timestamp associated with the sensor's time using the above mentioned algorithm.
     *
     * This timestamp should be called as soon as a sensor measurement is received to minimize any additional
     * computation latency into the measurement.
     *
     * \param    sensorTime           Timestamp with the sensor's clock
     * \return   Timestamp with the system's clock for the provided sensorTime.
     */
    int64_t timestamp(int64_t sensorTime);

private:
    const double slowRate_;
    const double fastRate_;

    int64_t sensorTimeForOffset_;   ///< Sensor time measured for the best clock offset estimate (p in Algorithm 1)
    int64_t systemTimeForOffset_;   ///< System time measured for the best clock offset estimate (q in Algorithm 1)

    int64_t previousSensorTime_;   ///< Last measurement of the sensor time
    int64_t previousSystemTime_;   ///< Last measurement of uncorrected system

    int64_t totalSensorTimeElapsed_;   ///< Total sensor time elapsed for duration of program
    int64_t totalSystemTimeElapsed_;   ///< Total system time elapsed for duration of program

    int numUpdates_;   ///< Number of updates to the time
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_SENSOR_TIME_H
