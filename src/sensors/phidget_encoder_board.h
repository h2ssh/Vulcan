/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     phidget_encoder_board.h
 * \author   Collin Johnson
 *
 * Declaration of PhidgetEncoderBoard for interacting with the Phidget USB encoder module.
 */

#ifndef VGO_ROBOT
    #ifndef SENSORS_PHIDGET_ENCODER_BOARD_H
        #define SENSORS_PHIDGET_ENCODER_BOARD_H

        #include "sensors/wheel_encoders.h"
        #include "utils/condition_variable.h"
        #include "utils/mutex.h"
        #include "utils/sensor_time.h"
        #include <phidget21.h>

namespace vulcan
{
namespace sensors
{

const std::string PHIDGET_ENCODER_BOARD_TYPE("phidget");

/**
 * PhidgetEncoderBoard is an subclass of the WheelEncoders base class. It talks with a Phidget
 * encoder board via the Phidget libraries to calculate wheel odometry for the robot.
 *
 * The config parameters for PhidgetEncoderBoard are:
 *
 *   [PhidgetEncoderBoardParameters]
 *   serial_number       = serial # of the board, so the correct board is opened
 *   left_encoder_index  = index to which left encoder is attached
 *   right_encoder_index = index to which right encoder is attached
 */
class PhidgetEncoderBoard : public WheelEncoders
{
public:
    /**
     * Constructor for PhidgetEncoderBoard.
     *
     * \param    params          Parameters for connecting to the board and for the WheelEncoders base class
     */
    PhidgetEncoderBoard(const wheel_encoders_params_t& params);

    virtual ~PhidgetEncoderBoard(void);

    // WheelEncoders interface
    virtual encoder_data_t getEncoders(void);
    virtual void resetEncoders(void);

private:
    // Friend functions to act as handlers for the Phidget callbacks
    friend int phidget_encoder_attach_handler(CPhidgetHandle encoder, void* phidgetInstance);
    friend int phidget_encoder_detach_handler(CPhidgetHandle encoder, void* phidgetInstance);
    friend int phidget_encoder_error_handler(CPhidgetHandle encoder,
                                             void* phidgetInstance,
                                             int errorCode,
                                             const char* errorDescription);
    friend int phidget_position_change(CPhidgetEncoderHandle encoder,
                                       void* phidgetInstance,
                                       int index,
                                       int time,
                                       int relativePosition);
    friend int phidget_index_change(CPhidgetEncoderHandle encoder, void* phidgetInstance, int index, int position);

    CPhidgetEncoderHandle encoder;

    encoder_data_t encoderData;

    int64_t previousEncoderTimestamp;

    int64_t totalRightEncoderTime_;
    int64_t totalLeftEncoderTime_;
    utils::SensorTime leftTime_;
    utils::SensorTime rightTime_;

    int64_t lastLeftIndexTime;
    int64_t lastRightIndexTime;

    phidget_encoder_board_params_t params;

    utils::Mutex tickLock;
    utils::Mutex indexLock;
    utils::ConditionVariable encoderTrigger;
};

// Callbacks used with the Phidget C API
int phidget_encoder_attach_handler(CPhidgetHandle encoder, void* phidgetInstance);
int phidget_encoder_detach_handler(CPhidgetHandle encoder, void* phidgetInstance);
int phidget_encoder_error_handler(CPhidgetHandle encoder,
                                  void* phidgetInstance,
                                  int errorCode,
                                  const char* errorDescription);
int phidget_position_change(CPhidgetEncoderHandle encoder,
                            void* phidgetInstance,
                            int index,
                            int time,
                            int relativePosition);
int phidget_index_change(CPhidgetEncoderHandle encoder, void* phidgetInstance, int index, int position);

}   // namespace sensors
}   // namespace vulcan

    #endif   // SENSORS_PHIDGET_ENCODER_BOARD_H
#endif       // VGO_ROBOT
