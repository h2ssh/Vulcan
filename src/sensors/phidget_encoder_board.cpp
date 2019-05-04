/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     phidget_encoder_board.cpp
* \author   Collin Johnson
*
* Definition of PhidgetEncoderBoard.
*/

#ifndef VGO_ROBOT

#include <sensors/phidget_encoder_board.h>
#include <utils/auto_mutex.h>
#include <utils/timestamp.h>
#include <iostream>
#include <cassert>

// #define DEBUG_TIME

namespace vulcan
{
namespace sensors
{

PhidgetEncoderBoard::PhidgetEncoderBoard(const wheel_encoders_params_t& params)
: WheelEncoders(params)
, encoder(0)
, totalRightEncoderTime_(0)
, totalLeftEncoderTime_(0)
, leftTime_(0.05, 0.05)
, rightTime_(0.05, 0.05)
, lastLeftIndexTime(0)
, lastRightIndexTime(0)
, params(params.phidgetParams)
, encoderTrigger(false)
{

    encoderData.leftTicksPerRevolution  = params.configuration.leftTicksPerRevolution;
    encoderData.leftWheelCircumference  = params.configuration.leftWheelCircumference;
    encoderData.rightTicksPerRevolution = params.configuration.rightTicksPerRevolution;
    encoderData.rightWheelCircumference = params.configuration.rightWheelCircumference;
    encoderData.wheelbase               = params.configuration.wheelbase;

    CPhidgetEncoder_create(&encoder);
    
    std::cout << "Created Phidget encoder...\n";

    CPhidget_set_OnAttach_Handler(reinterpret_cast<CPhidgetHandle>(encoder), phidget_encoder_attach_handler, this);
    CPhidget_set_OnDetach_Handler(reinterpret_cast<CPhidgetHandle>(encoder), phidget_encoder_detach_handler, this);
    CPhidget_set_OnError_Handler (reinterpret_cast<CPhidgetHandle>(encoder), phidget_encoder_error_handler, this);

    CPhidgetEncoder_set_OnPositionChange_Handler(encoder, phidget_position_change, this);
    CPhidgetEncoder_set_OnIndex_Handler(encoder, phidget_index_change, this);

    CPhidget_open(reinterpret_cast<CPhidgetHandle>(encoder), this->params.serialNumber);

    CPhidget_waitForAttachment(reinterpret_cast<CPhidgetHandle>(encoder), 0);

    std::cout<<"INFO:PhidgetEncoderBoard: Successfully attached to encoder board: "<<this->params.serialNumber<<'\n';
}


PhidgetEncoderBoard::~PhidgetEncoderBoard(void)
{
    CPhidget_close(reinterpret_cast<CPhidgetHandle>(encoder));
    CPhidget_delete(reinterpret_cast<CPhidgetHandle>(encoder));
}


encoder_data_t PhidgetEncoderBoard::getEncoders(void)
{
    encoderTrigger.wait();

    utils::AutoMutex autoTickLock(tickLock);
    utils::AutoMutex autoIndexLock(indexLock);

    encoder_data_t ticks = encoderData;

    // If the time hasn't changed between pulls of the encoder data, then update the timestamp manually. Otherwise, go with the
    // time that was set by the event handler, which will be closer to the time that the data actually arrived.
    if(previousEncoderTimestamp == ticks.timestamp)
    {
        // Time should not remain the same!
        assert(previousEncoderTimestamp != ticks.timestamp);
        ticks.timestamp = utils::system_time_us();
    }

    previousEncoderTimestamp = encoderData.timestamp;

    encoderTrigger.setPredicate(false);

    return ticks;
}


void PhidgetEncoderBoard::resetEncoders(void)
{
    utils::AutoMutex autoTickLock(tickLock);
    utils::AutoMutex autoIndexLock(indexLock);

    encoderData.leftIndexPulseTotal  = 0;
    encoderData.rightIndexPulseTotal = 0;

    encoderData.leftTicksTotal  = 0;
    encoderData.rightTicksTotal = 0;
}


int phidget_encoder_attach_handler(CPhidgetHandle encoder, void* phidgetInstance)
{
    PhidgetEncoderBoard* board = static_cast<PhidgetEncoderBoard*>(phidgetInstance);

    //Retrieve the device ID and number of encoders so that we can set the enables if needed
    CPhidget_DeviceID deviceId;
    int               inputCount;
    CPhidget_getDeviceID(encoder, &deviceId);
    CPhidgetEncoder_getEncoderCount(board->encoder, &inputCount);

    //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047
    if(deviceId == PHIDID_ENCODER_HS_4ENCODER_4INPUT)
    {
        std::cout<<"Encoder requires Enable. Enabling inputs....\n";
        CPhidgetEncoder_setEnabled(board->encoder, board->params.leftIndex, 1);
        CPhidgetEncoder_setEnabled(board->encoder, board->params.rightIndex, 1);
    }

    std::cout<<"INFO:PhidgetEncoderBoard: Connected to encoder board: "<<board->params.serialNumber<<'\n';

    return 0;
}


int phidget_encoder_detach_handler(CPhidgetHandle encoder, void* phidgetInstance)
{
    std::cerr<<"ERROR:PhidgetEncoderBoard: Encoder board detached!"<<std::endl;

    return 0;
}


int phidget_encoder_error_handler(CPhidgetHandle encoder, void* phidgetInstance, int errorCode, const char* errorDescription)
{
    std::cerr<<"ERROR:PhidgetEncoderBoard:"<<errorCode<<":"<<errorDescription<<'\n';

    return 0;
}


int phidget_position_change(CPhidgetEncoderHandle encoder, void* phidgetInstance, int index, int time, int relativePosition)
{
    PhidgetEncoderBoard* board = static_cast<PhidgetEncoderBoard*>(phidgetInstance);

    utils::AutoMutex autoLock(board->tickLock);
    
    if(index == board->params.leftIndex)
    {
        board->encoderData.leftTicksTotal += -relativePosition;
        board->totalLeftEncoderTime_ += time;
        board->encoderData.timestamp = board->leftTime_.timestamp(board->totalLeftEncoderTime_);
    }
    else if(index == board->params.rightIndex)
    {
        board->encoderData.rightTicksTotal += -relativePosition;
        board->totalRightEncoderTime_ += time;
        board->encoderData.timestamp = board->rightTime_.timestamp(board->totalRightEncoderTime_);
    }
    else
    {
        std::cerr<<"ERROR:PhidgetEncoderBoard: Unknown source of encoder ticks at index "<<index<<'\n';
    }
    
    assert(time >= 0);

#ifdef DEBUG_TIME
    std::cout << "DEBUG:PhidgetEncoderBoard: Time:" << time <<" Position change:" << relativePosition << " Left time:" << board->totalLeftEncoderTime_ << " Right time:"
        << board->totalRightEncoderTime_ << " Delta:" << (board->totalRightEncoderTime_ - board->totalLeftEncoderTime_) << " Timestamp:" << board->encoderData.timestamp << '\n';
#endif

    // Left and right wheel encoders are read in alternating order. Wait for both before updating by just triggering on the left wheel.
    if(index == board->params.leftIndex)
    {
        board->encoderTrigger.setPredicate(true);
        board->encoderTrigger.broadcast();
    }
    
    return 0;
}


int phidget_index_change(CPhidgetEncoderHandle encoder, void* phidgetInstance, int index, int position)
{
    PhidgetEncoderBoard* board = static_cast<PhidgetEncoderBoard*>(phidgetInstance);

    utils::AutoMutex autoLock(board->indexLock);

    int64_t indexTime = utils::system_time_us();

    if(index == board->params.leftIndex)
    {
        board->encoderData.leftIndexPulseTotal += 1;

        if(board->lastLeftIndexTime)
        {
            board->encoderData.leftRPM = utils::usec_to_sec(indexTime - board->lastLeftIndexTime);
        }

        board->lastLeftIndexTime = indexTime;
    }
    else if(index == board->params.rightIndex)
    {
        board->encoderData.rightIndexPulseTotal += 1;

        if(board->lastRightIndexTime)
        {
            board->encoderData.rightRPM = utils::usec_to_sec(indexTime - board->lastRightIndexTime);
        }

        board->lastRightIndexTime = indexTime;
    }

    return 0;
}

} // namespace sensors
} // namespace vulcan

#endif
