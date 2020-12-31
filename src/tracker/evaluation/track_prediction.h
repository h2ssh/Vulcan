/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     track_prediction.h
 * \author   Collin Johnson
 *
 * Declaration of functions to predict an ObjectTrack given an ObjectTrack and other relevant state:
 *
 *   - predict_track_using_motion_state : predict future track using just the (x, y, v_x, v_y, a_x, a_y) information
 */

#ifndef TRACKER_EVALUATION_TRACK_PREDICTION_H
#define TRACKER_EVALUATION_TRACK_PREDICTION_H

#include "tracker/evaluation/object_track.h"

namespace vulcan
{
namespace tracker
{

/**
 * predict_track_using_motion_state
 *
 * The state in the returned track will be predictionMs in the future from track's first state. Thus, a comparison will
 * need to take into account the offset. Once identifying the first member, then all future states will have the same
 * timestamps. Thus, the closest time gap to predictionMs will be used to avoid the need for interpolation.
 *
 * \param    track           Track from which predictions will be generated
 * \param    predictionMs    How long into the future to predict the motion
 * \return   ObjectTrack containing predictions into the future.
 */
ObjectTrack predict_track_using_motion_state(const ObjectTrack& track, int predictionMs);


}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_EVALUATION_TRACK_PREDICTION_H
