/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor_edge.h
* \author   Collin Johnson
* 
* Declaration of FactorEdge.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_EDGE_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_EDGE_H

#include "core/vector.h"
#include <array>
#include <memory>

namespace vulcan 
{
namespace hssh 
{
    
/**
* MsgDir encodes the direction of a message along the edge.
*/
enum class MsgDir
{
    to_var,         ///< Message from a factor to a variable
    to_factor,      ///< Message from a variable to a factor
};

/**
* MsgStatus encodes the status of a message along the edge.
*/
enum class MsgStatus
{
    unset,          ///< No values have been assigned to the message
    stale,          ///< Values have been set and used for this message
    fresh,          ///< A new value has been set for the message, but it hasn't been read yet
};

std::ostream& operator<<(std::ostream& out, MsgDir dir);
std::ostream& operator<<(std::ostream& out, MsgStatus status);

/**
* FactorEdge is an edge between a factor and a variable.
*/
class FactorEdge
{
public:
    
    using Ptr = std::shared_ptr<FactorEdge>;
    
    /**
    * Constructor for FactorEdge.
    * 
    * \param    varId           Id of the variable attached to the edge
    * \param    factorId        Id of the factor attached to the edge
    * \param    numStates       Number of states for messages traveling along the edge
    * \pre  varId != factorId
    * \pre  numStates > 0
    */
    FactorEdge(int varId, int factorId, int numStates);
    
    /**
    * varId retrieves the id of the variable node associated with this edge.
    */
    int varId(void) const;
    
    /**
    * factorId retrieves the id of the factor node associated with this edge.
    */
    int factorId(void) const;
    
    /**
    * numStates retrieves the number of states for the message that flows along this edge.
    */
    int numStates(void) const { return numStates_; }
    
    /**
    * message retrieves the message with the specified direction.
    * 
    * \param    direction           Direction of message to retrieve
    * \return   Message traveling in the specified direction.
    */
    Vector message(MsgDir direction) const;
    
    /**
    * status retrieves the status of the message in the specified direction.
    * 
    * \param    direction           Direction of the message
    * \return   Status of message moving in direction.
    */
    MsgStatus status(MsgDir direction) const;
    
    /**
    * setMessage sets the message in the given direction. Setting the message makes the status go to fresh.
    * 
    * \param    direction           Direction the message is going
    * \param    message             Contents of the message
    * \pre  message.n_elem == numStates
    * \return   True if the message changed the state of the edge. False if it was the same value as before
    *   and was ignored.
    */
    bool setMessage(MsgDir direction, const Vector& message);
    
    /**
    * setStatus sets the status of a message in the given direction.
    * 
    * \param    direction           Direction of the message
    * \param    status              New status to assign to the message
    */
    void setStatus(MsgDir direction, MsgStatus status);
    
    /**
    * setChangeThreshold changes the threshold at which a change is deemed inconsequential and thus doesn't result in
    * a fresh message being created. Setting this threshold allows for an easy way to see if belief propagation is
    * converging. Eventually a converged network will generate only stale messages so the updates should stop.
    * 
    * \param    threshold           New change threshold to use
    */
    void setChangeThreshold(double threshold) { changeThreshold_ = std::max(threshold, 0.0); }
    
    /**
    * setMessagesToUnity sets the messages to be 1 for all states to initialize the edge for loopy BP.
    */
    void setMessagesToUnity(void);

private: 
    
    int numStates_;
    std::array<int, 2> ids_;
    std::array<MsgStatus, 2> status_;
    std::array<Vector, 2> msgs_;
    double changeThreshold_ = 0.0;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_EDGE_H
