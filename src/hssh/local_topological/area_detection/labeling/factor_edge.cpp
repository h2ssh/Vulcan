/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     factor_edge.cpp
 * \author   Collin Johnson
 *
 * Definition of FactorEdge.
 */

#include "hssh/local_topological/area_detection/labeling/factor_edge.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

int dir_idx(MsgDir direction)
{
    return static_cast<int>(direction);
}


std::ostream& operator<<(std::ostream& out, MsgDir dir)
{
    switch (dir) {
    case MsgDir::to_var:
        out << "to_var";
        break;
    case MsgDir::to_factor:
        out << "to_fact";
        break;
    }
    return out;
}


std::ostream& operator<<(std::ostream& out, MsgStatus status)
{
    switch (status) {
    case MsgStatus::unset:
        out << "unset";
        break;
    case MsgStatus::stale:
        out << "stale";
        break;
    case MsgStatus::fresh:
        out << "fresh";
        break;
    }
    return out;
}


FactorEdge::FactorEdge(int varId, int factorId, int numStates) : numStates_(numStates)
{
    ids_[dir_idx(MsgDir::to_var)] = varId;
    ids_[dir_idx(MsgDir::to_factor)] = factorId;

    status_[0] = MsgStatus::unset;
    status_[1] = MsgStatus::unset;

    msgs_[0] = Vector(numStates);
    msgs_[1] = Vector(numStates);
}


int FactorEdge::varId(void) const
{
    return ids_[dir_idx(MsgDir::to_var)];
}


int FactorEdge::factorId(void) const
{
    return ids_[dir_idx(MsgDir::to_factor)];
}


Vector FactorEdge::message(MsgDir direction) const
{
    return msgs_[dir_idx(direction)];
}


MsgStatus FactorEdge::status(MsgDir direction) const
{
    return status_[dir_idx(direction)];
}


bool FactorEdge::setMessage(MsgDir direction, const Vector& message)
{
    assert(msgs_[dir_idx(direction)].n_elem == message.n_elem);

    Vector diff = arma::abs(message - msgs_[dir_idx(direction)]);
    bool shouldChange = (changeThreshold_ == 0.0) || arma::any(diff > changeThreshold_);

    if (shouldChange) {
        msgs_[dir_idx(direction)] = message;
        setStatus(direction, MsgStatus::fresh);
    }

    return shouldChange;
}


void FactorEdge::setStatus(MsgDir direction, MsgStatus status)
{
    status_[dir_idx(direction)] = status;
}


void FactorEdge::setMessagesToUnity(void)
{
    msgs_[0].ones();
    msgs_[1].ones();
    status_[0] = MsgStatus::fresh;
    status_[1] = MsgStatus::fresh;
}

}   // namespace hssh
}   // namespace vulcan
