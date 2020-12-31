/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     variable.cpp
 * \author   Collin Johnson
 *
 * Definition of Variable.
 */

#include "hssh/local_topological/area_detection/labeling/variable.h"
#include "hssh/local_topological/area_detection/labeling/factor_graph_utils.h"
#include "utils/algorithm_ext.h"
#include <cassert>

// #define DEBUG_MSGS

namespace vulcan
{
namespace hssh
{

Variable::Variable(int id, int numStates) : id_(id), state_(numStates)
{
}

bool Variable::isComplete(void) const
{
    // A msg is complete if there are no unset messages
    return !utils::contains_if(edges_, [](FactorEdge* e) {
        return e->status(MsgDir::to_var) == MsgStatus::unset;
    });
}


void Variable::addEdge(FactorEdge* edge)
{
    assert(edge);
    assert(edge->varId() == id());

    edges_.push_back(edge);
}


int Variable::freshCount(void) const
{
    int numFresh = std::count_if(edges_.begin(), edges_.end(), [](FactorEdge* e) {
        return e->status(MsgDir::to_var) == MsgStatus::fresh;
    });

    return numFresh;
}


int Variable::sendMarginals(bool force)
{
    int numSent = 0;
    consumedEdges_.clear();

    for (auto& edge : edges_) {
        if (force || should_send_message_on_edge(edge, MsgDir::to_var, edges_)) {
#ifdef DEBUG_MSGS
            std::cout << "Preparing message for Factor " << edge->factorId() << " from Var " << id_ << "...\n";
#endif
            state_.ones();
            for (auto& e : edges_) {
                if (e != edge) {
                    state_ %= e->message(MsgDir::to_var);
                    consumedEdges_.push_back(e);
                }
            }
            state_ /= arma::accu(state_);
            // If the message is accepted, it counts as being sent
            if (edge->setMessage(MsgDir::to_factor, state_)) {
                ++numSent;
            }

#ifdef DEBUG_MSGS
            std::cout << state_ << "message sent!\n";
#endif
        }
    }

    // Mark edges that helped send a message as stale
    for (auto& e : consumedEdges_) {
        e->setStatus(MsgDir::to_var, MsgStatus::stale);
    }

    return numSent;
}


Vector Variable::marginal(void)
{
    if (!isComplete()) {
        for (auto& e : edges_) {
            std::cout << "Edge " << e->varId() << "->" << e->factorId() << ": " << e->status(MsgDir::to_var) << '\n';
        }
        assert(isComplete());
    }

    state_.ones();
    for (auto& e : edges_) {
        state_ %= e->message(MsgDir::to_var);
    }

    // Normalize the probabilities
    state_ /= arma::accu(state_);

    return state_;
}

}   // namespace hssh
}   // namespace vulcan
