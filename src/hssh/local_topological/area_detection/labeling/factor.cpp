/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor.cpp
* \author   Collin Johnson
*
* Definition of Factor.
*/

#include <hssh/local_topological/area_detection/labeling/factor.h>
#include <hssh/local_topological/area_detection/labeling/factor_graph_utils.h>
#include <utils/algorithm_ext.h>
#include <cassert>

// #define DEBUG_MSGS

namespace vulcan
{
namespace hssh
{

int state_index(const std::vector<std::pair<int, int>>& states, const std::unordered_map<int, int>& varToOffset);


Factor::Factor(int id, const std::vector<Variable*>& variables, const std::vector<FactorStateProb>& probs)
: id_(id)
, stateProbs_(probs.size())
{
    assert(!variables.empty());

    auto sortedVars = variables;
    std::sort(sortedVars.begin(), sortedVars.end(), [](const auto& lhs, const auto& rhs) {
        return lhs->id() < rhs->id();
    });

    for(auto& v : sortedVars)
    {
        vars_.push_back(v->id());
        varStates_.push_back(v->numStates());
    }

    std::vector<int> offsets;
    offsets.push_back(1);
    for(std::size_t n = 1; n < varStates_.size(); ++n)
    {
        offsets[n] = offsets[n - 1] * varStates_[n - 1];
    }

    // Fill out the var->index offset
    varStateIndex_.reserve(vars_.size());
    for(std::size_t n = 0; n < vars_.size(); ++n)
    {
        varStateIndex_[vars_[n]] = offsets[n];
    }

    for(auto v : sortedVars)
    {
        assert(varStateIndex_.find(v->id()) != varStateIndex_.end());
    }

    for(auto& prob : probs)
    {
        int index = state_index(prob.states, varStateIndex_);
        assert(index < static_cast<int>(stateProbs_.n_elem));
        stateProbs_(index) = prob.prob;
    }
}


bool Factor::isComplete(void) const
{
    // A msg is complete if there are no unset messages
    return !utils::contains_if(edges_, [](FactorEdge* e) { return e->status(MsgDir::to_var) == MsgStatus::unset; });
}


void Factor::addEdge(FactorEdge* edge)
{
    assert(edge->factorId() == id_);
    assert(utils::contains(vars_, edge->varId()));

    edges_.push_back(edge);

    // Maintain the same sorting as the variables
    std::sort(edges_.begin(), edges_.end(), [](auto lhs, auto rhs) { return lhs->varId() < rhs->varId(); });
}


int Factor::freshCount(void) const
{
    int numFresh = std::count_if(edges_.begin(), edges_.end(), [](FactorEdge* e) {
        return e->status(MsgDir::to_factor) == MsgStatus::fresh;
    });

    return numFresh;
}


int Factor::sendMarginals(UpdateType type, UpdateRule rule)
{
    int numSent = 0;
    consumedEdges_.clear();

    for(auto& edge : edges_)
    {
        // If only one edges, there's always a message to send!
        Vector msg;
        if((rule == UpdateRule::force) || should_send_message_on_edge(edge, MsgDir::to_factor, edges_))
        {
            msg = computeEdgeMarginal(edge, type);
#ifdef DEBUG_MSGS
            std::cout << "Preparing message for Var " << edge->varId() << " from Factor " << id_ << " with "
                << edges_.size() << " edges...\n";
        } else {
            std::cout << "Ignored sending messages Factor " << id_ << " with " << edges_.size() << " edges...\n";
#endif
        }

        // If the message was set, then send it away
        if(!msg.empty())
        {
#ifdef DEBUG_MSGS
            std::cout << msg << "message sent!\n";
#endif
            msg /= arma::accu(msg);
            // If the message changed the edge value, it counts as being sent.
            if(edge->setMessage(MsgDir::to_var, msg))
            {
                ++numSent;
            }
        }
    }

    for(auto& e : consumedEdges_)
    {
        e->setStatus(MsgDir::to_factor, MsgStatus::stale);
    }

    return numSent;
}


Vector Factor::computeEdgeMarginal(FactorEdge* edge, UpdateType type)
{
    std::vector<std::pair<int, int>> varToState;
    varToState.reserve(vars_.size());

    // Accumulate all messages sent not including the edge to marginalize over
    std::vector<Vector> msgs;
    msgs.reserve(vars_.size());
    for(auto e : edges_)
    {
        msgs.push_back(e->message(MsgDir::to_factor));
        consumedEdges_.push_back(e);
    }

    int edgeIdx = std::distance(edges_.begin(), std::find(edges_.begin(), edges_.end(), edge));
    // Initialize the probability holder
    msgProbs_.resize(varStates_[edgeIdx]);
    msgProbs_.zeros();

    sumRecursiveHelper(0, 0, 1.0, msgs, edgeIdx, type);
    return msgProbs_;
}


void Factor::sumRecursiveHelper(std::size_t idx,
                                int probIdx,
                                double product,
                                const std::vector<Vector>& msgs,
                                std::size_t varIdx,
                                UpdateType type)
{
    // Base case incorporate this state into the marginal computation
    if(idx == vars_.size())
    {
        updateMsgProduct(probIdx, product, varIdx, type);
    }
    // Skip if this is the variable in question
    else if(idx == varIdx)
    {
        sumRecursiveHelper(idx + 1, probIdx, product, msgs, varIdx, type);
    }
    else
    {
        // For each state for this variable, add on the offset into the stateProbs, accumulate the product, and
        // recurse to process the remaining values
        for(int state = 0, num = varStates_[idx]; state < num; ++state)
        {
            sumRecursiveHelper(idx + 1,
                               probIdx + (state * varStateIndex_[vars_[idx]]),
                               product * msgs[idx](state),
                               msgs,
                               varIdx,
                               type);
        }
    }
}


void Factor::updateMsgProduct(int probIdx, double product, std::size_t varIdx, UpdateType type)
{
    for(int state = 0, end = varStates_[varIdx]; state < end; ++state)
    {
        int probIdxIncr = state * varStateIndex_[vars_[varIdx]];
        int factorIdx = probIdx + probIdxIncr;

        if(type == UpdateType::sum)
        {
            msgProbs_(state) += stateProbs_(factorIdx) * product;
        }
        else if(type == UpdateType::max)
        {
            msgProbs_(state) = std::max(stateProbs_(factorIdx) * product, msgProbs_(state));
        }

#ifdef DEBUG_MSGS
        std::cout << "fact idx:" << factorIdx << " prob:" << stateProbs_(factorIdx) << " prod:" << product
            << " state idx:" << state << " msg prob:" << msgProbs_(state) << '\n';
#endif // DEBUG_MSGS
    }
}


int state_index(const std::vector<std::pair<int, int>>& states, const std::unordered_map<int, int>& varToOffset)
{
    int index = 0;
    for(auto st : states)
    {
        index += varToOffset.at(st.first) * st.second;
    }

    return index;
}

} // namespace hssh
} // namespace vulcan
