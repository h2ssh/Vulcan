/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     belief_prop.cpp
* \author   Collin Johnson
*
* Definition of functions for running belief propagation:
*
*   - belief_propagation
*   - loopy_belief_propagation
*/

#include <hssh/local_topological/area_detection/labeling/belief_prop.h>
#include <hssh/local_topological/area_detection/labeling/factor_graph.h>
#include <boost/range/iterator_range.hpp>
#include <cassert>

namespace vulcan
{
namespace hssh
{

int run_to_convergence(FactorGraph::VarIter beginVars,
                       FactorGraph::VarIter endVars,
                       FactorGraph::FactorIter beginFactors,
                       FactorGraph::FactorIter endFactors,
                       UpdateType type,
                       int maxIters = 0);



void belief_propagation(FactorGraph& graph, UpdateType type)
{
    assert(graph.sizeVars() > 0);

    // Use the first variable as the root
    int totalMessages = 0;

    // Initialize with all leaves sending their messages
    for(auto& fact : boost::make_iterator_range(graph.beginFactors(), graph.endFactors()))
    {
        if(fact->isLeaf())
        {
            int numSent = fact->sendMarginals(type, UpdateRule::force);
            assert(numSent == 1);   // only one marginal can be sent by a leaf
            totalMessages += numSent;
        }
    }

    for(auto& var : boost::make_iterator_range(graph.beginVars() + 1, graph.endVars()))
    {
        if(var->isLeaf())
        {
            int numSent = var->sendMarginals(true);
            assert(numSent == 1);   // only one marginal can be sent by a leaf
            totalMessages += numSent;
        }
    }

    // Run a single iteration
    totalMessages += run_to_convergence(graph.beginVars() + 1,
                                        graph.endVars(),
                                        graph.beginFactors(),
                                        graph.endFactors(),
                                        type);

    // Now the root node is ready, so send that message
    assert((*graph.beginVars())->freshCount() == (*graph.beginVars())->degree());
    totalMessages += (*graph.beginVars())->sendMarginals();

    // Run the algorithm to completion
    totalMessages += run_to_convergence(graph.beginVars() + 1,
                                        graph.endVars(),
                                        graph.beginFactors(),
                                        graph.endFactors(),
                                        type);

    std::cout << "Completed belief propagation sum-product using a total of " << totalMessages << " messages.\n";
}


void loopy_belief_propagation(FactorGraph& graph, UpdateType type)
{
    for(auto& e : boost::make_iterator_range(graph.beginEdges(), graph.endEdges()))
    {
        e->setChangeThreshold(1e-4);
        e->setMessagesToUnity();
    }

//     // Initialize with all leaves sending their messages
//     for(auto& fact : boost::make_iterator_range(graph.beginFactors(), graph.endFactors()))
//     {
//         if(fact->isLeaf())
//         {
//             int numSent = fact->sendMarginals(true);
//             assert(numSent == 1);   // only one marginal can be sent by a leaf
//         }
//     }
//
//     for(auto& var : boost::make_iterator_range(graph.beginVars() + 1, graph.endVars()))
//     {
//         if(var->isLeaf())
//         {
//             int numSent = var->sendMarginals(true);
//             assert(numSent == 1);   // only one marginal can be sent by a leaf
//         }
//     }

    int totalMessages = 0;
    totalMessages = run_to_convergence(graph.beginVars(),
                                       graph.endVars(),
                                       graph.beginFactors(),
                                       graph.endFactors(),
                                       type,
                                       20);

    std::cout << "Completed loopy belief propagation sum-product using a total of " << totalMessages << " messages.\n";
}


int run_to_convergence(FactorGraph::VarIter beginVars,
                       FactorGraph::VarIter endVars,
                       FactorGraph::FactorIter beginFactors,
                       FactorGraph::FactorIter endFactors,
                       UpdateType type,
                       int maxIters)
{
    // Iterate until there are no remaining fresh entities
    int iteration = 1;
    int totalMessages = 0;
    int numSent = 1;
    while((numSent > 0) && ((maxIters == 0) || (iteration < maxIters)))
    {
        numSent = 0;

        for(auto& fact : boost::make_iterator_range(beginFactors, endFactors))
        {
            if(fact->freshCount() > 0)
            {
                numSent += fact->sendMarginals(type, UpdateRule::if_needed);
            }
        }

        for(auto& var : boost::make_iterator_range(beginVars, endVars))
        {
            if(var->freshCount() > 0)
            {
                numSent += var->sendMarginals();
            }
        }

        std::cout << "Sent " << numSent << " messages on iteration " << iteration << '\n';
        ++iteration;
        totalMessages += numSent;
    }

    return totalMessages;
}

} // namespace hssh
} // namespace vulcan
