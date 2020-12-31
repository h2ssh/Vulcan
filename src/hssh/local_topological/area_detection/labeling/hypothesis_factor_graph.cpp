/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_factor_graph.cpp
* \author   Collin Johnson
*
* Definition of convert_hypothesis_graph_to_factor_graph.
*/

#include "hssh/local_topological/area_detection/labeling/hypothesis_factor_graph.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/boundary_classifier.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_classifier.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace hssh
{

using VarVec = std::vector<Variable::Ptr>;
using FactVec = std::vector<Factor::Ptr>;
using EdgeVec = std::vector<FactorEdge::Ptr>;

struct ConstructionState
{
    int nextId = 0;
    VarVec vars;
    FactVec factors;
    EdgeVec edges;
    std::unordered_map<int, AreaHypothesisBoundary*> varToBoundary;
    std::unordered_map<int, AreaHypothesis*> varToHyp;
    std::unordered_map<AreaHypothesis*, Variable*> hypToVar;
    std::unordered_map<AreaHypothesisBoundary*, Variable*> bndToVar;
};


void add_area_variable(AreaHypothesis* hyp,
                       const HypothesisClassifier& classifier,
                       ConstructionState& state);
void add_gateway_variable(AreaHypothesisBoundary* boundary, ConstructionState& state);
void add_boundary_factor(AreaHypothesisBoundary* boundary,
                         const BoundaryClassifier& classifier,
                         ConstructionState& state);


HypothesisFactorGraph convert_hypothesis_graph_to_factor_graph(HypothesisGraph& hypGraph,
                                                               const HypothesisClassifier& hypClassifier,
                                                               const BoundaryClassifier& bndClassifier)
{
    ConstructionState state;

    // Create a factor for the type distribution for each area
    for(AreaHypothesis* hyp : boost::make_iterator_range(hypGraph.beginHypothesis(), hypGraph.endHypothesis()))
    {
        add_area_variable(hyp, hypClassifier, state);
    }

    // Create a factor for the gateway probability for each boundary
    for(AreaHypothesisBoundary* bnd : boost::make_iterator_range(hypGraph.beginBoundary(), hypGraph.endBoundary()))
    {
        add_gateway_variable(bnd, state);
    }

    // Create a factor for the type distribution for each boundary and its bounding areas
    for(AreaHypothesisBoundary* bnd : boost::make_iterator_range(hypGraph.beginBoundary(), hypGraph.endBoundary()))
    {
        add_boundary_factor(bnd, bndClassifier, state);
    }

    HypothesisFactorGraph hypFactorGraph;
    hypFactorGraph.graph = FactorGraph(state.vars, state.factors, state.edges);
    hypFactorGraph.varToHyp = state.varToHyp;
    hypFactorGraph.varToBoundary = state.varToBoundary;

    return hypFactorGraph;
}


void add_area_variable(AreaHypothesis* hyp,
                       const HypothesisClassifier& classifier,
                       ConstructionState& state)
{
    int id = state.nextId++;
    state.varToHyp[id] = hyp;

    auto hypVar = std::make_shared<Variable>(id, 3);
    std::vector<Variable*> hypVars;
    hypVars.push_back(hypVar.get());

    auto dist = classifier.calculateDistribution(hyp->features());

    std::vector<FactorStateProb> hypProbs;
    FactorStateProb pathProb;
    pathProb.states.emplace_back(id, kPathIdx);
    pathProb.prob = dist.path;
    hypProbs.push_back(pathProb);

    FactorStateProb decisionProb;
    decisionProb.states.emplace_back(id, kDecisionIdx);
    decisionProb.prob = dist.decision;
    hypProbs.push_back(decisionProb);

    FactorStateProb destProb;
    destProb.states.emplace_back(id, kDestIdx);
    destProb.prob = dist.destination;
    hypProbs.push_back(destProb);

    auto hypFactor = std::make_shared<Factor>(state.nextId++, hypVars, hypProbs);
    auto hypEdge = std::make_shared<FactorEdge>(hypVar->id(), hypFactor->id(), 3);
    hypVar->addEdge(hypEdge.get());
    hypFactor->addEdge(hypEdge.get());

    state.hypToVar[hyp] = hypVar.get();
    state.vars.push_back(hypVar);
    state.factors.push_back(hypFactor);
    state.edges.push_back(hypEdge);
}


void add_gateway_variable(AreaHypothesisBoundary* boundary, ConstructionState& state)
{
    int id = state.nextId++;
    state.varToBoundary[id] = boundary;

    auto bndVar = std::make_shared<Variable>(id, 2);
    std::vector<Variable*> bndVars;
    bndVars.push_back(bndVar.get());

    std::vector<FactorStateProb> bndProbs;
    FactorStateProb onProb;
    onProb.states.emplace_back(id, 1);
    onProb.prob = boundary->getGateway().probability();
    bndProbs.push_back(onProb);

    FactorStateProb offProb;
    offProb.states.emplace_back(id, 0);
    offProb.prob = 1.0 - onProb.prob;
    bndProbs.push_back(offProb);

    auto bndFactor = std::make_shared<Factor>(state.nextId++, bndVars, bndProbs);
    auto bndEdge = std::make_shared<FactorEdge>(bndVar->id(), bndFactor->id(), 2);
    bndVar->addEdge(bndEdge.get());
    bndFactor->addEdge(bndEdge.get());

    state.bndToVar[boundary] = bndVar.get();
    state.vars.push_back(bndVar);
    state.factors.push_back(bndFactor);
    state.edges.push_back(bndEdge);
}


void add_boundary_factor(AreaHypothesisBoundary* boundary,
                         const BoundaryClassifier& classifier,
                         ConstructionState& state)
{
    auto bndVar = state.bndToVar[boundary];

    std::array<Variable*, 2> hypVars;
    hypVars[0] = state.hypToVar[boundary->getHypotheses()[0]];
    hypVars[1] = state.hypToVar[boundary->getHypotheses()[1]];

    std::vector<Variable*> allVars;
    allVars.push_back(bndVar);
    allVars.push_back(hypVars[0]);
    allVars.push_back(hypVars[1]);

    std::vector<FactorStateProb> bndProbs;
    auto dist = classifier.boundaryDistribution();

    std::array<AreaHypothesis*, 2> hyps;
    hyps[0] = boundary->getHypotheses()[0];
    hyps[1] = boundary->getHypotheses()[1];

    std::array<int, 2> pathIdx;
    for(int n = 0; n < 2; ++n)
    {
        pathIdx[n] = hyps[n]->isEndGateway(boundary->getGateway().id()) ? kPathEndIdx : kPathIdx;
    }

    // Create a state probability for each combination of path/dest/decision for the areas and on/off for boundary
    for(int i = 0; i < 3; ++i)
    {
        int idxI = i == kPathIdx ? pathIdx[0] : i;

        for(int j = 0; j < 3; ++j)
        {
            int idxJ = j == kPathIdx ? pathIdx[1] : j;

            FactorStateProb onProb;
            onProb.states.emplace_back(hypVars[0]->id(), i);
            onProb.states.emplace_back(hypVars[1]->id(), j);
            onProb.states.emplace_back(bndVar->id(), 1);
            onProb.prob = dist.posModel(idxI, idxJ);
            bndProbs.push_back(onProb);

            FactorStateProb offProb;
            offProb.states.emplace_back(hypVars[0]->id(), i);
            offProb.states.emplace_back(hypVars[1]->id(), j);
            offProb.states.emplace_back(bndVar->id(), 0);
            offProb.prob = dist.negModel(idxI, idxJ);
            bndProbs.push_back(offProb);
        }
    }

    auto bndFactor = std::make_shared<Factor>(state.nextId++, allVars, bndProbs);
    auto edge1 = std::make_shared<FactorEdge>(hypVars[0]->id(), bndFactor->id(), 3);
    auto edge2 = std::make_shared<FactorEdge>(hypVars[1]->id(), bndFactor->id(), 3);
    auto edge3 = std::make_shared<FactorEdge>(bndVar->id(), bndFactor->id(), 2);

    bndFactor->addEdge(edge1.get());
    bndFactor->addEdge(edge2.get());
    bndFactor->addEdge(edge3.get());
    hypVars[0]->addEdge(edge1.get());
    hypVars[1]->addEdge(edge2.get());
    bndVar->addEdge(edge3.get());

    state.factors.push_back(bndFactor);
    state.edges.push_back(edge1);
    state.edges.push_back(edge2);
    state.edges.push_back(edge3);
}

} // namespace hssh
} // namespace vulcan
