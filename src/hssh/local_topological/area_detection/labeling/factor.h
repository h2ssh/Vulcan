/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor.h
* \author   Collin Johnson
*
* Declaration of Factor.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_H

#include <hssh/local_topological/area_detection/labeling/factor_edge.h>
#include <hssh/local_topological/area_detection/labeling/variable.h>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vulcan
{
namespace hssh
{

using VarToState = std::vector<std::pair<int, int>>;

/**
* UpdateRule defines the types of factor updates that can occur:
*
*   - force : forces the update to be performed, no matter what
*   - if_needed : only run the update if there are fresh messages.
*/
enum class UpdateRule
{
    force,
    if_needed,
};

/**
* UpdateType defines the possible types of factor update to perform:
*
*   - sum : compute the marginal probability for the factor
*   - max : compute the max probability for the factor
*/
enum class UpdateType
{
    sum,
    max,
};

/**
* FactorStateProb holds the probabilities associated with a particular state of all the variables associated with the
* factor. The states are the integer states that each of the variables takes on. They are specified as varId->state
* pairs. The probability of the configuration is then specified.
*/
struct FactorStateProb
{
    VarToState states;
    double prob;
};

/**
* Factor represents a factor in the factor graph.
*/
class Factor
{
public:

    using Ptr = std::shared_ptr<Factor>;

    /**
    * Constructor for Factor.
    *
    * \param    id          Id of the factor
    * \param    variables   Variables dependent on this factor
    * \param    probs       Probabilities of each combination of states (size = product of variables.numStates())
    */
    Factor(int id, const std::vector<Variable*>& variables, const std::vector<FactorStateProb>& probs);

    /**
    * id retrieves the id of the variable.
    */
    int id(void) const { return id_; }

    /**
    * degree checks how many factors are attached to this edge.
    */
    int degree(void) const { return edges_.size(); }

    /**
    * isLeaf checks if this factor is a leaf in the tree. A leaf has degree 1.
    */
    bool isLeaf(void) const { return degree() == 1; }

    /**
    * isComplete checks if a message has passed along every incoming edge so the marginal can be computed.
    */
    bool isComplete(void) const;

    /**
    * addEdge adds an edge between this variable and a factor. The pointer to the edge is stored, so it needs to stay
    * alive beyond the lifetime of the Variable.
    *
    * \param    edge            Edge to be added
    * \pre  edge != nullptr
    * \pre  edge->factorId() == id()
    * \pre  edge->varId() in variables
    */
    void addEdge(FactorEdge* edge);

    /**
    * freshCount counts the number of fresh messages available at this Variable.
    */
    int freshCount(void) const;

    /**
    * sendMarginals sends marginals messages to all nodes whose computation will include at least one fresh message.
    *
    * \param    type            Type of update to perform -- sum or max
    * \param    rule            Rule to use when doing the update -- force or if needed
    * \return   Number of messages sent.
    */
    int sendMarginals(UpdateType type, UpdateRule rule);

private:

    int id_;
    std::vector<int> vars_;                 ///< Ids of the stored variables
    std::vector<int> varStates_;            ///< Number of states stored for the variables
    std::vector<FactorEdge*> edges_;
    std::vector<FactorEdge*> consumedEdges_;        ///< Edges consumed on current iteration
    std::unordered_map<int, int> varStateIndex_;    ///< Index increment for the states to quickly identify the correct index for a state (varid->vars idx)
    Vector stateProbs_;           ///< Probabilities for this factor associated with each variable state
    Vector msgProbs_;             ///< Probabliities coming in from messages


    Vector computeEdgeMarginal(FactorEdge* edge, UpdateType type);
    void sumRecursiveHelper(std::size_t idx,
                            int probIdx,
                            double product,
                            const std::vector<Vector>& msgs,
                            std::size_t varIdx,
                            UpdateType type
                           );
    void updateMsgProduct(int probIdx, double product, std::size_t varIdx, UpdateType type);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_H
