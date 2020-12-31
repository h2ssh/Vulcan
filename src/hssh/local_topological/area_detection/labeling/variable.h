/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     variable.h
* \author   Collin Johnson
*
* Declaration of Variable.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_VARIABLE_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_VARIABLE_H

#include "hssh/local_topological/area_detection/labeling/factor_edge.h"
#include <memory>
#include <vector>

namespace vulcan 
{
namespace hssh 
{

/**
* Variable represents a variable in the factor graph.
*/
class Variable
{
public:
    
    using Ptr = std::shared_ptr<Variable>;
    
    /**
    * Constructor for Variable.
    * 
    * \param    id          Id for the variable
    * \param    numStates   Number of states associated with the variable
    * \pre  numStates > 0
    */
    Variable(int id, int numStates);

    /**
    * id retrieves the id of the variable.
    */
    int id(void) const { return id_; }
    
    /**
    * numStates retrieves the number of states for the variable.
    */
    int numStates(void) const { return state_.n_elem; }
    
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
    * \pre  edge->varId() == id()
    */
    void addEdge(FactorEdge* edge);
    
    /**
    * freshCount counts the number of fresh messages available at this Variable.
    */
    int freshCount(void) const;
    
    /**
    * sendMarginals sends marginals messages to all nodes whose computation will include at least one fresh message.
    * 
    * \param    force       Force sending even if there are no fresh messages
    * \return   Number of messages sent.
    */
    int sendMarginals(bool force = false);
    
    /**
    * marginal retrieves the marginal probability of each state for the variable.
    * 
    * \pre  isComplete() == true
    * \return   Marginal probability across all states.
    */
    Vector marginal(void);
    
private:
    
    int id_;
    std::vector<FactorEdge*> edges_;
    std::vector<FactorEdge*> consumedEdges_;
    Vector state_;        ///< Place to hold all state-based computations
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_VARIABLE_H
