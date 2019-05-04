/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration.h
* \author   Collin Johnson
*
* Declaration of ExplorationAffordance.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_EXPLORATION_H
#define HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_EXPLORATION_H

#include <hssh/local_topological/affordance.h>
#include <hssh/local_topological/frontier.h>
#include <cereal/types/base_class.hpp>

namespace vulcan 
{
namespace hssh 
{
    
/**
* ExplorationAffordance describes a frontier in the the current area and how the robot can reach it. A frontier is an unexplored region,
* so this affordance provides the robot with a means of completely exploring its current area by traveling to each of frontiers
* encapsulated in the exploration affordance.
*/
class ExplorationAffordance : public NavigationAffordance
{
public:
    
    /**
    * Default constructor for ExplorationAffordance.
    */
    ExplorationAffordance(void);
    
    /**
    * Constructor for ExplorationAffordance.
    *
    * \param    frontier            Frontier the robot can explore
    */
    ExplorationAffordance(const Frontier& frontier);
    
    /**
    * frontier retrieves the frontier associated with the affordance.
    */
    Frontier frontier(void) const { return frontier_; }

    // NavigationAffordance interface
    virtual void accept(NavigationAffordanceVisitor& visitor) const override;
    
private:
    
    Frontier frontier_;

    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (cereal::base_class<NavigationAffordance>(this),
            frontier_);
    }
};

}
}

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::ExplorationAffordance)

#endif // HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_EXPLORATION_H
