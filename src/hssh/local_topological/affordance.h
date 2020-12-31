/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     affordances.h
* \author   Collin Johnson
*
* Declaration of NavigationAffordance base class.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AFFORDANCE_H
#define HSSH_LOCAL_TOPOLOGICAL_AFFORDANCE_H

#include "core/pose.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{
    
class NavigationAffordanceVisitor;

/**
* NavigationAffordance describes a navigation affordance in small-scale space. A navigation affordance provides the
* robot with a distinctive location in its local surround to travel to, like a gateway, frontier, or otherwise 
* remembered special location.
*/
class NavigationAffordance
{
public:
    
    /**
    * Constructor for NavigationAffordance.
    *
    * \param    targetPose          Pose of the afforance motion target
    */
    NavigationAffordance(const pose_t& targetPose)
    : target_(targetPose)
    {
    }
    
    /**
    * Destructor for NavigationAffordance.
    */
    virtual ~NavigationAffordance(void) { }
    
    /**
    * target retrieves the target pose indication the location of the affordance.
    */
    pose_t target(void) const { return target_; }
    
    /**
    * accept should be called by a visitor per the Visitor pattern.
    */
    virtual void accept(NavigationAffordanceVisitor& visitor) const = 0;

protected:
    
    // For serialization support, allow subclasses to default construct
    NavigationAffordance(void) { }
    
private:
    
    pose_t target_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (target_);
    }
};
    
}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AFFORDANCE_H
