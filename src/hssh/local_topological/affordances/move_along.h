/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     move_along.h
* \author   Collin Johnson
*
* Declaration of MoveAlongAffordance.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_MOVE_ALONG_H
#define HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_MOVE_ALONG_H

#include <hssh/local_topological/affordance.h>
#include <hssh/types.h>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
* MoveAlongAffordance represents the possible motion along a path. The affordance is created by considering the two
* endpoints of the path. The direction determines which endpoint the affordance is associated with, along with the
* target heading.
*/
class MoveAlongAffordance : public NavigationAffordance
{
public:
    
    /**
    * Default constructor for MoveAlongAffordance.
    */
    MoveAlongAffordance(void);

    /**
    * Constructor for MoveAlongAffordance.
    *
    * \param    plusEndpoint        Current known endpoint of the plus end of the path
    * \param    minusEndpoint       Current known endpoint of the minus end of the path
    * \param    direction           Direction of motion this affordance represents
    */
    MoveAlongAffordance(const Point<float>& plusEndpoint,
                        const Point<float>& minusEndpoint,
                        TopoDirection             direction);

    /**
    * direction retrieves the direction of motion along the path that this affordance represents. The robot is either
    * driving toward the plus or minus end.
    */
    TopoDirection direction(void) const { return direction_; }

    // NavigationAffordance interface
    virtual void accept(NavigationAffordanceVisitor& visitor) const override;

private:
    
    TopoDirection direction_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (cereal::base_class<NavigationAffordance>(this),
            direction_);
    }
};

}
}

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::MoveAlongAffordance)

#endif // HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_MOVE_ALONG_H
