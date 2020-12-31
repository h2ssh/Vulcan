/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     transition.h
 * \author   Collin Johnson
 *
 * Declaration of TransitionAffordance.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_TRANSITION_H
#define HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_TRANSITION_H

#include "hssh/local_topological/affordance.h"
#include "hssh/local_topological/gateway.h"
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * TransitionAffordance describes the transition from one area to another area. The area transition is from the current
 * area to a different area. The transition is represented as a gateway through which the robot will travel.
 */
class TransitionAffordance : public NavigationAffordance
{
public:
    /**
     * Default constructor for TransitionAffordance.
     */
    TransitionAffordance(void);

    /**
     * Constructor for TransitionAffordance.
     *
     * \param    gateway         Gateway for the transition
     * \param    type            Type of area the transition leads to
     */
    TransitionAffordance(const Gateway& gateway, AreaType type);

    /**
     * gateway retrieves the gateway that provides the affordance.
     */
    Gateway gateway(void) const { return gateway_; }

    /**
     * type retrieves the type of area that the transition leads to.
     */
    AreaType type(void) const { return type_; }

    // NavigationAffordance interface
    virtual void accept(NavigationAffordanceVisitor& visitor) const override;

private:
    Gateway gateway_;
    AreaType type_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<NavigationAffordance>(this), gateway_, type_);
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::TransitionAffordance)

#endif   // HSSH_LOCAL_TOPOLOGICAL_AFFORDANCES_TRANSITION_H
