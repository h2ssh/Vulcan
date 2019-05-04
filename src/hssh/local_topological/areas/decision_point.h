/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_point.h
* \author   Collin Johnson
*
* Declaration of LocalDecisionPoint.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_DECISION_POINT_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_DECISION_POINT_H

#include <hssh/local_topological/areas/place.h>
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh 
{
    
/**
* LocalDecisionPoint describes a decision point in small-scale space. A decision point occurs at the intersection of two
* or more path segments or at a dead end at the end of a path segment. The local topology of the decision point is described
* by a small-scale star, which is described elsewhere. A decision point can also have a number of destinations connected to
* it, especially in the case of a large, open area with offices and classrooms splitting off from it.
*/
class LocalDecisionPoint : public LocalPlace
{
public:
    
    /**
    * Constructor for LocalDecisionPoint.
    *
    * \param    star            Small-scale star describing the topology of the place
    * \param    map             Local metric map of the decision point
    * \param    id              Unique id assigned to the decision point
    * \param    extent          Physical extent of the area in the world
    * \param    gateways        Gateways bounding the area from adjacent areas -- destinations and path segments
    */
    LocalDecisionPoint(const SmallScaleStar&       star,
                       const LocalPerceptualMap&   map,
                       int                         id,
                       const AreaExtent&           extent,
                       const std::vector<Gateway>& gateways);
    
    // Right turn affordances (entry frag)
    // Left turn affordances  (entry frag)
    // Drive straight affordance (entry frag)
    // Turn back affordance (entry frag)

    ////////////////////////////// LocalArea interface //////////////////////////
    AreaType type(void) const { return AreaType::decision_point; }
    std::string description     (void) const override;
    void        accept          (LocalAreaVisitor& visitor) const override;
    
private:
    
    // Serialization support
    LocalDecisionPoint(void) { }
    
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (cereal::base_class<LocalPlace>(this));
    }
};
    
}
}

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::LocalDecisionPoint)

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_DECISION_POINT_H
