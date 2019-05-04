/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     place.h
* \author   Collin Johnson
* 
* Declaration of LocalPlace abstract base class.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_PLACE_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_PLACE_H

#include <hssh/local_topological/area.h>
#include <hssh/local_topological/small_scale_star.h>
#include <hssh/local_topological/affordances/transition.h>
#include <hssh/local_metric/lpm.h>
#include <boost/optional.hpp>
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh 
{
    
/**
* LocalPlace describes a place in small-scale space. Places generally occur at the intersection of
* paths, offices, or other non-place areas. A place adds a metric map to the description of the area.
*/
class LocalPlace : public LocalArea
{
public:
    
    /**
    * Constructor for LocalPlace.
    *
    * \param    star            Symbolic description of the transitions for the area
    * \param    map             Metric map of the place
    * \param    id              Unique id for the area
    * \param    extent          Physical extent of the area in the world
    * \param    gateways        Gateways in the place
    */
    LocalPlace(const SmallScaleStar&       star,
               const LocalPerceptualMap&   map,
               int                         id,
               const AreaExtent&           extent,
               const std::vector<Gateway>& gateways);
    
    /**
    * star retrieves the small-scale star description of the local topology.
    */
    const SmallScaleStar& star(void) const { return star_; }
    
    /**
    * map retrieves the map of the place.
    */
    const LocalPerceptualMap& map(void) const { return map_; }
    
    /**
    * adjacent retrieves the transitions leading to areas adjacent to this place.
    */
    const std::vector<TransitionAffordance> adjacent(void) const { return adjacent_; }
    
    /**
    * affordanceForFragment finds the affordance associated with the provided path fragment in the small-scale star.
    * If no such fragment exists, then there won't be an affordance.
    *
    * \param    fragment            Path fragment for which to find the affordance
    * \return   The affordance associated with the path fragment, if one exists.
    */
    boost::optional<TransitionAffordance> affordanceForFragment(const local_path_fragment_t& fragment) const;

    /**
    * findGatewayFragment finds the path fragment associated with the gateway through which the area was entered.
    * 
    * \param    gateway         Gateway through which the area was entered
    * \return   Entry fragment if a match is found for the gateway.
    */
    boost::optional<local_path_fragment_t> findGatewayFragment(const Gateway& gateway) const;
    
    // LocalArea interface
    void visitAffordances(NavigationAffordanceVisitor& visitor) const override;
    bool isEndpoint(const LocalArea& adj) const override { return false; }  // there are no endpoints for places
    
protected:
    
    // Default constructor for use in serialization of subclasses
    LocalPlace(void) { }
    
private:
    
    SmallScaleStar star_;
    LocalPerceptualMap map_;
    std::vector<TransitionAffordance> adjacent_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (cereal::base_class<LocalArea>(this),
            star_,
            map_,
            adjacent_);
    }
};
    
} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_PLACE_H
