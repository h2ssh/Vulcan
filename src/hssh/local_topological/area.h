/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_H

#include "hssh/local_topological/affordance.h"
#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/gateway.h"
#include "math/coordinates.h"
#include "core/point.h"
#include "utils/pose_trace.h"
#include <boost/optional.hpp>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <iosfwd>
#include <memory>

namespace vulcan
{
namespace utils { class PoseTrace; }
namespace hssh
{

class LocalAreaVisitor;

/**
* gateway_crossing_t contains the gateway that was possibly crossed along with the time it was crossed.
*/
struct gateway_crossing_t
{
    boost::optional<Gateway> gateway;
    int64_t                  timestamp;
};


/**
* LocalArea is the abstract base class of the LocalArea hierarchy for describing small-scale space. Each local area
* contains at least the following:
*
*   - unique id to use for identifying a specific area
*   - a classification to identify the type of area and allow for casting down the hierarchy
*   - a description of the type of area, a combination of classification + id
*   - a rectangular boundary that approximates the area in the current LPM
*   - a polygon boundary that traces more exactly the boundary of the area, not necessarily a convex polygon
*/
class LocalArea
{
public:

    using Id = int;
    using Ptr = std::shared_ptr<LocalArea>;

    /**
    * Constructor for LocalArea.
    *
    * \param    id              Unique id for the area
    * \param    extent          Physical extent of the area in the world
    * \param    gateways        Gateways bounding the area from adjacent areas
    */
    LocalArea(Id id, const AreaExtent& extent, const std::vector<Gateway>& gateways);

    // Accessors

    /**
    * id retrieves the unique identifier for the LocalArea.
    */
    Id id(void) const { return id_; }

    /**
    * center retrieves the center of the area in the current global reference frame.
    */
    pose_t center(void) const { return extent_.center(); }

    /**
    * extent retrieves the full AreaExtent structure.
    */
    const AreaExtent& extent(void) const { return extent_; }

    /**
    * gateways retrieves the gateways bounding the area.
    */
    const std::vector<Gateway>& gateways(void) const { return gateways_; }

    /**
    * boundary retrieves the rectangular boundary of the area.
    */
    math::Rectangle<double> boundary(math::ReferenceFrame frame = math::ReferenceFrame::GLOBAL) const;

    /**
    * polygonBoundary retrieves the polygon boundary of the area. The polygon boundary is relative to the center
    * of the place.
    */
    math::Polygon<double> polygonBoundary(math::ReferenceFrame frame = math::ReferenceFrame::GLOBAL) const;

    /**
    * setOrientation sets the orientation of the local area. It doesn't rotate anything, just indicates what sort of
    * rotation exists globally in the area.
    */
    void setOrientation(double orientation) { extent_.setOrientation(orientation); }

    /**
    * contains checks to see if the area contains a particular point.
    */
    bool contains(const Point<float>& point,
                  math::ReferenceFrame      frame = math::ReferenceFrame::GLOBAL) const;

    /**
    * containsCell checks at the occupancy-grid-cell-level to see if the area contains a particular point. This method
    * takes more time, but is more precise than contains, which just looks at the convex hull. containsCell should
    * only be called after a call to contains returns true.
    *
    * \param    position        Point to check for containment in the global frame
    */
    bool containsCell(const Point<float>& position) const;

    /**
    * hasGateway checks to see if the area is bounded by the provided gateway.
    *
    * \param    gateway         Gateway to check
    * \return   True if the gateway is one of the gateways that bounds the area.
    */
    bool hasGateway(const Gateway& gateway) const;

    /**
    * findTransitionGateway locates the gateway the robot transitioned through using a pose in the area and a pose not
    * in the area. The gateway selected will be the gateway found in which the poses exist on opposite sides of
    * the gateway and a line connecting the poses intersects the gateway boundary.
    *
    * If no gateway with that condition is found, then the flag will be set to false in the return value.
    *
    * The poses must be in the GLOBAL reference frame. The returned gateway is in the GLOBAL reference frame.
    *
    * \param    poseInArea          Pose in the area
    * \param    poseOutOfArea       Pose not in the area
    * \return   An instance of gateway_crossing_t describing the crossing, if one occurred.
    */
    gateway_crossing_t findTransitionGateway(const pose_t& poseInArea, const pose_t& poseOutOfArea) const;

    ///////////// Interface for subclasses ////////////////////

    /**
    * type retrieves the type enum for the given LocalArea.
    */
    virtual AreaType type(void) const = 0;

    /**
    * description provides a string description of the area.
    */
    virtual std::string description(void) const = 0;

    /**
    * visitAffordances provides accepts the visitor for each of the affordances contained within the area.
    */
    virtual void visitAffordances(NavigationAffordanceVisitor& visitor) const = 0;

    /**
    * accept method for the LocalAreaVisitor interface.
    */
    virtual void accept(LocalAreaVisitor& visitor) const = 0;

    /**
    * isEndpoint checks if the area is adjacent to an endpoint of this area.
    *
    * \param    area        Adajcent area
    * \return   True if the adjacent area is an endpoint of this area.
    */
    virtual bool isEndpoint(const LocalArea& area) const = 0;

protected:

    // Allow default construction for subclasses to help with serialization
    LocalArea(void)
    : id_(-1)
    {
    }

private:

    Id                   id_;
    AreaExtent           extent_;
    std::vector<Gateway> gateways_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar (id_,
            extent_,
            gateways_);
    }
};

// Operators for LocalArea
/**
 * operator== for LocalArea. Two LocalAreas are considered equal if:
 *
 *   - They have overlapping boundaries > 0.75 the area of each of the individual boundaries
 *   - They have at least two similar gateways, or their only gateway is similar
 */
bool operator==(const LocalArea& lhs, const LocalArea& rhs);

} // namespace hssh
} // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::LocalArea)

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_H
