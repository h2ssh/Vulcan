/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_analysis.h
* \author   Collin Johnson
*
* Declaration of VisibilityAnalysis.
*/

#ifndef MPEPC_GRIDS_VISIBILITY_ANALYSIS_H
#define MPEPC_GRIDS_VISIBILITY_ANALYSIS_H

#include <core/point_util.h>
#include <core/pose.h>
#include <math/geometry/polygon.h>
#include <system/message_traits.h>
#include <utils/ray_tracing.h>

#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }
namespace tracker { class DynamicObjectCollection; }
namespace mpepc
{

/**
* laser_configuration_t store the configuration of a particular laser.
*/
struct laser_configuration_t
{
    pose_t offset;               ///< Offset of the laser in the robot frame
    utils::ray_trace_range_t range;     ///< Range of angles to scan in the laser frame
};

/**
* VisibilityAnalysis computes the visible portion of the environment from a robot pose with a laser in some
* known configuration relative to the robot. Two forms of visibility analysis are performed. First, we consider the
* visible portion of the static environment, ignoring any dynamic objects that may be around the robot. Second, we
* consider how the dynamic objects affect the visibility of the environment.
*/
class VisibilityAnalysis
{
public:

    using const_iterator = PointVec<double>::const_iterator;

    /**
    * Default constructor for VisibilityAnalysis.
    *
    * Create visibility at the origin with no rays.
    */
    VisibilityAnalysis(void) = default;

    /**
    * Constructor for VisibilityAnalysis.
    *
    * Analyze the scene around the robot for a particular laser.
    *
    * \param    pose            Pose of the robot
    * \param    lasers          Lasers for which to simulate the view
    * \param    map             Map in which to compute the visibility
    * \param    objects         Dynamic objects around the robot
    */
    VisibilityAnalysis(const pose_t& pose,
                       const std::vector<laser_configuration_t>& lasers,
                       const hssh::LocalPerceptualMap& map,
                       const tracker::DynamicObjectCollection& objects);

    /**
    * Constructor for VisibilityAnalysis.
    *
    * Analyze the scene around the robot for a particular laser.
    *
    * \param    pose            Pose of the robot
    * \param    laser           Laser for which to simulate the view
    * \param    map             Map in which to compute the visibility
    * \param    objects         Dynamic objects around the robot
    */
    VisibilityAnalysis(const pose_t& pose,
                       const laser_configuration_t& laser,
                       const hssh::LocalPerceptualMap& map,
                       const tracker::DynamicObjectCollection& objects);

    /**
    * origin retrieves the origin of the scan.
    */
    Point<double> origin(void) const { return origin_; }

    /**
    * staticView retrieves the viewing polygon in the map when ignoring all dynamic objects.
    */
    const math::Polygon<double>& staticView(void) const { return staticView_; }

    /**
    * dynamicView retrieves the viewing polygon in the map when considering collisions with
    * dynamic objects.
    */
    const math::Polygon<double>& dynamicView(void) const { return dynamicView_; }

    // Iterate over the rays used to determine visibility
    std::size_t sizeRays(void) const { return staticView_.size(); }
    const_iterator beginStatic(void) const { return staticView_.begin(); }
    const_iterator endStatic(void) const { return staticView_.end(); }
    const_iterator beginDynamic(void) const { return dynamicView_.begin(); }
    const_iterator endDynamic(void) const { return dynamicView_.end(); }

private:

    Point<double> origin_;
    math::Polygon<double> staticView_;
    math::Polygon<double> dynamicView_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( origin_,
            staticView_,
            dynamicView_);
    }
};

} // namespace mpepc
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(mpepc::VisibilityAnalysis, ("DEBUG_MPEPC_VISIBILITY_ANALYSIS"))

#endif // MPEPC_GRIDS_VISIBILITY_ANALYSIS_H
