/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_segment.h
 * \author   Collin Johnson
 *
 * Declaration of LocalPathSegment.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_PATH_SEGMENT_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_PATH_SEGMENT_H

#include "hssh/local_topological/affordances/move_along.h"
#include "hssh/local_topological/affordances/transition.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/lambda.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * LocalPathSegment describes a path segment running between two decision points. A path segment is a portion of the
 * environment of approximately constant width that is longer than it is wide, usually substantially so. The path
 * segment is described by the places on either end and the change in the reference frame of those places, called
 * lambda, which is the distance of the path segment and the change in the principal axes of small-scale space.
 *
 * A path segment contains two sets of destinations, those on the left and right side of the segment. Left and right are
 * based on the direction from the entry to the exit. For example, if the exit is north of the entry, then the left
 * destinations are on the west side of the segment and the right destinations are on the east side.
 *
 * The entry of a path segment corresponds to the plus place. The exit is the minus place. Thus, for a normal traversal
 * of a path segment the robot moves in the MINUS direction.
 *
 * Lambda is defined as the transform from the plus area to the minus area.
 */
class LocalPathSegment : public LocalArea
{
public:
    /**
     * Constructor for LocalPathSegment.
     *
     * \param    plus                Affordance for reaching the currently known plus end of the path
     * \param    minus               Affordance for reaching the currently known minus end of the path
     * \param    leftDestinations    Destinations on the left side of the path segment
     * \param    rightDestinations   Destinations on the right side of the path segment
     * \param    lambda              Change in reference frame from the start to the end
     * \param    id                  Unique id for the area
     * \param    extent              Physical extent of the area in the world
     * \param    gateways            Gateways bounding the area from adjacent areas -- end places and destinations
     */
    LocalPathSegment(const TransitionAffordance& plus,
                     const TransitionAffordance& minus,
                     const std::vector<TransitionAffordance>& leftDestinations,
                     const std::vector<TransitionAffordance>& rightDestinations,
                     const Lambda& lambda,
                     int id,
                     const AreaExtent& extent,
                     const std::vector<Gateway>& gateways);

    /**
     * plusTransition retrieves the affordance used for transitioning to the area at the plus end of the path segment,
     * if there is an area there. If there's no area there (dead end or frontier), then the transition doesn't lead
     * anywhere.
     */
    TransitionAffordance plusTransition(void) const { return plusTransition_; }

    /**
     * minusTransition retrieves the affordance used for transitioning to the area at the minus end of the path segment,
     * if there is an area there. If there's no area there (dead end or frontier), then the transition doesn't lead
     * anywhere.
     */
    TransitionAffordance minusTransition(void) const { return minusTransition_; }

    /**
     * moveAlongMinus retrieve the affordance for moving toward the plus end of the path segment.
     */
    MoveAlongAffordance moveAlongPlus(void) const { return plus_; }

    /**
     * moveAlongMinus retrieve the affordance for moving toward the minus end of the path segment.
     */
    MoveAlongAffordance moveAlongMinus(void) const { return minus_; }

    /**
     * leftDestinations retrieves the destinations on the left side of the path segment.
     */
    std::vector<TransitionAffordance> leftDestinations(void) const { return leftDestinations_; }

    /**
     * rightDestinations retrieves the destinations on the right side of the path segment.
     */
    std::vector<TransitionAffordance> rightDestinations(void) const { return rightDestinations_; }

    /**
     * lambda retrieves the lambda value associated with the path segment.
     */
    Lambda lambda(void) const { return lambda_; }

    /**
     * setLambda is used to change the stored lambda value. A lambda might not be initially available upon construction
     * of the LocalPathSegment.
     *
     * \param    lambda      New lambda value to use
     */
    void setLambda(const Lambda& lambda) { lambda_ = lambda; }


    ////////////////////////////// LocalArea interface //////////////////////////

    AreaType type(void) const override { return AreaType::path_segment; }
    std::string description(void) const override;
    void visitAffordances(NavigationAffordanceVisitor& visitor) const override;
    void accept(LocalAreaVisitor& visitor) const override;
    bool isEndpoint(const LocalArea& adj) const override;

private:
    TransitionAffordance plusTransition_;
    TransitionAffordance minusTransition_;
    MoveAlongAffordance plus_;
    MoveAlongAffordance minus_;
    std::vector<TransitionAffordance> leftDestinations_;
    std::vector<TransitionAffordance> rightDestinations_;
    Lambda lambda_;

    // Serialization support
    LocalPathSegment(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LocalArea>(this),
           plusTransition_,
           minusTransition_,
           plus_,
           minus_,
           leftDestinations_,
           rightDestinations_,
           lambda_);
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::LocalPathSegment)

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREAS_PATH_SEGMENT_H
