/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     shrink_and_stretch.h
 * \author   Collin Johnson
 *
 * Declaration of ShrinkAndStretchBoundary implementation of SmallScaleSpaceBoundary interface.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_BOUNDARY_SHRINK_AND_STRETCH_H
#define HSSH_LOCAL_TOPOLOGICAL_BOUNDARY_SHRINK_AND_STRETCH_H

#include "hssh/local_topological/event_visitor.h"
#include "hssh/local_topological/small_scale_space_boundary.h"

namespace vulcan
{
namespace hssh
{

const std::string kShrinkAndStretchBoundaryType("shrink_and_stretch");

/**
 * ShrinkAndStretchBoundary is a boundary that maintains the small-scale space in the LPM as defined in the
 * LocalMetrical section of the H2SSH description. The boundary as defined there contains:
 *
 *   - the extent of the current area
 *   - the extent of all areas adjacent to the current area
 *   - everything visible from the robot's current pose
 *
 * The first two conditions are easy to maintain within the Local Topological layer. Whenever a transition occurs, then
 * a new boundary is computed based on the areas within the LocalTopMap. The final condition will be maintained by
 * having the Local Metrical layer in the stretching mode, so as the robot sees more of the environment, the LPM will
 * stretch and grow along with it.
 */
class ShrinkAndStretchBoundary
: public SmallScaleSpaceBoundary
, public LocalAreaEventVisitor
{
public:
    // SmallScaleSpaceBoundary interface
    boost::optional<MapBoundary> computeBoundary(const LocalAreaEventVec& events,
                                                 const LocalTopoMap& topoMap,
                                                 const LocalPose& pose,
                                                 const LocalPerceptualMap& lpm) override;

    // LocalAreaEventVisitor interface
    void visitAreaTransition(const AreaTransitionEvent& event) override;
    void visitTurnAround(const TurnAroundEvent& event) override;

private:
    const LocalTopoMap* currentMap_;
    const LocalPerceptualMap* currentLpm_;
    Point<float> currentPosition_;
    boost::optional<MapBoundary> updatedBoundary_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_BOUNDARY_SHRINK_AND_STRETCH_H
