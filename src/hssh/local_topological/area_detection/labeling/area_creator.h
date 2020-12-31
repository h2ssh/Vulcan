/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_builder.h
* \author   Collin Johnson
*
* Declaration of AreaBuilder.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_BUILDER_H

#include "hssh/local_topological/params.h"
#include "utils/hough_transform.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace math { template <typename T> class Rectangle; }
namespace hssh
{

class AreaProposal;
class LocalArea;
class LocalPerceptualMap;
class SmallScaleStarBuilder;
class VoronoiIsovistField;
class VoronoiSkeletonGrid;
class AreaExtent;


/**
* AreaBuilder is responsible for taking an AreaProposal and turning it into a LocalArea.
*/
class AreaBuilder
{
public:

    /**
    * Constructor for AreaBuilder.
    *
    * \param    starBuilder     Strategy to use for building the small-scale star for decision points
    */
    AreaBuilder(std::shared_ptr<SmallScaleStarBuilder> starBuilder);

    /**
    * buildArea accepts a an AreaProposal and converts it into a LocalArea.
    * The local metric map is extracted for each LocalPlace, while the destinations
    * along a path segment or at a decision point are associated with their parent area.
    *
    * The id for the area is automatically generated.
    *
    * \param    proposal           Proposal to be converted into an area
    * \param    lpm                 LPM from which to extract the local metric maps
    * \param    skeleton            Grid containing the Voronoi skeleton
    * \return   A LocalArea instance based on the information in the proposal.
    */
    std::unique_ptr<LocalArea> buildArea(const AreaProposal&        proposal,
                                         const LocalPerceptualMap&  lpm,
                                         const VoronoiSkeletonGrid& skeleton,
                                         const VoronoiIsovistField& isovists);

    /**
    * buildArea accepts a an AreaProposal and converts it into a LocalArea.
    * The local metric map is extracted for each LocalPlace, while the destinations
    * along a path segment or at a decision point are associated with their parent area.
    *
    * This version should be called if the id for the area is known and only a new model of
    * the area is being created.
    *
    * \param    id                  Id to assign to the area
    * \param    proposal            Proposal to be converted into an area
    * \param    lpm                 LPM from which to extract the local metric maps
    * \param    skeleton            Grid containing the Voronoi skeleton
    * \return   A LocalArea instance based on the information in the proposal.
    */
    std::unique_ptr<LocalArea> buildArea(int                        id,
                                         const AreaProposal&        proposal,
                                         const LocalPerceptualMap&  lpm,
                                         const VoronoiSkeletonGrid& skeleton,
                                         const VoronoiIsovistField& isovists);

private:

    struct proposal_info_t
    {
        int id;

        const AreaProposal&        proposal;
        const LocalPerceptualMap&  lpm;
        const VoronoiSkeletonGrid& grid;
        const VoronoiIsovistField& isovists;
    };

    int   nextAreaId;
    float placeBoundaryRadius;

    std::shared_ptr<SmallScaleStarBuilder> starBuilder;
    utils::HoughTransform hough_;


    std::unique_ptr<LocalArea> buildAreaFromProposal(const proposal_info_t& proposal);
    std::unique_ptr<LocalArea> createDecisionPoint  (const proposal_info_t& proposal);
    std::unique_ptr<LocalArea> createDestination    (const proposal_info_t& proposal);
    std::unique_ptr<LocalArea> createPathSegment    (const proposal_info_t& proposal);

    LocalPerceptualMap   createPlaceMetricMap(const AreaExtent& extent, const LocalPerceptualMap& map);
    math::Rectangle<int> calculateMapBoundary(const AreaExtent& extent, const LocalPerceptualMap& map);
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_BUILDER_H
