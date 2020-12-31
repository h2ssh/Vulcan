/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_association.cpp
* \author   Collin Johnson
*
* Definition of AreaHypothesisAssociation.
*/

#include "hssh/local_topological/area_detection/labeling/hypothesis_association.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_graph.h"
#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/algorithm_ext.h"
#include <boost/range/iterator_range.hpp>

// #define DEBUG_SCORES

namespace vulcan
{
namespace hssh
{

/*
* score_t keeps track of the association of a current area with a prior one. Thus, it is the amount of
* the current area's skeleton contained in the prior area. Similarly for the contained boundaries.
*/
struct score_t
{
    AreaHypothesis* prior;
    double amountCellsContained;
    int numSharedBoundaries;
    int numContainedBoundaries;

    score_t(AreaHypothesis& prior, AreaHypothesis& current, const VoronoiSkeletonGrid& skeleton);
};

bool operator>(const score_t& lhs, const score_t& rhs)
{
    double lhsScore = (2 * lhs.numSharedBoundaries) + lhs.numContainedBoundaries + lhs.amountCellsContained;
    double rhsScore = (2 * rhs.numSharedBoundaries) + rhs.numContainedBoundaries + rhs.amountCellsContained;

    return (lhs.numSharedBoundaries > rhs.numSharedBoundaries)
        || ((lhs.numSharedBoundaries == rhs.numSharedBoundaries) && (lhsScore > rhsScore));
}

std::ostream& operator<<(std::ostream& out, const score_t& score);


double amount_cells_contained(AreaHypothesis& prior, AreaHypothesis& current, const VoronoiSkeletonGrid& skeleton);
std::tuple<int, int> num_shared_and_contained_boundaries(AreaHypothesis& prior, AreaHypothesis& current);


AreaHypothesisAssociation::AreaHypothesisAssociation(void)
{
    // Nothing to initialize
}


AreaHypothesisAssociation::AreaHypothesisAssociation(HypothesisGraph& prior,
                                                     HypothesisGraph& current,
                                                     const VoronoiSkeletonGrid& skeleton)
{
    // Ensure there are valid hypotheses before attempting to create an association between them
    if((prior.numHypotheses() > 0) && (current.numHypotheses() > 0))
    {
        findAssociationsWith(prior, current, skeleton);
    }
}


bool AreaHypothesisAssociation::isAssociatedWith(AreaHypothesis* prior, AreaHypothesis* current) const
{
    return std::find(beginAssociated(prior), endAssociated(prior), current) != endAssociated(prior);
}


AreaHypothesisAssociation::AssocVecIter AreaHypothesisAssociation::beginAssociated(AreaHypothesis* hyp) const
{
    auto assocIt = associations_.find(hyp);
    return (assocIt == associations_.end()) ? nullAssociation_.begin() : assocIt->second.begin();
}


AreaHypothesisAssociation::AssocVecIter AreaHypothesisAssociation::endAssociated(AreaHypothesis* hyp) const
{
    auto assocIt = associations_.find(hyp);
    return (assocIt == associations_.end()) ? nullAssociation_.end() : assocIt->second.end();
}


AreaHypothesisAssociation::AssocVec::size_type AreaHypothesisAssociation::sizeAssociated(AreaHypothesis* hyp) const
{
    auto assocIt = associations_.find(hyp);
    return (assocIt == associations_.end()) ? 0 : assocIt->second.size();
}


void AreaHypothesisAssociation::findAssociationsWith(HypothesisGraph& prior,
                                                     HypothesisGraph& current,
                                                     const VoronoiSkeletonGrid& skeleton)
{
    // Each current is matched with at most one prior hypothesis. The associated hypothesis with the one that returns
    // is associated and has the maximum area amongst all possible matches
    for(AreaHypothesis* currHyp : boost::make_iterator_range(current.beginHypothesis(), current.endHypothesis()))
    {
        // Find prior -> current scores
        std::vector<score_t> scores;
        std::transform(prior.beginHypothesis(),
                       prior.endHypothesis(),
                       std::back_inserter(scores),
                       [&currHyp, &skeleton](AreaHypothesis* priorHyp) {
            return score_t(*priorHyp, *currHyp, skeleton);
        });

        // Preference is always given to areas with more shared boundaries
        std::sort(scores.begin(), scores.end(), std::greater<score_t>());

        AreaHypothesis* priorMatch = nullptr;

        if(!scores.empty()
            && ((scores.front().numSharedBoundaries + scores.front().numContainedBoundaries > 1)
                || scores.front().amountCellsContained > 0.25))
        {
            priorMatch = scores.front().prior;
        }

#ifdef DEBUG_SCORES
        if(!scores.empty())
        {
            const auto& prior = *scores.front().prior;
            std::cout << "Best match stats: Gwys: ";
            for(auto& currBoundary : boost::make_iterator_range(currHyp->beginBoundary(), currHyp->endBoundary()))
            {
                bool isContained = prior.extent().contains(currBoundary->getGateway().center(), math::ReferenceFrame::GLOBAL)
                    && prior.extent().cellContains(currBoundary->getGateway().center());
                std::cout << currBoundary->getGateway().center() << ':' << isContained << ' ';
            }
            std::cout << '\n';

            std::cout << "Containment cells: ";
            for(auto& cell : boost::make_iterator_range(currHyp->beginSkeleton(), currHyp->endSkeleton()))
            {
                auto globalPoint = utils::grid_point_to_global_point(cell, skeleton);
                bool contained = (prior.extent().contains(globalPoint, math::ReferenceFrame::GLOBAL)
                    && prior.extent().cellContains(globalPoint));
                std::cout << globalPoint << ':' << contained << ' ';
            }
            std::cout << '\n';
        }
#endif // DEBUG_SCORES

        // If there's at least one contained node, then we'll call it a potential match
        if(priorMatch && !isAssociatedWith(priorMatch, currHyp))
        {
            // Then create a bidirectional association between the two hypotheses
            associations_[priorMatch].push_back(currHyp);
            associations_[currHyp].push_back(priorMatch);
        }

#ifdef DEBUG_SCORES
        std::cout << "INFO: AreaHypAssociation: Scores for " << currHyp->rectangleBoundary() << ":\n";
        std::copy(scores.begin(), scores.end(), std::ostream_iterator<score_t>(std::cout, "\n"));
#endif
    }
}


score_t::score_t(AreaHypothesis& prior, AreaHypothesis& current, const VoronoiSkeletonGrid& grid)
: prior(&prior)
, amountCellsContained(amount_cells_contained(prior, current, grid))
{
    std::tie(numSharedBoundaries, numContainedBoundaries) = num_shared_and_contained_boundaries(prior, current);
}


std::ostream& operator<<(std::ostream& out, const score_t& score)
{
    out << score.prior->rectangleBoundary() << " : " << score.amountCellsContained << ','
        << score.numSharedBoundaries << ',' << score.numContainedBoundaries;
    return out;
}


double amount_cells_contained(AreaHypothesis& prior, AreaHypothesis& current, const VoronoiSkeletonGrid& skeleton)
{
    if(current.numSkeleton() == 0)
    {
        return 0.0;
    }

    double numContained = 0.0;

    for(auto& cell : boost::make_iterator_range(current.beginSkeleton(), current.endSkeleton()))
    {
        auto globalPoint = utils::grid_point_to_global_point(cell, skeleton);
        if(prior.extent().cellContains(globalPoint))
        {
            numContained += 1.0;
        }
    }

    return numContained / current.numSkeleton();
}


std::tuple<int, int> num_shared_and_contained_boundaries(AreaHypothesis& prior, AreaHypothesis& current)
{
    int numShared = 0;
    int numContained = 0;

    std::vector<const AreaHypothesisBoundary*> matched;

    for(auto& currBoundary : boost::make_iterator_range(current.beginBoundary(), current.endBoundary()))
    {
        bool isShared = false;

        for(auto& priorBoundary : boost::make_iterator_range(prior.beginBoundary(), prior.endBoundary()))
        {
            if(!utils::contains(matched, priorBoundary)
                && currBoundary->getGateway().isSimilarTo(priorBoundary->getGateway()))
            {
                matched.push_back(priorBoundary);
                isShared = true;
            }
        }

        if(!isShared && prior.extent().cellContains(currBoundary->getGateway().center()))
        {
            ++numContained;
        }
    }

    numShared = matched.size();

    return std::make_pair(numShared, numContained);
}

} // namespace hssh
} // namespace vulcan
