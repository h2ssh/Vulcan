/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_association.h
* \author   Collin Johnson
*
* Declaration of AreaHypothesisAssociation.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_ASSOCIATION_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_ASSOCIATION_H

#include <unordered_map>
#include <vector>

namespace vulcan
{
namespace hssh
{

class AreaHypothesis;
class HypothesisGraph;
class VoronoiSkeletonGrid;

/**
* AreaHypothesisAssociation associates the AreaHypotheses contained in two HypothesisGraphs. The association between the
* areas is determined using the following criteria:
*
*   1) At least two associated gateways:
*       a) The areas have a boundary that is the same.
*       b) The gateway of one area is inside the extent of the other area.
*
*   2) The center of the area is contained in the extent of the other area
*       a) Only used if an area has less than two gateways.
*/
class AreaHypothesisAssociation
{
public:

    using AssocVec = std::vector<AreaHypothesis*>;
    using AssocVecIter = AssocVec::const_iterator;

    /**
    * Default constructor for AreaHypothesisAssociation.
    *
    * Create an empty association.
    */
    AreaHypothesisAssociation(void);

    /**
    * Constructor for AreaHypothesisAssociation.
    *
    * \param    lhs         Prior hypotheses that the current hypotheses will be matched to
    * \param    rhs         Current hypotheses that will be matched to the prior hypotheses
    * \param    skeleton    Voronoi skeleton from which these hypotheses were created
    */
    AreaHypothesisAssociation(HypothesisGraph& lhs, HypothesisGraph& rhs, const VoronoiSkeletonGrid& skeleton);

    // Access to the area associations

    /**
    * isAssociatedWith checks if a source hypothesis is associated with some destination hypothesis.
    *
    * \param    prior           Hypothesis whose associations are to be checked
    * \param    current         Hypothesis to check if contained within source's associations
    * \return   If std::find(beginAssociated(prior), endAssociated(prior), current) != endAssociated(prior).
    */
    bool isAssociatedWith(AreaHypothesis* prior, AreaHypothesis* current) const;

    /**
    * beginAssociated retrieves the beginning iterator for the hypotheses associated with the provided hypothesis.
    *
    * If no areas are associated with the provided AreaHypothesis, then beginAssociated(hyp) == endAssociated(hyp).
    */
    AssocVecIter beginAssociated(AreaHypothesis* hyp) const;

    /**
    * endAssociated retrieves the one-past-the-end iterator for the hypothesis associated with the provided hypothesis.
    */
    AssocVecIter endAssociated(AreaHypothesis* hyp) const;

    /**
    * sizeAssociated retrieves the number of areas associated with the provided hypothesis.
    */
    AssocVec::size_type sizeAssociated(AreaHypothesis* hyp) const;

private:

    using AssocMap = std::unordered_map<AreaHypothesis*, AssocVec>;

    AssocMap associations_;
    AssocVec nullAssociation_;      // used for creating a valid iterator for hypotheses without associations

    void findAssociationsWith(HypothesisGraph& prior, HypothesisGraph& current, const VoronoiSkeletonGrid& skeleton);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_ASSOCIATION_H
