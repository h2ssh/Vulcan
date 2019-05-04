/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     alignment_constraint.h
* \author   Collin Johnson
*
* Declaration of AlignmentConstraint and AlignmentGraph.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_CONSTRAINT_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_CONSTRAINT_H

#include <hssh/local_topological/area_detection/labeling/hypothesis_type.h>
#include <array>
#include <iosfwd>
#include <utility>
#include <vector>

namespace vulcan
{
namespace hssh
{

class AlignmentGraph;
class AreaHypothesis;

/**
* ConstraintAdjacentArea defines the area id and gatewaay id of the adjacent area associated with the constraint.
*/
struct ConstraintAdjacentArea
{
    int id;
    int32_t gatewayId;
};

/**
* AlignmentConstraint enforces constraints between pairs of gateways that boundary an area. Gateways can have two
* possible alignments:
*
*   - aligned : the gateways belong to the same path in the small-scale star
*   - unaligned : the gateways belong to different paths in the small-scale star
*
* A constraint consists of two outside and one inside area. The outside areas are those bounded by only one of the two
* gateways, while the inside is bounded by both gateways.
*
* The valid states for areas are defined below, split between the outside and inside areas. The ordering of the outside
* areas doesn't matter.
*
*              Outside:    Inside:          Notes:
*   Aligned:
*
*               path,path   path,dp         Destination can't have a path running through it
*               path,dst    any
*               path,dp     path,dp         Destination can't close loops between path and decision
*               dp,dp       path,dp         Destination can't close loops between path and decision
*               dp,dst      any
*               dst,dst     any
*
*   Unaligned:
*
*               path,path   dp              Unaligned paths are the definition of a decision point
*               path,dst    any
*               path,dp     dp              A path unaligned with a decision is still a decision
*               dp,dp       dp              Unaligned decisions must have a decision between them
*               dp,dst      any
*               dst,dst     any
*
*   Aligned:
*               path,path   dp              Aligned paths can only have a decision point between them
*               path,dst    dp,dst          Aligned path and dest only occurs at a place
*               path,dp     none            If a path and decision are aligned, nothing can go between
*               dp,dp       path            Aligned decision points can only have a path between them
*               dp,dst      path,dst        Aligned decision and dest can have a dest or path between
*               dst,dst     dp,path         Aligned dests can have paths or decision
*
*   End:        dp          dest,path       End area can only be a dest or path
*               dest        dest            The outside for a path can only connect to decision
*               path        dest            Any other connection must be a destination
*
*   Fixed:      The area must be assigned a single fixed type.
*
*   Exit:       Neighboring mutable areas cannot be the same type, which avoid the exit transition being merged away.
*
*/
class AlignmentConstraint
{
public:

    enum Type
    {
        endpoints,
        unaligned,
        aligned,
        adjacent,
        all_neighbors,
        fixed,
        connectivity,
        exit,
    };

    using IdVec = std::vector<int>;
    using IdIter = IdVec::const_iterator;
    using AdjVec = std::vector<ConstraintAdjacentArea>;
    using AdjIter = AdjVec::const_iterator;

    /**
    * CreateEndpointsConstraint creates a constraint relating the adjacent areas at the endpoints of the inside area.
    *
    * If there is only one endpoint, this constraint should still be used.
    *
    * \param    endpoints       Area(s) at the endpoints
    * \param    inside          Inside area
    * \return   An endpoints constraint.
    */
    static AlignmentConstraint CreateEndpointsConstraint(AdjVec endpoints, int inside);

    /**
    * CreateUnalignedConstraint creates an unaligned constraint between two adjacent areas.
    *
    * There must be two unaligned areas.
    *
    * \param    unaligned       Unaligned areas
    * \param    inside          Inside area
    * \return   An unaligned constraint.
    */
    static AlignmentConstraint CreateUnalignedConstraint(AdjVec unaligned, int inside);

    /**
    * CreateAlignedConstraint creates an aligned constraint between two adjacent areas.
    *
    * There must be two aligned areas.
    *
    * \param    aligned         Aligned areas
    * \param    inside          Inside area
    * \return   An aligned constraint.
    */
    static AlignmentConstraint CreateAlignedConstraint(AdjVec aligned, int inside);

    /**
    * CreateAdjacentConstraint creates a constraint between non-endpoint adjacent areas.
    *
    * \param    adjacent            Area adjacent
    * \param    inside              Inside area for the constraint
    * \return   An adjacent constraint.
    */
    static AlignmentConstraint CreateAdjacentConstraint(ConstraintAdjacentArea adjacent, int inside);

    /**
    * CreateAllNeighborsConstraint creates a constraint between an area and all of its adjacent neighbors.
    *
    * \param    neighbors           All areas adjacent to inside
    * \param    inside              Area inside the constraint
    * \return   An all_neighbors constraint.
    */
    static AlignmentConstraint CreateAllNeighborsConstraint(AdjVec neighbors, int inside);

    /**
    * CreateFixedConstraint creates a fixed constraint for an area.
    *
    * \param    adjacent            All areas adjacent to the fixed area
    * \param    inside              Area inside the constraint
    * \param    type                Fixed type that the area must always be assigned
    * \return   A fixed constraint.
    */
    static AlignmentConstraint CreateFixedConstraint(AdjVec adjacent, int inside, HypothesisType type);

    /**
    * Create an exit constraint for the exited area with all adjacent mutable areas.
    *
    * \param    adjacent            Adjacent mutable areas
    * \param    inside              Exit area
    * \param    type                Fixed type of the exit area
    */
    static AlignmentConstraint CreateExitConstraint(AdjVec adjacent, int inside, HypothesisType type);

    /**
    * CreateConnectivityConstraint creates a constraint between an area and all of its adjacent neighbors.
    *
    * \param    neighbors           All areas adjacent to inside
    * \param    inside              Area inside the constraint
    * \return   A connectivity constraint.
    */
    static AlignmentConstraint CreateConnectivityConstraint(AdjVec neighbors, int inside);


    AlignmentConstraint(void) { }

    /**
    * isSatisfied checks if the constraint is currently satisifed by the types assigned in the provided network.
    *
    * \param    network
    */
    bool isSatisfied(const AlignmentGraph& network, HypothesisType insideType) const;

    /**
    * isUnalignedPathEnds checks if the constraint is amongst unaligned path ends.
    */
    bool isUnalignedPathEnds(const AlignmentGraph& network) const;

    /**
    * insideId retrieve the id of the inside area, which is adjacent to all of the areas in the constraint. Only
    * the inside are should be added to the adjacency list of its neighbors.
    */
    int insideId(void) const;

    /**
    * beginAdjacent retrieves the iterator to the first adjacent area.
    */
    AdjIter beginAdjacent(void) const { return adjacent_.begin(); }

    /**
    * endAdjacent retrieves the iterator to one-past the last adjacent area.
    */
    AdjIter endAdjacent(void) const { return adjacent_.end(); }

    /**
    * sizeAdjacent retrieves the number of adjacent areas.
    */
    int sizeAdjacent(void) const { return adjacent_.size(); }

    /**
    * beginIds retrieves the iterator to the first of all ids.
    */
    IdIter beginIds(void) const;

    /**
    * endIds retrieves the one-past the end iterator for all ids.
    */
    IdIter endIds(void) const;

    /**
    * type retrieves the type of alignment being constrained.
    */
    Type type(void) const { return type_; }

    /**
    * fixedType retrieves the fixed type that must be assigned to an area when the constraint is a fixed constraint.
    */
    HypothesisType fixedType(void) const { return fixedType_; }

private:

    // INVARIANT: The adjacent ids are stored continguously.

    AlignmentConstraint(Type type, int insideId, AdjVec adjacent, HypothesisType fixedType);

    Type type_;
    IdVec areas_;
    AdjVec adjacent_;
    HypothesisType fixedType_;          // fixed type for a fixed constraint
    mutable std::vector<std::pair<int, HypothesisType>> adjIds_;   // cache for storing the associated id information
};

// Operators for ConstraintAdjacentArea
bool operator<(const ConstraintAdjacentArea& lhs, const ConstraintAdjacentArea& rhs);
bool operator==(const ConstraintAdjacentArea& lhs, const ConstraintAdjacentArea& rhs);
bool operator!=(const ConstraintAdjacentArea& lhs, const ConstraintAdjacentArea& rhs);
std::ostream& operator<<(std::ostream& out, const ConstraintAdjacentArea& c);

// Operators for AlignmentConstraint
/**
* Two constraints are considered the same if they have the same type and areas.
*/
bool operator==(const AlignmentConstraint& lhs, const AlignmentConstraint& rhs);
bool operator!=(const AlignmentConstraint& lhs, const AlignmentConstraint& rhs);
std::ostream& operator<<(std::ostream& out, const AlignmentConstraint& c);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_ALIGNMENT_CONSTRAINT_H

