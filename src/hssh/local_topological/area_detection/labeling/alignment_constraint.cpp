/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     alignment_constraint.cpp
* \author   Collin Johnson
*
* Definition of AlignmentConstraint.
*/

#include "hssh/local_topological/area_detection/labeling/alignment_constraint.h"
#include "hssh/local_topological/area_detection/labeling/alignment_graph.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "utils/algorithm_ext.h"
#include <boost/range/iterator_range.hpp>
#include <cassert>

// #define DEBUG_UNSATISFIED

namespace vulcan
{
namespace hssh
{

bool is_endpoints_satisfied(int numOutsidePathEnd,
                            int numOutsidePathDest,
                            int numOutsideDest,
                            int numOutsideDecision,
                            int numAdjacent,
                            HypothesisType inside);
bool is_unaligned_satisfied(int numOutsidePathEnd,
                            int numOutsidePathDest,
                            int numOutsideDest,
                            int numOutsideDecision,
                            HypothesisType inside);
bool is_aligned_satisfied(int numOutsidePathEnd,
                          int numOutsidePathDest,
                          int numOutsideDest,
                          int numOutsideDecision,
                          HypothesisType inside);
bool is_adjacent_satisfied(int numOutsidePathEnd,
                           int numOutsidePathDest,
                           int numOutsideDest,
                           int numOutsideDecision,
                           HypothesisType inside);
bool is_all_neighbors_satisfied(int numOutsidePathEnd,
                                int numOutsidePathDest,
                                int numOutsideDest,
                                int numOutsideDecision,
                                HypothesisType inside);
bool is_exit_satisfied(int numOutsidePathEnd,
                       int numOutsidePathDest,
                       int numOutsideDest,
                       int numOutsideDecision,
                       HypothesisType inside);


bool operator<(const ConstraintAdjacentArea& lhs, const ConstraintAdjacentArea& rhs)
{
    return (lhs.id < rhs.id) || (lhs.id == rhs.id && lhs.gatewayId < rhs.gatewayId);
}


bool operator==(const ConstraintAdjacentArea& lhs, const ConstraintAdjacentArea& rhs)
{
    return (lhs.id == rhs.id) && (lhs.gatewayId == rhs.gatewayId);
}


bool operator!=(const ConstraintAdjacentArea& lhs, const ConstraintAdjacentArea& rhs)
{
    return !(lhs == rhs);
}


std::ostream& operator<<(std::ostream& out, const ConstraintAdjacentArea& c)
{
    out << c.id << ':' << c.gatewayId;
    return out;
}


std::ostream& operator<<(std::ostream& out, const AlignmentConstraint& c)
{
    switch(c.type())
    {
        case AlignmentConstraint::endpoints:
            out << "Endpoints: ";
            break;
        case AlignmentConstraint::unaligned:
            out << "Unaligned: ";
            break;
        case AlignmentConstraint::aligned:
            out << "Aligned: ";
            break;
        case AlignmentConstraint::adjacent:
            out << "Adjacent: ";
            break;
        case AlignmentConstraint::all_neighbors:
            out << "All Neighbors: ";
            break;
        case AlignmentConstraint::fixed:
            out << "Fixed: ";
            break;
        case AlignmentConstraint::exit:
            out << "Exit: ";
            break;
        case AlignmentConstraint::connectivity:
            out << "Connectivity: ";
    }
    out << c.insideId() << "->";
    std::copy(c.beginAdjacent(), c.endAdjacent(), std::ostream_iterator<ConstraintAdjacentArea>(out, ","));
    return out;
}


bool operator==(const AlignmentConstraint& lhs, const AlignmentConstraint& rhs)
{
    if((lhs.type() != rhs.type()) || (lhs.sizeAdjacent() != rhs.sizeAdjacent()))
//         || (lhs.insideId() != rhs.insideId())
    {
        return false;
    }

    for(auto adj : boost::make_iterator_range(lhs.beginAdjacent(), lhs.endAdjacent()))
    {
        if(std::find(rhs.beginAdjacent(), rhs.endAdjacent(), adj) == rhs.endAdjacent())
        {
            return false;
        }
    }

    return true;
}


bool operator!=(const AlignmentConstraint& lhs, const AlignmentConstraint& rhs)
{
    return !(lhs == rhs);
}


AlignmentConstraint AlignmentConstraint::CreateEndpointsConstraint(AdjVec endpoints, int inside)
{
    return AlignmentConstraint(Type::endpoints, inside, std::move(endpoints), HypothesisType::kArea);
}


AlignmentConstraint AlignmentConstraint::CreateUnalignedConstraint(AdjVec unaligned, int inside)
{
    return AlignmentConstraint(Type::unaligned, inside, std::move(unaligned), HypothesisType::kArea);
}


AlignmentConstraint AlignmentConstraint::CreateAlignedConstraint(AdjVec aligned, int inside)
{
    return AlignmentConstraint(Type::aligned, inside, std::move(aligned), HypothesisType::kArea);
}


AlignmentConstraint AlignmentConstraint::CreateAdjacentConstraint(ConstraintAdjacentArea adjacent, int inside)
{
    return AlignmentConstraint(Type::adjacent, inside, AdjVec{adjacent}, HypothesisType::kArea);
}


AlignmentConstraint AlignmentConstraint::CreateAllNeighborsConstraint(AdjVec neighbors, int inside)
{
    return AlignmentConstraint(Type::all_neighbors, inside, std::move(neighbors), HypothesisType::kArea);
}


AlignmentConstraint AlignmentConstraint::CreateFixedConstraint(AdjVec adjacent, int inside, HypothesisType type)
{
    return AlignmentConstraint(Type::fixed, inside, std::move(adjacent), type);
}


AlignmentConstraint AlignmentConstraint::CreateExitConstraint(AdjVec adjacent, int inside, HypothesisType type)
{
    return AlignmentConstraint(Type::exit, inside, std::move(adjacent), type);
}


AlignmentConstraint AlignmentConstraint::CreateConnectivityConstraint(AdjVec neighbors, int inside)
{
    return AlignmentConstraint(Type::connectivity, inside, std::move(neighbors), HypothesisType::kArea);
}


AlignmentConstraint::AlignmentConstraint(Type type, int insideId, AdjVec adjacent, HypothesisType fixedType)
: type_(type)
, areas_(adjacent.size() + 1)       // store all adjacent and the inside
, adjacent_(std::move(adjacent))
, fixedType_(fixedType)

{
    // Cannot fix a type to be an area. Must be a specific label.
    assert((type_ != fixed) || (fixedType != HypothesisType::kArea));

    areas_[0] = insideId;
    std::transform(adjacent_.begin(), adjacent_.end(), areas_.begin() + 1, [](const ConstraintAdjacentArea& a) {
        return a.id;
    });
}


int AlignmentConstraint::insideId(void) const
{
    return areas_.front();
}


AlignmentConstraint::IdIter AlignmentConstraint::beginIds(void) const
{
    return areas_.begin();
}


AlignmentConstraint::IdIter AlignmentConstraint::endIds(void) const
{
    return areas_.end();
}


bool AlignmentConstraint::isSatisfied(const AlignmentGraph& network, HypothesisType insideType) const
{
    if(network.assignedType(areas_.front()) != insideType)
    {
        std::cout << "ERROR: AlignmentConstraint: Assigned inside type must match desired inside type to check. Expect:"
            << insideType << " Assigned:" << network.assignedType(areas_.front()) << " Area:" << areas_.front() << '\n';
        assert(network.assignedType(areas_.front()) == insideType);
    }

    adjIds_.clear();

    for(auto adj : boost::make_iterator_range(beginAdjacent(), endAdjacent()))
    {
        // Count the type for the area
        auto assignedType = network.assignedType(adj.id);
        auto assignedId = network.assignedArea(adj.id);

        if(assignedType == HypothesisType::kPath)
        {
            if(network.isPathEndGateway(adj.id, adj.gatewayId, insideId()))
            {
                assignedType = HypothesisType::kPathEndpoint;
            }
            else
            {
                assignedType = HypothesisType::kPathDest;
            }
        }

        auto adjIt = std::find_if(adjIds_.begin(), adjIds_.end(), [assignedId](auto& adj) {
            return adj.first == assignedId;
        });

        // If found, need to check if we're in a situation with both a path-dest and path-endpoint
        // If the assigned is path-dest, then can always safely change to assignedType, which is also
        // path-dest or path-endpoint. path-endpoint will then latch and never get changed as the assigned type
        if((adjIt != adjIds_.end()) && (adjIt->second == HypothesisType::kPathDest))
        {
            adjIt->second = assignedType;
        }

        // Don't double-count areas
        if(adjIt != adjIds_.end())
        {
            continue;
        }

        adjIds_.emplace_back(assignedId, assignedType);
    }

    int numDest = 0;
    int numPathEnd = 0;
    int numPathDest = 0;
    int numDecision = 0;

    for(auto& adj : adjIds_)
    {
        switch(adj.second)
        {
            case HypothesisType::kDecision:
                ++numDecision;
                break;
            case HypothesisType::kDest:
                ++numDest;
                break;
            case HypothesisType::kPathDest:
                ++numPathDest;
                break;
            case HypothesisType::kPathEndpoint:
                ++numPathEnd;
                break;
            default:
                // ignore all others
                break;
        }
    }

    bool satisfied = false;

    if(type_ == endpoints)
    {
        satisfied = is_endpoints_satisfied(numPathEnd, numPathDest, numDest, numDecision, adjacent_.size(), insideType);
    }
    else if(type_ == unaligned)
    {
        satisfied = is_unaligned_satisfied(numPathEnd, numPathDest, numDest, numDecision, insideType);
    }
    else if(type_ == aligned)
    {
        satisfied = is_aligned_satisfied(numPathEnd, numPathDest, numDest, numDecision, insideType);
    }
    else if(type_ == adjacent)
    {
        satisfied = is_adjacent_satisfied(numPathEnd, numPathDest, numDest, numDecision, insideType);
    }
    else if(type_ == all_neighbors)
    {
        satisfied = is_all_neighbors_satisfied(numPathEnd, numPathDest, numDest, numDecision, insideType);
    }
    else if(type_ == fixed)
    {
        satisfied = insideType == fixedType_;

        if(!satisfied)
        {
            std::cout << "Failed fixed constraint: Expected:" << fixedType_ << " Actual:" << insideType << '\n';
        }
    }
    else if(type_ == exit)
    {
        satisfied = is_exit_satisfied(numPathEnd, numPathDest, numDest, numDecision, insideType);
    }

#ifdef DEBUG_UNSATISFIED
    if(!satisfied)
    {
        std::cout << "Checking constraint:" << *this << " Path End:" << numPathEnd << " Path:" << numPathDest << " Dest:"
            << numDest << " Decision: " << numDecision << " Inside:" << insideType << " Fixed:" << fixedType_
            << std::boolalpha << " Satisfied? " << satisfied << '\n';
    }
#endif // DEBUG_UNSATISFIED

    return satisfied;
}


bool AlignmentConstraint::isUnalignedPathEnds(const AlignmentGraph& network) const
{
    if(type_ != unaligned)
    {
        return false;
    }

    // An area can be adjacent to the same path through two gateways. These gateways will almost certainly be unaligned
    // However, the gateways are unlikely to both be endpoints, or either one be an endpoint really
    // So a simple counting of the ends is sufficient
    int pathEndCount = 0;

    for(auto adj : adjacent_)
    {
        if(network.isPathEndGateway(adj.id, adj.gatewayId, insideId()))
        {
            ++pathEndCount;
        }
    }

    return pathEndCount > 1;
}


bool is_endpoints_satisfied(int numOutsidePathEnd,
                            int numOutsidePathDest,
                            int numOutsideDest,
                            int numOutsideDecision,
                            int numAdjacent,
                            HypothesisType inside)
{
    // The endpoints constraint only applies to paths.
    if(inside != HypothesisType::kPath)
    {
        return true;
    }

    // There can only be adjacent places.
    if(numOutsidePathEnd + numOutsidePathDest > 0)
    {
        return false;
    }

    // If adjacent to only one other area, then it must be a place of some kind
    if(numAdjacent == 1)
    {
        return numOutsideDest + numOutsideDecision >= 1;
    }
    // If adjacent to more than one area at the endpoints, there must be only two areas.
    else if(numAdjacent > 1)
    {
        return numOutsideDest + numOutsideDecision >= 1;
    }

    // The constraint is always satisfied if not adjacent to any non-dead ends
    return true;
}


bool is_unaligned_satisfied(int numOutsidePathEnd,
                            int numOutsidePathDest,
                            int numOutsideDest,
                            int numOutsideDecision,
                            HypothesisType inside)
{
    // Unaligned path ends are either decision points or paths that have gone around small bends
    if(numOutsidePathEnd == 2)
    {
//         return inside != HypothesisType::kDest;
        return inside == HypothesisType::kDecision;
    }
    // Two non-path ends must be destinations because only destinations can exist along paths
    else if(numOutsidePathDest == 2)
    {
        return inside == HypothesisType::kDest;
    }
    // Two unaligned decision points is likely for very long paths, but might happen for a destination in rare cases
    else if(numOutsideDecision == 2)
    {
        return (inside == HypothesisType::kPath) || (inside == HypothesisType::kDest);
    }
    // Destinations can't be adjacent to other destinations. They must all be merged into a single destination.
    else if(numOutsideDest == 2)
    {
        return inside != HypothesisType::kDest;
    }
    // If a path terminates somewhere, but another path is running by, that isn't valid. The path would just run
    //  through the area fully
    else if((numOutsidePathEnd == 1) && (numOutsidePathDest == 1))
    {
        return inside == HypothesisType::kDest;
    }
    // If a path terminates at an area, it can't also be adjacent to a decision because the path should run to the
    // decision point or the decision point should include that area
    else if((numOutsidePathEnd == 1) && (numOutsideDecision == 1))
    {
        return false;
    }
    // If a path terminates somewhere and is also adjacent to a destination, it must be a decision point
    else if((numOutsidePathEnd == 1) && (numOutsideDest == 1))
    {
        return inside == HypothesisType::kDecision;
    }
    // If along a path, then must be a destination. Weird cases could result in an unaligned decision, especially for
    // a destination with multiple exits along different area
    else if((numOutsidePathDest == 1) && (numOutsideDecision == 1))
    {
        return inside == HypothesisType::kDest;
    }
    // Being along a path means a destination, but if also unaligned with another destination, that situation can't
    // occur because the destinations should be a single destination.
    else if((numOutsidePathDest == 1) && (numOutsideDest == 1))
    {
        return false;
    }
    // Destinations and decision points will frequently be unaligned along a path, but they must be along a path
    else if((numOutsideDest == 1) && (numOutsideDecision == 1))
    {
        return inside == HypothesisType::kPath;
    }

    return true;
}


bool is_aligned_satisfied(int numOutsidePathEnd,
                          int numOutsidePathDest,
                          int numOutsideDest,
                          int numOutsideDecision,
                          HypothesisType inside)
{
    // Only dests can be adjacent to path sides
    if(numOutsidePathDest == 2)
    {
        return inside == HypothesisType::kDest;
    }
    else if(numOutsidePathDest == 1 && numOutsideDecision == 1)
    {
        return inside == HypothesisType::kDest;
    }
    else if(numOutsidePathDest == 1 && numOutsidePathEnd == 1)
    {
        return false;
    }
    // Unaligned path ends are either decision points or paths that have gone around small bends
    else if(numOutsidePathEnd == 2)
    {
        return inside == HypothesisType::kDecision;
    }
    // Two aligned decision points is likely for very long paths, but might happen for a destination in rare cases
    else if(numOutsideDecision == 2)
    {
        return inside != HypothesisType::kDecision;
    }
    // Destinations can't be adjacent to other destinations. They must all be merged into a single destination.
    else if(numOutsideDest == 2)
    {
        return inside != HypothesisType::kDest;
    }
    // If a path terminates at an area, it can't also be adjacent to a decision because the path should run to the
    // decision point or the decision point should include that area
    else if((numOutsidePathEnd == 1) && (numOutsideDecision == 1))
    {
        return false;
    }
    // If a path terminates somewhere and is also adjacent to a destination, it must be a decision point
    else if((numOutsidePathEnd == 1) && (numOutsideDest == 1))
    {
        return inside == HypothesisType::kDecision;
    }
    // Destinations and decision points will frequently be unaligned along a path, but they must be along a path
    else if((numOutsideDest == 1) && (numOutsideDecision == 1))
    {
        return inside == HypothesisType::kPath;
    }

    // No other combinations should exist or are allowed
//     assert(!"Invalid number of areas for unaligned constraint.");
    return true;
}


bool is_adjacent_satisfied(int numOutsidePathEnd,
                           int numOutsidePathDest,
                           int numOutsideDest,
                           int numOutsideDecision,
                           HypothesisType inside)
{
    // NOTE: Adjacent constraints are not defined for endpoint gateways. Those are endpoint constraints only.

    // If terminating a path, then must be a place
    if(numOutsidePathEnd == 1)
    {
        return (inside == HypothesisType::kDest) || (inside == HypothesisType::kDecision);
    }
    // If along a path, then must be a destination
    else if(numOutsidePathDest == 1)
    {
        return inside == HypothesisType::kDest;
    }
    // If the single gateway leads to a decision point, it can't lead to another decision.
    else if(numOutsideDecision == 1)
    {
        return (inside == HypothesisType::kDest) || (inside == HypothesisType::kPath);
    }
    // If the single gateway leads to a destination, then the adjacent area must be a path or decision
    else if(numOutsideDest == 1)
    {
        return (inside == HypothesisType::kDecision) || (inside == HypothesisType::kPath);
    }

    // No other combinations are allowed
    assert(!"Invalid number of areas for adjacent constraint.");
    return false;
}


bool is_all_neighbors_satisfied(int numOutsidePathEnd,
                                int numOutsidePathDest,
                                int numOutsideDest,
                                int numOutsideDecision,
                                HypothesisType inside)
{
    // A decision must have two or more path ends incident to it, no decisions, no path-dests, and any number of dest
    if(inside == HypothesisType::kDecision)
    {
        return (numOutsidePathEnd > 1) && (numOutsidePathDest == 0) && (numOutsideDecision == 0);
    }
    // A destination can be adjacent to at one endpoint or decision and any number of path dests and no other dests
    else if(inside == HypothesisType::kDest)
    {
        if(numOutsideDest > 0)
        {
            return false;
        }

        if(numOutsidePathEnd > 0)
        {
            return numOutsidePathEnd == 1;
//             return numOutsidePathDest == 0;
//             return (numOutsideDecision == 0) && (numOutsidePathEnd == 1) && (numOutsidePathDest == 0);
        }

//         if(numOutsideDecision > 0)
//         {
//             return numOutsidePathDest == 0;
// //             return (numOutsideDecision == 1) && (numOutsidePathEnd == 0) && (numOutsidePathDest == 0);
//         }

        return true; //numOutsideDecision < 2;
    }
    // A path can be adjacent to no other path ends or path dests and at most two decisions.
    else if(inside == HypothesisType::kPath)
    {
        return (numOutsidePathDest == 0) && (numOutsidePathEnd == 0) && (numOutsideDecision <= 2);
    }

    std::cerr << "ERROR: Unknown hypothesis type for AlignmentConstraint:" << inside << '\n';
    assert(!"Unknown hypothesis type for AlignmentConstraint.");
    return false;
}


bool is_exit_satisfied(int numOutsidePathEnd,
                       int numOutsidePathDest,
                       int numOutsideDest,
                       int numOutsideDecision,
                       HypothesisType inside)
{
    if(inside == HypothesisType::kDecision)
    {
        return numOutsideDecision == 0;
    }
    else if(inside == HypothesisType::kDest)
    {
        return numOutsideDest == 0;
    }
    else if(inside == HypothesisType::kPath)
    {
        return numOutsidePathEnd + numOutsidePathDest == 0;
    }

    // Unknown type always succeeds.
    return true;
}

} // namespace hssh
} // namespace vulcan
