/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     types.h
 * \author   Collin Johnson
 *
 * Definition of types that are common to multiple levels of the HSSH:
 *
 * Grid types:
 *   - cell_t  : representation of a cell position
 *
 * Topological types:
 *   - AreaType      : enumeration of all possible area types
 *   - TopoDirection : direction of motion when moving along a path or crossing a transition.
 */

#ifndef HSSH_TYPES_H
#define HSSH_TYPES_H

#include "core/point.h"
#include "core/point_util.h"
#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vulcan
{
namespace hssh
{

//////////////////// Grid types ////////////////////////
using cell_idx_t = int16_t;
using cell_t = Point<cell_idx_t>;

using CellHash = PointHash<cell_idx_t>;

template <typename T>
using CellToTypeMap = std::unordered_map<cell_t, T, CellHash>;

using CellSet = std::unordered_set<cell_t, CellHash>;

using CellToIntMap = CellToTypeMap<int>;
using CellVector = std::vector<cell_t>;

using CellIter = CellVector::iterator;
using CellConstIter = CellVector::const_iterator;

using CellRange = std::pair<CellIter, CellIter>;
using CellConstRange = std::pair<CellConstIter, CellConstIter>;

/////////////////// Topology types ////////////////////

/**
 * AreaType defines all possible types of areas exist within the HSSH. Anything that needs an identifier for the type of
 * area should use AreaType.
 */
enum class AreaType
{
    area,
    place,
    decision_point,
    destination,
    path_segment,
    path,
    dead_end,
    frontier,
};

/**
 * is_place_type checks if the type is a place. Either destination or decision_point.
 */
constexpr bool is_place_type(AreaType type)
{
    return (type == AreaType::decision_point) || (type == AreaType::destination);
}


inline std::ostream& operator<<(std::ostream& out, AreaType type)
{
    switch (type) {
    case AreaType::area:
        out << "area";
        break;
    case AreaType::place:
        out << "place";
        break;
    case AreaType::decision_point:
        out << "decision_point";
        break;
    case AreaType::destination:
        out << "destination";
        break;
    case AreaType::path_segment:
        out << "path_segment";
        break;
    case AreaType::path:
        out << "path";
        break;
    case AreaType::dead_end:
        out << "dead_end";
        break;
    case AreaType::frontier:
        out << "frontier";
        break;
    default:
        out << "unknown";
        assert(false);
        break;
    }

    return out;
}


inline std::istream& operator>>(std::istream& in, AreaType& type)
{
    std::string str;
    in >> str;

    if (str == "area") {
        type = AreaType::area;
    } else if (str == "place") {
        type = AreaType::place;
    } else if (str == "decision_point") {
        type = AreaType::decision_point;
    } else if (str == "destination") {
        type = AreaType::decision_point;
    } else if (str == "path_segment") {
        type = AreaType::path_segment;
    } else if (str == "path") {
        type = AreaType::path;
    } else if (str == "dead_end") {
        type = AreaType::dead_end;
    } else if (str == "frontier") {
        type = AreaType::frontier;
    } else {
        assert(!"Unknown area type string");
        type = AreaType::area;
    }

    return in;
}


/**
 * TopoDirection defines the two topological directions that exist, plus and minus. The choice of plus and minus is
 * arbitrary -- it could just as easily be forward and backward -- the key is that there are two possibilities.
 */
enum class TopoDirection
{
    plus,
    minus,
    null,
};


/**
 * opposite_direction gets the topological direction that's the opposite of whatever direction is passed in. As there
 * are only two directions, plus and minus, plus->minus and minus->plus.
 *
 * \param    direction       Direction to get the opposite of
 * \return   minus if direction == plus or plus if direction == minus.
 */
inline TopoDirection opposite_direction(TopoDirection direction)
{
    switch (direction) {
    case TopoDirection::plus:
        return TopoDirection::minus;

    case TopoDirection::minus:
        return TopoDirection::plus;

    case TopoDirection::null:
        return TopoDirection::null;

    default:   // shouldn't happen
        assert((direction == TopoDirection::plus) || (direction == TopoDirection::minus)
               || (direction == TopoDirection::null));
    }

    return TopoDirection::null;
}


inline std::ostream& operator<<(std::ostream& out, TopoDirection direction)
{
    switch (direction) {
    case TopoDirection::plus:
        out << "plus";
        break;

    case TopoDirection::minus:
        out << "minus";
        break;

    case TopoDirection::null:
        out << "null";
        break;

    default:
        out << "unknown";
        assert(direction == TopoDirection::plus || direction == TopoDirection::minus
               || direction == TopoDirection::null);
    }

    return out;
}

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_TYPES_H
