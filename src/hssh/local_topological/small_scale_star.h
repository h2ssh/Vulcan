/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     small_scale_star.h
* \author   Collin Johnson
*
* Definition of SmallScaleStar, local_path_fragment_t, and path_fragment_direction_t.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_STAR_H
#define HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_STAR_H

#include <hssh/local_topological/gateway.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <array>
#include <cstdint>
#include <cstddef>
#include <ostream>
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
* path_fragment_direction_t defines the two directions possible for a path fragment. The assignment
* of direction is left to the particular implmentation of the small-scale star. The only
* requirement is the two path fragments in a path have different directions.
*/
enum path_fragment_direction_t
{
    PATH_FRAGMENT_PLUS,
    PATH_FRAGMENT_MINUS
};

/**
* local_path_fragment_t describes a path in the small-scale star. A path fragment represents
* one half of a path in a place. The direction of a place is either PLUS or MINUS, a PLUS is
* assigned to path fragments with an orientation in the range (-pi/2, pi/2].
*
* A path fragment also has a navigable flag. If a particular fragment is not represented by
* an existing path in the place, then the fragment is an endpoint of the larger path structure.
* Such paths are marked as not navigable.
*
* A path fragment has an assigned path index. This index is shared by two path fragments in each
* small-scale star and indicates the two fragments that makeup the complete path through the place.
*
* The direction of the gateway in the fragment is inbound to the place.
*/
struct local_path_fragment_t
{
    int8_t                    fragmentId;       ///< Id of the fragment within the star
    int8_t                    pathId;           ///< Id of the path in the star this fragment is part of
    path_fragment_direction_t direction;        ///< Direction along the path this fragment travels
    bool                      navigable;        ///< Flag if the fragment is navigable or not
    Gateway                   gateway;          ///< Gateway to use for traveling to this fragment
    AreaType                  type;             ///< Type of area this fragment leads to

    local_path_fragment_t(void)
    : fragmentId(-1)
    , pathId(-1)
    , direction(PATH_FRAGMENT_MINUS)
    , navigable(false)
    , type(AreaType::area)
    {
    }
};

/// A local_star_path_t contains the path fragments for a path in the star
using local_star_path_t = std::array<local_path_fragment_t, 2>;

inline bool operator==(const local_path_fragment_t& lhs, const local_path_fragment_t& rhs)
{
    return (lhs.pathId == rhs.pathId) && (lhs.direction == rhs.direction) && (lhs.navigable == rhs.navigable);
}

inline bool operator!=(const local_path_fragment_t& lhs, const local_path_fragment_t& rhs)
{
    return !(lhs == rhs);
}

inline bool operator<(const local_path_fragment_t& lhs, const local_path_fragment_t& rhs)
{
    float lhsAngle = (lhs.gateway.direction() < 0) ? lhs.gateway.direction() + 2*M_PI : lhs.gateway.direction();
    float rhsAngle = (rhs.gateway.direction() < 0) ? rhs.gateway.direction() + 2*M_PI : rhs.gateway.direction();

    return lhsAngle < rhsAngle;
}

// Serialization for local_path_fragment_t
template <class Archive>
void serialize(Archive& ar, local_path_fragment_t& frag)
{
    ar (frag.fragmentId,
        frag.pathId,
        frag.direction,
        frag.navigable,
        frag.gateway,
        frag.type
    );
}

/**
* SmallScaleStar is the symbolic representation of a Place. A small-scale star contains descriptions about
* the paths incident with a place. Each path is described by two local path fragments representing the two
* directions of the path.
*
* SmallScaleStar has the following invariants:
*
*   - The path fragments are sorted counter-clockwise around the star.
*   - The direction of the gateway associated with a path fragment points outward. It is the direction one would be
*       traveling if leaving the area through that gateway.
*   - For a path with two fragments, the plus direction is the one where the angle from one gateway to the other is
*       positive. atan(minus.gateway.center - plus.gateway.center) > 0.
*   - For a path with one fragment, the fragment will be the plus direction if the angle from the center of the area
*       to the center of the gateway is positive.
*   - The path ids for all fragments start at 0 and with have an id in the range [0, numPaths).
*/
class SmallScaleStar
{
public:

    using ConstIter = std::vector<local_path_fragment_t>::const_iterator;

    /**
    * Default constructor for SmallScaleStar.
    *
    * Creates a star with no fragments.
    */
    SmallScaleStar(void);

    /**
    * Constructor for SmallScaleStar.
    *
    * \param    fragments           Path fragments that make up the small-scale star
    */
    explicit SmallScaleStar(const std::vector<local_path_fragment_t>& fragments);

    // Operator overloads
    bool operator==(const SmallScaleStar& rhs) const;
    bool operator!=(const SmallScaleStar& rhs) const { return !this->operator==(rhs); }

    /**
    * fragmentsLeftOf finds the path fragments logically to the left of the supplied fragment.
    */
    std::vector<local_path_fragment_t> fragmentsLeftOf(const local_path_fragment_t& fragment) const;

    /**
    * fragmentsRightOf finds the path fragments logically to the right of the supplied fragment.
    */
    std::vector<local_path_fragment_t> fragmentsRightOf(const local_path_fragment_t& fragment) const;

    /**
    * otherFragmentOnPath finds the other fragment on the same path as the provided fragment.
    */
    local_path_fragment_t otherFragmentOnPath(const local_path_fragment_t& fragment) const;

    /**
    * getAllFragments retrieves a vector of all path fragments in the star.
    */
    const std::vector<local_path_fragment_t>& getAllFragments(void) const { return fragments; }

    /**
    * hasFragmentWithId checks to see if the star contains a fragment with the specified id.
    *
    * \param    id          Id of the fragment to check
    * \return   True if a fragment exists with the specified id. False otherwise.
    */
    bool hasFragmentWithId(uint8_t id) const { return id < fragments.size(); }

    /**
    * getFragmentWithId retrieves the fragment with the specified fragmentId.
    *
    * Expect fireworks if trying to use an invalid id.
    *
    * \pre      fragmentId < getNumPaths()*2
    * \param    id          Id of the fragment to retrieve
    * \return   Fragment with specified id.
    */
    local_path_fragment_t getFragmentWithId(uint8_t id) const { return fragments[id]; }

    /**
    * getNumPaths retrieves the number of paths associated with the star.
    *
    * \return   The number of unique paths in the star.
    */
    std::size_t getNumPaths(void) const { return fragments.size()/2; }

    /**
    * getNumNavigableFragments retrieves the number of navigable path fragments in the star.
    */
    std::size_t getNumNavigableFragments(void) const;

    /**
    * getPath retrieves the path with the provided index. If pathNum > getNumPaths(), both path fragments
    * will be not be navigable, indicating an invalid path.
    *
    * \param    pathNum         Number of the path to retrieve
    * \return   The desired path if pathNum < getNumPaths(). Otherwise, an empty path.
    */
    local_star_path_t getPath(std::size_t pathNum) const;

    /**
    * getPathFragment retrieves the particular path fragment requested. getPathFragment() begins the
    * iteration process for the current star. Subsequent calls to next() will be based on a traversal
    * of path fragments based on requested path fragment.
    *
    * \param    pathIndex       Path index of the fragment
    * \param    direction       Direction of the fragment
    * \param    fragment        The fragment to be obtained (output)
    * \return   True if a fragment with the specified parameters exists.
    */
    bool getPathFragment(int8_t pathIndex, path_fragment_direction_t direction, local_path_fragment_t& fragment);

    // Iterators to iterate through all the path fragments, beginning with path 0
    ConstIter begin(void) const { return fragments.begin(); }
    ConstIter end(void)   const { return fragments.end(); }

private:

    std::vector<local_path_fragment_t> fragments;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive & ar)
    {
        ar (fragments);
    }
};


std::ostream& operator<<(std::ostream& out, const SmallScaleStar& star);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_STAR_H
