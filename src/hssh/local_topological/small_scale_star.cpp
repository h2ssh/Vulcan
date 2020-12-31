/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     small_scale_star.cpp
 * \author   Collin Johnson
 *
 * Definition of SmallScaleStar.
 */

#include "hssh/local_topological/small_scale_star.h"
#include <cassert>
#include <iostream>

// #define DEBUG_EQUALITY

namespace vulcan
{
namespace hssh
{

std::ostream& operator<<(std::ostream& out, const SmallScaleStar& star)
{
    auto fragments = star.getAllFragments();

    for (size_t n = 0; n < fragments.size(); ++n) {
        out << fragments[n].gateway << "->" << fragments[n].navigable << ':' << fragments[n].type << ' ';
    }

    return out;
}


SmallScaleStar::SmallScaleStar(void)
{
}


SmallScaleStar::SmallScaleStar(const std::vector<local_path_fragment_t>& fragments) : fragments(fragments)
{
    if (!fragments.empty()) {
        assert(!this->fragments.empty());
        assert(this->fragments.size() % 2 == 0);

        for (size_t n = 0; n < this->fragments.size(); ++n) {
            this->fragments[n].fragmentId = n;
        }
    } else {
        std::cerr << "WARNING: SmallScaleStar: Created a star with no path fragments!\n";
    }
}


bool SmallScaleStar::operator==(const SmallScaleStar& rhs) const
{
    // For two small-scale stars to be the same, they must have the same number of path fragments and sequence
    // of navigable paths must be the same when moving clockwise around the star for some starting point
    // in each star's fragments
    if (fragments.size() == rhs.fragments.size()) {
        bool equal = false;

        for (size_t startIndex = 0; startIndex < rhs.fragments.size() && !equal; ++startIndex) {
            size_t n = 0;
            while ((n < fragments.size())
                   && (fragments[n].navigable == rhs.fragments[(n + startIndex) % fragments.size()].navigable)) {
                ++n;
            }

            // If made it all the way through the fragments, then a match was found for the sequence
            if (n == fragments.size()) {
                equal = true;
            }
        }

#ifdef DEBUG_EQUALITY
        std::cout << "DEBUG:Star==:lhs:";
        for (size_t n = 0; n < fragments.size(); ++n) {
            std::cout << fragments[n].navigable << ' ';
        }
        std::cout << "rhs:";
        for (size_t n = 0; n < rhs.fragments.size(); ++n) {
            std::cout << rhs.fragments[n].navigable << ' ';
        }
        std::cout << "Equal:" << equal << '\n';
#endif

        return equal;
    }

    return false;
}


std::vector<local_path_fragment_t> SmallScaleStar::fragmentsLeftOf(const local_path_fragment_t& fragment) const
{
    // The fragments are ordered counter-clockwise around the star. There are numPaths - 1 fragments left of the
    // provided fragment as all fragments are either left or right except the fragment on the same path.
    // As the fragments are sorted counter-clockwise, increasing the index will grab those fragments to the left

    int numLeftFragments = getNumPaths() - 1;
    std::vector<local_path_fragment_t> leftFragments;
    leftFragments.reserve(numLeftFragments);

    for (int n = 1; n <= numLeftFragments; ++n) {
        leftFragments.push_back(fragments[(n + fragment.fragmentId) % fragments.size()]);
    }

    return leftFragments;
}


std::vector<local_path_fragment_t> SmallScaleStar::fragmentsRightOf(const local_path_fragment_t& fragment) const
{
    // The fragments are ordered counter-clockwise around the star. There are numPaths - 1 fragments right of the
    // provided fragment as all fragments are either left or right except the fragment on the same path.
    // As the fragments are sorted counter-clockwise, decreasing the index will grab those fragments to the right.

    int numRightFragments = getNumPaths() - 1;
    std::vector<local_path_fragment_t> rightFragments;
    rightFragments.reserve(numRightFragments);

    for (int n = 1; n <= numRightFragments; ++n) {
        if (fragment.fragmentId - n >= 0) {
            rightFragments.push_back(fragments[fragment.fragmentId - n]);
        } else {
            rightFragments.push_back(fragments[fragments.size() - 1 - (numRightFragments - n)]);
        }
    }

    return rightFragments;
}


local_path_fragment_t SmallScaleStar::otherFragmentOnPath(const local_path_fragment_t& fragment) const
{
    assert(fragment.fragmentId < static_cast<int>(fragments.size()));
    return fragments[(getNumPaths() + fragment.fragmentId) % fragments.size()];
}


std::size_t SmallScaleStar::getNumNavigableFragments(void) const
{
    auto num = std::size_t{0};

    for (auto& fragment : fragments) {
        if (fragment.navigable) {
            ++num;
        }
    }

    return num;
}


local_star_path_t SmallScaleStar::getPath(std::size_t pathNum) const
{
    if (pathNum >= getNumPaths()) {
        local_path_fragment_t nonnavigable;
        nonnavigable.navigable = false;
        return local_star_path_t{{nonnavigable, nonnavigable}};
    } else {
        local_star_path_t path;
        int nextFrag = 0;
        for (auto& frag : fragments) {
            if (frag.pathId == static_cast<int>(pathNum)) {
                path[nextFrag] = frag;
                ++nextFrag;
            }
        }

        return path;
    }
}


bool SmallScaleStar::getPathFragment(int8_t pathIndex,
                                     path_fragment_direction_t direction,
                                     local_path_fragment_t& fragment)
{
    // Search through the path fragments to find one with the specified index and direction. That fragment becomes
    // the start index. currentIndex is the same as startIndex to begin with. next() increases the currentIndex
    // BEFORE determining the fragment.

    for (size_t n = 0; n < fragments.size(); ++n) {
        if ((fragments[n].pathId == pathIndex) && (fragments[n].direction == direction)) {
            fragment = fragments[n];
            return true;
        }
    }

    return false;
}

}   // namespace hssh
}   // namespace vulcan
