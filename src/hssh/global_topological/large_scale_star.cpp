/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     large_scale_star.cpp
 * \author   Collin Johnson
 *
 * Definition of LargeScaleStar.
 */

#include "hssh/global_topological/large_scale_star.h"
#include "hssh/local_topological/small_scale_star.h"
#include <cassert>
#include <iostream>

// #define DEBUG_POTENTIAL_SEGMENTS

namespace vulcan
{
namespace hssh
{

bool star_rotations_match(const std::vector<global_path_fragment_t>& lhs,
                          const std::vector<global_path_fragment_t>& rhs,
                          size_t rotationOffset);

LargeScaleStar::LargeScaleStar(void)
{
}


LargeScaleStar::LargeScaleStar(const SmallScaleStar& smallStar)
{
    // There is a one-to-one mapping between the small-scale star and large-scale star. Iterate through the local path
    // fragments and create a corresponding global star segment for each.
    const std::vector<hssh::local_path_fragment_t>& starFragments = smallStar.getAllFragments();

    for (size_t n = 0; n < starFragments.size(); ++n) {
        fragments.push_back(global_path_fragment_t(n, starFragments[n].pathId, starFragments[n].navigable));
    }
}


LargeScaleStar::LargeScaleStar(const SmallScaleStar& smallStar, const local_path_fragment_t& entryFragment)
{
    // There is a one-to-one mapping between the small-scale star and large-scale star. Iterate through the local path
    // fragments and create a corresponding global star segment for each.
    const std::vector<hssh::local_path_fragment_t>& starFragments = smallStar.getAllFragments();

    for (size_t n = 0; n < starFragments.size(); ++n) {
        fragments.push_back(global_path_fragment_t(n, starFragments[n].pathId, starFragments[n].navigable));

        if (starFragments[n] == entryFragment) {
            this->entryFragment = fragments.back();
        }
    }
}


LargeScaleStar::LargeScaleStar(const std::vector<global_path_fragment_t>& fragments) : fragments(fragments)
{
}


bool LargeScaleStar::operator==(const LargeScaleStar& rhs) const
{
    // For two large-scale stars to be the same, they must have the same number of path fragments and sequence
    // of navigable paths must be the same when moving clockwise around the star for some starting point
    // in each star's fragments
    if (fragments.size() == rhs.fragments.size()) {
        bool equal = false;

        for (size_t startIndex = 0; startIndex < rhs.fragments.size() && !equal; ++startIndex) {
            equal = star_rotations_match(fragments, rhs.fragments, startIndex);
        }

        return equal;
    }

    return false;
}


bool LargeScaleStar::operator!=(const LargeScaleStar& rhs) const
{
    return !this->operator==(rhs);
}


global_path_fragment_t LargeScaleStar::getPathFragment(int pathId, path_direction_t direction) const
{
    for (auto fragmentIt = fragments.begin(), fragmentEnd = fragments.end(); fragmentIt != fragmentEnd; ++fragmentIt) {
        if ((fragmentIt->pathId == pathId) && (fragmentIt->direction == direction)) {
            return *fragmentIt;
        }
    }

    return global_path_fragment_t();
}


global_path_fragment_t LargeScaleStar::getFragmentWithId(int8_t fragmentId) const
{
    if ((fragmentId >= 0) && (static_cast<size_t>(fragmentId) < fragments.size())) {
        return fragments[fragmentId];
    } else {
        std::cerr << "WARNING:LargeScaleStar:getFragmentWithId():Attempted to access invalid star segment:"
                  << static_cast<int>(fragmentId) << '\n';
        return global_path_fragment_t();
    }
}


global_path_fragment_t LargeScaleStar::getOtherFragmentOnPath(int8_t fragmentId) const
{
    if ((fragmentId >= 0) && (static_cast<size_t>(fragmentId) < fragments.size())) {
        return fragments[(fragmentId + fragments.size() / 2) % fragments.size()];
    } else {
        std::cerr << "WARNING:LargeScaleStar:getOtherFragmentOnPath():Attempted to access invalid star segment:"
                  << static_cast<int>(fragmentId) << '\n';
        return global_path_fragment_t();
    }
}


void LargeScaleStar::setEntryPathFragment(int8_t fragmentId)
{
    if ((fragmentId >= 0) && (static_cast<size_t>(fragmentId) < fragments.size())) {
        entryFragment = fragments[fragmentId];
    }
}


SmallScaleStar LargeScaleStar::toSmallScaleStar(void) const
{
    // The only difference between the global_path_fragment_t and the local_path_fragment_t is the pathId. For the
    // global fragment, id is the path in the global map. For the local fragment, the id is arbitrary and only needs to
    // be the same as the other fragment on the given path.

    std::vector<local_path_fragment_t> smallFragments;

    int numPaths = fragments.size() / 2;

    for (size_t n = 0; n < fragments.size(); ++n) {
        local_path_fragment_t localFragment;

        localFragment.fragmentId = fragments[n].fragmentId;
        localFragment.pathId = n % numPaths;
        localFragment.direction = (fragments[n].direction == PATH_PLUS) ? PATH_FRAGMENT_PLUS : PATH_FRAGMENT_MINUS;
        localFragment.navigable = fragments[n].navigable;

        smallFragments.push_back(localFragment);
    }

    return SmallScaleStar(smallFragments);
}


global_path_fragment_t LargeScaleStar::findExitFragment(const SmallScaleStar& smallStar,
                                                        const local_path_fragment_t& entry,
                                                        const local_path_fragment_t& exit) const
{
    // The ordering of the small-scale star and large-scale star are the same, so iterate around the small-star until
    // the entry fragment was found
    const std::vector<hssh::local_path_fragment_t>& localFragments = smallStar.getAllFragments();

    int entryIndex = 0;
    int exitIndex = 0;

    for (int n = localFragments.size(); --n >= 0;) {
        if (localFragments[n] == entry) {
            entryIndex = n;
        } else if (localFragments[n] == exit) {
            exitIndex = n;
        }
    }

    int fragmentId = entryFragment.fragmentId + exitIndex - entryIndex;

    if (fragmentId < 0) {
        fragmentId += localFragments.size();
    } else if (fragmentId >= static_cast<int>(localFragments.size()))   // take care of compiler warning
    {
        fragmentId -= localFragments.size();
    }

    global_path_fragment_t exitGlobalFragment = fragments[fragmentId];

    std::cout << "Small:Entry:" << entryIndex << " Exit:" << exitIndex
              << " Large:Entry:" << (int)entryFragment.fragmentId << " Exit:" << (int)exitGlobalFragment.fragmentId
              << '\n';

    if (!exitGlobalFragment.navigable) {
        std::cerr << "ERROR:LargeScaleStar:Exited star through non-navigable segment. Entry star:"
                  << (int)entryFragment.fragmentId << " Small entry:" << entryIndex << " Small exit:" << exitIndex
                  << '\n';

        std::cerr << "Small star:";
        for (size_t n = 0; n < localFragments.size(); ++n) {
            std::cout << localFragments[n].navigable << ' ';
        }

        std::cerr << "\nLarge star:";
        for (auto fragmentIt = fragments.begin(), fragmentEnd = fragments.end(); fragmentIt != fragmentEnd;
             ++fragmentIt) {
            std::cerr << fragmentIt->navigable << ' ';
        }
        std::cerr << '\n';
    }

    return exitGlobalFragment;
}


void LargeScaleStar::assignPath(int8_t fragmentId, int pathId, path_direction_t direction)
{
    if ((fragmentId >= 0) && (static_cast<size_t>(fragmentId) < fragments.size())) {
        fragments[fragmentId].pathId = pathId;
        fragments[fragmentId].direction = direction;
    }
}


void LargeScaleStar::changePath(int currentId, int newId, bool reverseDirection)
{
    for (auto segmentIt = fragments.begin(), segmentEnd = fragments.end(); segmentIt != segmentEnd; ++segmentIt) {
        if (segmentIt->pathId == currentId) {
            segmentIt->pathId = newId;
            segmentIt->direction = reverseDirection ? opposite_direction(segmentIt->direction) : segmentIt->direction;
        }
    }
}


void LargeScaleStar::setFragmentExploration(int8_t fragmentId, path_fragment_exploration_t exploration)
{
    if ((fragmentId >= 0) && (static_cast<size_t>(fragmentId) < fragments.size())) {
        fragments[fragmentId].exploration = exploration;
    }
}


bool LargeScaleStar::enteredOnKnownPath(const LargeScaleStar& largeStar, global_path_fragment_t& segment) const
{

    return false;
}


std::vector<global_path_fragment_t>
  LargeScaleStar::potentialEntryFragments(const LargeScaleStar& largeStar,
                                          const global_path_fragment_t& entryFragment) const
{
    std::vector<global_path_fragment_t> potentialFragments;

    if (fragments.size() != largeStar.fragments.size()) {
        return potentialFragments;
    }

    // If the rotation between the two stars is valid and the corresponding segment in this star is a frontier, then
    // entryFragment could potentially be connected to this->segment
    for (size_t n = 0; n < fragments.size(); ++n) {
        if (star_rotations_match(largeStar.fragments, fragments, n)
            && (fragments[(entryFragment.fragmentId + n) % fragments.size()].exploration == PATH_FRAGMENT_FRONTIER)) {
            potentialFragments.push_back(fragments[(entryFragment.fragmentId + n) % fragments.size()]);

#ifdef DEBUG_POTENTIAL_SEGMENTS
            std::cout << "DEBUG:LargeScaleStar:Potential segment(Frag=path:dir):"
                      << potentialFragments.back().fragmentId << '=' << potentialFragments.back().pathId << ':'
                      << potentialFragments.back().direction << '\n';
#endif
        }
    }

    return potentialFragments;
}


bool LargeScaleStar::areStarsCompatible(const LargeScaleStar& star) const
{
    // Check the topologies before investigating further
    if (*this != star) {
        return false;
    }

    // See if there is some valid rotation of the stars such that the entry fragments are the same
    for (std::size_t n = 0; n < fragments.size(); ++n) {
        int otherSegmentId = (star.entryFragment.fragmentId + n) % fragments.size();
        // If the star rotation matches, then these are equivalent topologies. Check if the rotated entry segment from
        // the incoming star has the same segment id as the entry segment for this star. If so, then these stars have
        // equivalent paths
        if (star_rotations_match(star.fragments, fragments, n) && (otherSegmentId == entryFragment.fragmentId)) {
            return true;
        }
    }

    return false;
}


std::ostream& operator<<(std::ostream& out, const LargeScaleStar& star)
{
    const std::vector<global_path_fragment_t>& fragments = star.getPathFragments();

    for (auto fragmentIt = fragments.begin(), fragmentEnd = fragments.end(); fragmentIt != fragmentEnd; ++fragmentIt) {
        std::cout << '(' << fragmentIt->pathId << ',' << fragmentIt->direction << ',' << fragmentIt->navigable << ") ";
    }

    return out;
}


bool star_rotations_match(const std::vector<global_path_fragment_t>& lhs,
                          const std::vector<global_path_fragment_t>& rhs,
                          size_t rotationOffset)
{
    // PRE: lhs.size() == rhs.size()

    size_t n = 0;
    while ((n < lhs.size()) && (lhs[n].navigable == rhs[(n + rotationOffset) % rhs.size()].navigable)) {
        ++n;
    }

    // If made it all the way through the fragments, then a match was found for the sequence
    return n == lhs.size();
}

}   // namespace hssh
}   // namespace vulcan
