/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_DISJOINT_SET_FOREST_H
#define UTILS_DISJOINT_SET_FOREST_H

#include <cstddef>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
 * DisjointSetForest is a data structure that maintains a collection of data, where
 * each member of the collection belongs to a single set amongst a forest of sets.
 * This implementation is based on the description of the disjoint-set forest with
 * "union by rank" and "path compression" described in Chapter 21.3 of CLRS.
 *
 * This current implementation of only supports a fixed-size collection of sets.
 * The MAKE_SET operation is not supported.
 *
 * From Chapter 21.1 of CLRS:
 *
 * A disjoint-set data structure maintains a collection S = {s1, s1, ..., sk} of disjoint
 * dynamic sets. Each set is identified by a representative, which is some member of
 * the set.
 */
class DisjointSetForest
{
public:
    /**
     * Constructor for DisjointSetForest.
     *
     * The forest is initialized such that each element from [0, size) is its own
     * set in the forest. UNION operations can then be used to merge connected sets.
     *
     * \param    size            Size of the collection of data
     */
    DisjointSetForest(std::size_t size);

    /**
     * setUnion merges two sets into a single set. Afterward, all elements in set y will
     * belong to set x.
     *
     * x and y only need to be members of sets, not the representatives. The appropriate
     * parents of the sets will be located and merged appropriately.
     *
     * \pre  x and y are valid members of the forest. x < size, y < size
     * \return   The representative of the merged set.
     */
    unsigned int setUnion(unsigned int x, unsigned int y);

    /**
     * findSet determines the set of which x is a member.
     *
     * \pre  x is a valid member of the forest. x < size
     * \return   The representative of the set for which x is a member.
     */
    unsigned int findSet(unsigned int x) const;

    /**
     * size retrieves the overall size of the forest as specified when it was created.
     */
    std::size_t size(void) const { return forestSize; }

    /**
     * reset resets the state of the forest so that each element belongs to its own
     * tree, as is the case after initial construction.
     */
    void reset(void);

private:
    // Disable copying
    DisjointSetForest(const DisjointSetForest& set) = delete;
    void operator=(const DisjointSetForest& rhs) = delete;

    struct set_element_t
    {
        unsigned int parent;
        unsigned int rank;
    };

    unsigned int link(unsigned int x, unsigned int y);

    // Using an array here as performance is really important and the size is fixed so
    // there are no concerns about reallocation
    mutable std::vector<set_element_t> forest;
    //     mutable set_element_t* forest;
    std::size_t forestSize;
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_DISJOINT_SET_FOREST_H
