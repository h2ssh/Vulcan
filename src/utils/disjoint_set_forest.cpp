/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     disjoint_set_forest.cpp
* \author   Collin Johnson
* 
* Definition of DisjointSetForest.
*/

#include <utils/disjoint_set_forest.h>
#include <cassert>

namespace vulcan
{
namespace utils
{

/*
* NOTE: The implementation for all of this code is direct from the pseudocode in CLRS 21.3.
*       See the book for details. Pretty straightforward overall.
*/

DisjointSetForest::DisjointSetForest(std::size_t size)
: forest(size)
, forestSize(size)
{
//     forest = new set_element_t[forestSize];
    
    reset();
}


unsigned int DisjointSetForest::setUnion(unsigned int x, unsigned int y)
{
    assert(x < forestSize && y < forestSize);    
    return link(findSet(x), findSet(y));
}


unsigned int DisjointSetForest::findSet(unsigned int x) const
{
    assert(x < forestSize);
    
    // The path compression heuristic flattens the tree every time a
    // search is made, making searches take shorter and shorter time
    
    // Find the top of the set, then copy that parent all the way down
    // the tree
    if(forest[x].parent != x)
    {
        forest[x].parent = findSet(forest[x].parent);
    }
    
    return forest[x].parent;
}


unsigned int DisjointSetForest::link(unsigned int x, unsigned int y)
{
    // Attached the smaller set to the larger set
    if(forest[x].rank > forest[y].rank)
    {
        forest[y].parent = x;
        
        return x;
    }
    else
    {
        forest[x].parent = y;
        
        // Only need to update rank if they are the same.
        // Otherwise, larger tree rank still describes the combined
        // tree
        if(forest[x].rank == forest[y].rank)
        {
            ++forest[y].rank;
        }
        
        return y;
    }
}


void DisjointSetForest::reset(void)
{
    // Initialize all sets to size 1, containing only themselves
    for(size_t n = 0; n < forestSize; ++n)
    {
        forest[n].parent = n;
        forest[n].rank   = 0;
    }
}

} // namespace utils
} // namespace vulcan
