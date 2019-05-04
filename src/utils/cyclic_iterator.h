/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     cyclic_iterator.h
* \author   Collin Johnson
* 
* Definition of cyclic_iterator class template and make_cyclic_iterator function.
*/

#ifndef UTILS_CYCLIC_ITERATOR_H
#define UTILS_CYCLIC_ITERATOR_H

#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <type_traits>

namespace vulcan
{
namespace utils
{

/**
* cyclic_iterator is an iterator that supports cycling through a range a pre-specified number of times. It is akin to 
* going through a circular buffer N times before deciding to be finished with iteration.
* 
* The end iterator for a cyclic_iterator has the same start, beginRange, and endRange, but has numCycles == 0. Once
* the number of cycles between this iterator and the other cyclic iterator match, so the end is reached.
*/
template <class Iter>
class cyclic_iterator : public boost::iterator_facade<
                            cyclic_iterator<Iter>,
                            typename std::iterator_traits<Iter>::value_type,
                            boost::forward_traversal_tag,
                            typename std::iterator_traits<const Iter>::reference
                            >
{
public:
    
    /**
    * Constructor for cyclic_iterator.
    * 
    * Creates a cyclic iterator whose starting point is start, where beginRange <= startIt < endRange. The iteration
    * will go through the cycle numCycles number of times before exiting. The cycle counter ticks up every time 
    * the current value of the iterator equals startIt.
    * 
    * \param    startIt         Starting point of the iteration
    * \param    beginRange      Begin point of the range through which to cyclically iterate
    * \param    endRange        End-plus-one of the range through which to cyclically iterate
    * \param    numCycles       Number of cycles to iterate (optional, default = 1, end iterator = 0)
    */
    cyclic_iterator(Iter startIt, Iter beginRange, Iter endRange, int numCycles = 1)
    : cyclesToGo_(numCycles)
    , begin_(beginRange)
    , end_(endRange)
    , start_(startIt)
    , it_(startIt)
    {
    }
    
private:
    
    friend class boost::iterator_core_access;
    
    // boost::iterator_facade interface
    typename std::iterator_traits<Iter>::reference dereference(void) const
    {
        return *it_;
    }
    
    void increment(void)
    {
        ++it_;
        if(it_ == end_)
        {
            it_ = begin_;
        }
        
        if(it_ == start_)
        {
            --cyclesToGo_;
        }
    }
    
    template <class OtherIter>
    bool equal(const cyclic_iterator<OtherIter>& other) const 
    {
        return (other.it_ == it_)
            && (other.cyclesToGo_ == cyclesToGo_);
    }
    
    int cyclesToGo_;
    Iter begin_;
    Iter end_;
    Iter start_;
    Iter it_;
};

/**
* make_cyclic_iterator_range creates a range that iterates through the cycle the provided number of times.
* 
* \param    start           Starting point in the range of the cycle
* \param    begin           Beginning of the underlying container
* \param    end             End of the underlying container
* \param    numCycles       Number of times to iterate through the cycle before reaching end
* \return   A pair defining the [begin, end) range for the cycle.
*/
template <class Iter>
std::pair<cyclic_iterator<Iter>, cyclic_iterator<Iter>> make_cyclic_iterator(Iter start, 
                                                                             Iter begin, 
                                                                             Iter end, 
                                                                             int numCycles = 1)
{
    using cyc_iter = cyclic_iterator<Iter>;
    return std::make_pair(cyc_iter(start, begin, end, numCycles), cyc_iter(start, begin, end, 0));
}

}
}

#endif // UTILS_CYCLIC_ITERATOR_H
