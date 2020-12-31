/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path.h
* \author   Collin Johnson
*
* Definition of GlobalPath and TopoDirection.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PATH_H
#define HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PATH_H

#include "hssh/global_topological/global_path_segment.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <iosfwd>
#include <memory>

namespace vulcan
{
namespace hssh
{

/**
* GlobalPath is an abstraction of a path in the world. A path consists of a series of one or more path segments
* between two places. Thus, a path joins two or more places, while a segment joins exactly two places.
*
* The base entity of a GlobalPath is a GlobalPathSegment. The GlobalPathSegment links two places via path transitions. The
* path transition gives both the place and the associated star segment. These two pieces of information are
* needed because a GlobalPath can contain a Place multiple times. If a Place appears multiple times, the path
* transitions provide the distinguishing information about which occurrence of the place is in question.
*
* At the moment, each path segment is expected to be observed in the path. This assumption is not valid
* though because the perception of the robot is such that a place could be unobserved by the robot. The
* missed observation could be caused by the presence of many temporary objects, a door being closed, etc.
* As such, a likelihood of observation will need to be attached to path segments to ensure that failing
* to observe a single place does not destroy the robot's localization.
*/
class GlobalPath
{
public:

    using Iter = std::vector<GlobalPathSegment>::const_iterator;
    using RIter = std::vector<GlobalPathSegment>::const_reverse_iterator;
    using Ptr = std::shared_ptr<GlobalPath>;

    /**
    * Constructor for GlobalPath.
    *
    * \param    id          Id of the path
    */
    GlobalPath(Id id);

    /**
    * Constructor for GlobalPath.
    *
    * Create a GlobalPath from an existing collection of places and lambdas.
    * Intended for internal use.
    *
    * \param    id                  Id of the path
    * \param    segments            Segments on the path
    */
    GlobalPath(Id id, const std::vector<GlobalPathSegment>& segments);

    // Operators for GlobalPath
    friend std::ostream& operator<<(std::ostream& out, const GlobalPath& path);

    /**
    * id retrieves the id for the path.
    */
    int id(void) const { return id_; }

    /**
    * numPlaces retrieves the number of places in the path, including frontier places.
    */
    std::size_t numPlaces(void) const;

    /**
    * places retrieves the sequentially ordered collection of place ids along the path.
    * The ordered is the same as the ordering in the path segments.
    */
    std::vector<Id> places(void) const;
    
    /**
    * pathSegments retrieves the sequentially ordered collection of path segments along the path.
    */
    const std::vector<GlobalPathSegment>& pathSegments(void) const { return segments_; }

    /**
    * getSegmentIndex retrieves the index of a path segment in the sequence of path segments
    * that comprise the GlobalPath.
    *
    * \param    segment             Segment for which to find the index
    * \return   The index of this path segment in the sequence. -1 if no such segment exists in the path.
    */
    int getSegmentIndex(const GlobalPathSegment& segment) const;

    /**
    * hasSegmentIndex checks to see if the path contains a segment with the given index.
    *
    * \param    index       Index to check
    * \return   True if such a segment exists. False otherwise.
    */
    bool hasSegmentIndex(std::size_t index) const { return index < segments_.size(); }

    /**
    * hasSegmentWithId checks to see if a path segment with the specified unique id exists
    * in the path.
    *
    * \param    id                  Id of the segment to look for
    * \return   True if such a segment exists. False otherwise.
    */
    bool hasSegmentWithId(Id id) const;

    /**
    * getSegment retrieves the segment at the specified index. If no segment exists with the given index,
    * then expect a crash.
    *
    * \param    index               Index of the segment to retrieve
    */
    GlobalPathSegment getSegment(std::size_t index) const { return segments_[index]; }

    /**
    * getSegmentWithId retrieves the segment with the specified unique id. If no segment exists, then
    * a garbage segment is returned.
    *
    * \param    id                  Unique id of the desired path segment
    */
    GlobalPathSegment getSegmentWithId(Id id) const;
    
    // Iterator support for GlobalPath -- begin->end is the plus direction. rbegin->rend is the minus direction
    std::size_t size(void) const { return segments_.size(); }
    bool empty(void) const { return segments_.empty(); }
    Iter begin(void) const { return segments_.begin(); }
    Iter end(void) const { return segments_.end(); }
    RIter rbegin(void) const { return segments_.rbegin(); }
    RIter rend(void) const { return segments_.rend(); }

private:
    
    Id id_ = kInvalidId;         // Global identifier for the path
    std::vector<GlobalPathSegment> segments_;        // Path segments along the path
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( id_,
            segments_
        );
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PATH_H
