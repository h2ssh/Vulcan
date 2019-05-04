/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map.h
* \author   Collin Johnson
*
* Declaration of TopologicalMap.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TOPOLOGICAL_MAP_H
#define HSSH_GLOBAL_TOPOLOGICAL_TOPOLOGICAL_MAP_H

#include <hssh/global_topological/area.h>
#include <hssh/global_topological/chi.h>
#include <hssh/global_topological/global_location.h>
#include <hssh/global_topological/global_path.h>
#include <hssh/global_topological/global_path_segment.h>
#include <hssh/global_topological/global_place.h>
#include <hssh/global_topological/transition.h>
#include <hssh/utils/id.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <boost/optional.hpp>
#include <memory>

namespace vulcan
{
namespace hssh
{

class TopologicalVisit;
class TopologicalMap;

/**
* TopologicalMap is the base class for various representations of a global topological map. A topological map
* consists of a set of unique areas, along with a set of frontier transitions indicating the unexplored parts of the
* map.
*
* A TopologicalMap is an immutable data structure. When adding areas or closing loops, a new map will be generated
* rather than modifying the current map. This approach makes maintenance of child hypotheses much simpler at the cost
* of some extra RAM for storing duplicates of various parts of the map.
*
* Aside from accessors for stored properties of the map (areas + transitions), the map can be manipulated in three ways:
*
*   - addArea : adds a new area to the map and connects it to an existing area via the specified transition
*   - closeLoop : establishes that two areas in the map actually share a transition and marks the transition accordingly
*   - updateArea : the robot revisited an area, so more information about the contents (path lambda, metric map, etc.)
*                   is known and hsould be incorporated into the map.
*/
class TopologicalMap
{
public:

    // Order of place and segment maps is important because Ids monotonically increase, we want to be able to always
    // iterate through the areas in order of oldest to newest.
    // This invariant is important for getting consistent computations of Chi.
    using PlaceMap = std::map<Id, GlobalPlace::Ptr>;
    using PathMap = std::unordered_map<Id, GlobalPath::Ptr>;
    using SegmentMap = std::map<Id, GlobalPathSegment::Ptr>;
    using Ptr = std::shared_ptr<TopologicalMap>;
    using FrontierIter = std::vector<GlobalTransition>::const_iterator;

    /**
    * CreateTopologicalMap creates a new TopologicalMap using the provided visit. The returned map is both the map
    * and the robot location within this initial map.
    *
    * \param    initialVisit            Initial visit from which to generate the topological map
    * \return   The TopologicalMap and GlobalLocation of the robot based on the initial visit.
    */
    static std::pair<Ptr, GlobalLocation> CreateTopologicalMap(const TopologicalVisit& initialVisit);

    /**
    * Default constructor for TopologicalMap.
    */
    TopologicalMap(void) = default;

    /**
    * Copy constructor for TopologicalMap.
    */
    TopologicalMap(const TopologicalMap& toCopy) = default;

    ///////////// Accessors ///////////////

    /**
    * id retrieves the unique identifier for this topological map.
    */
    Id id(void) const { return id_; }

    ///// Accessors for each type of area stored in the map //////
    std::size_t numPlaces(void) const { return places_.size(); }
    std::size_t numSegments(void) const { return segments_.size(); }

    const PlaceMap& places(void) const { return places_; }
    const PathMap& paths(void) const;
    const SegmentMap& segments(void) const { return segments_; }

    ///// Accessors to get a particular area by id ///////

    /**
    * hasArea checks if an area with the given id exists in the map.
    *
    * \param    id          Id to check for existence in the map
    * \return   True if an area with the given id does exist.
    */
    bool hasArea(Id id) const;

    /**
    * getDestination retrieves the destination with the specified id from the map.
    *
    * \param    id          Id of the destination
    * \return   A pointer to the destination with the given id. nullptr if no such destination exists.
    */
    const GlobalPlace* getDestination(Id id) const;

    /**
    * getDecisionPoint retrieves the decision point with the specified id from the map.
    *
    * \param    id          Id of the decision point
    * \return   A pointer to the decision point with the given id. nullptr if no such decision point exists.
    */
    const GlobalPlace* getDecisionPoint(Id id) const;

    /**
    * getPlace retrieves the place with the specified id from the map.
    *
    * \param    id          Id of the place
    * \return   A pointer to the place with the given id. nullptr if no such place exists.
    */
    const GlobalPlace* getPlace(Id id) const;

    /**
    * getPathSegment retrieves the path segment with the specified id.
    *
    * \param    id              Id of the path segment
    * \return   A pointer to the path segment with the given id. nullptr if no such path segment exists.
    */
    const GlobalPathSegment* getPathSegment(int id) const;

    ///// Iterators over all frontiers in the map ////

    std::size_t sizeFrontiers(void) const { return frontiers_.size(); }
    FrontierIter beginFrontiers(void) const { return frontiers_.begin(); }
    FrontierIter endFrontiers(void) const { return frontiers_.end(); }
    GlobalTransition frontierAt(int index) const { return frontiers_.at(index); }

    //////////  Map generators ////////////////

    /**
    * addArea adds a new area to the map. The area is added the the specified location. The visit describes the area
    * that is being added. The TopologicalMap will construct the appropriate place from the visit, ground it in the
    * global map symbols, and connect it to the existing areas as needed.
    *
    * \pre  location has the robot at a frontier area. Areas can't be added at non-frontier locations.
    * \param    location            Location of the robot, which is where the area is added
    * \param    visit               Visit describing the area being added
    * \return   A new TopologicalMap contains the new area along with the equivalent location in the new map to the
    *   location in the parent map. nullptr if the area can't be added for some reason.
    */
    std::pair<Ptr, GlobalLocation> addArea(const GlobalLocation& location, const TopologicalVisit& visit) const;

    /**
    * closeLoop closes a loop in the map by asserting that two transitions are the same transition. Making this
    * assertion then establishes equality between the areas on the either side of the transition. The provided visit
    * is the new area that was detected that resulted in the asserted loop closure.
    *
    * One transition is the entryTransition for the current location. The other transition is provided and is
    * determined by whatever method is searching for valid loop closures.
    *
    * \param    location            Location of the robot in the map. This location will be considered a frontier
    * \param    loopArea            Area that we are asserting is the same as the area defined by location
    * \param    loopTrans           Transition for which the assertion location.entryTranstion == loopTrans
    * \param    visit               The visit that resulted in the loop closure being made
    * \return   A new TopologicalMap that asserts the loop closure between mapTrans and visitTrans and the equivalent
    *   location in the new map to the location in the parent map. nullptr if the loop can't be created.
    */
    std::pair<Ptr, GlobalLocation> closeLoop(const GlobalLocation& location,
                                             const GlobalArea& loopArea,
                                             const GlobalTransition& loopTrans,
                                             const TopologicalVisit& visit) const;

    /**
    * revisitArea incorporates new information into an existing area in the map. The new information is available in
    * the provided visit.
    *
    * \param    location            Location of the robot
    * \param    visit               Visit containing new information about an existing place in the map
    * \return   A new map with additional information about the location added. nullptr if no such area exists.
    */
    Ptr revisitArea(const GlobalLocation& location, const TopologicalVisit& visit) const;


    /////////////  Map mutators  /////////////////////////

    /**
    * incorporateVisit provides additional information about location when the robot revisits or leaves alocation. At
    * the time of exit, the robot has as much information about the area as it will gain because it has been constantly
    * adding additional information during the time of the visit.
    *
    * \param    location            Location to be updated
    * \param    visit               Visit at the time of exit containing the most complete knowledge of the area
    */
    void incorporateVisit(const GlobalLocation& location, const TopologicalVisit& visit);

    /**
    * setReferenceFrames sets the origin of the local reference frame for every place based on the values computed
    * from Chi.
    *
    * \param    chi         Chi value containing the reference frame for places in the map
    */
    void setReferenceFrames(const Chi& chi);

    /**
    * referenceFrame retrieves the reference frame for the desired area.
    *
    * \param    id          Id of the area
    * \return   Computed reference frame, as determined by the Chi value. (0, 0, 0) if no such area.
    */
    pose_distribution_t referenceFrame(Id id) const;

    /**
    * chiLogLikelihood computes the log-likelihood of the place layout for this map.
    */
    double chiLogLikelihood(void) const { return chi_.getLogLikelihood(); }

private:

    Id id_ = kInvalidId;

    std::unordered_map<Id, GlobalArea> areas_;
    PlaceMap places_;
    SegmentMap segments_;
    PathMap paths_;     // lazily created only if requested -- not maintained during SLAM
    Chi chi_;           // reference frame of the places
    std::vector<GlobalTransition> frontiers_;

    // Private constructors
    TopologicalMap(Id id)
    : id_(id)
    {
    }

    // Return location in new map corresponding to location in the old map
    GlobalLocation addConcreteArea(const GlobalLocation& location, const TopologicalVisit& visit);
    void addPathSegmentFrontiers(const GlobalPathSegment& segment);
    void addPlaceFrontiers(const GlobalPlace& place);

    void updatePathSegment(const GlobalLocation& location, const TopologicalVisit& visit);
    void updatePlace(const GlobalLocation& location, const TopologicalVisit& visit);

    void updateTransition(const GlobalTransition& oldTrans, const GlobalTransition& newTrans);
    GlobalLocation locationInMap(const GlobalArea& area, const GlobalTransition& entry) const;

    bool replaceTransitionForArea(const GlobalArea& area,
                                  const GlobalTransition& oldTrans,
                                  const GlobalTransition& newTrans);
    // Find transform from the other end of the exited area to the exit transition area -- just apply the lambda
    pose_distribution_t estimateReferenceFrame(const GlobalArea& exitedArea,
                                                      const GlobalTransition& exitTransition);
    GlobalPlace* getModifiablePlace(Id id);
    GlobalPathSegment* getModifiableSegment(Id id);

    void duplicateArea(GlobalArea area);

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( id_,
            areas_,
            places_,
            segments_,
            chi_,
//             paths_,
            frontiers_
        );
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_TOPOLOGICAL_MAP_H
