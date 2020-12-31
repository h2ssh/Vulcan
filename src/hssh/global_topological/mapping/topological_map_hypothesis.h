/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map_hypothesis.h
* \author   Collin Johnson
*
* Declaration of TopologicalMapHypothesis, hypothesis_probability_t, and place_connection_t.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TOPOLOGICAL_MAP_HYPOTHESIS_H
#define HSSH_GLOBAL_TOPOLOGICAL_TOPOLOGICAL_MAP_HYPOTHESIS_H

#include "hssh/global_topological/topological_map.h"
#include "hssh/global_topological/chi.h"
#include <boost/shared_ptr.hpp>
#include <set>

namespace vulcan
{
namespace hssh
{

class TopologicalMap;
class TopologicalMapHypothesis;

typedef boost::shared_ptr<TopologicalMapHypothesis> TopoMapPtr;

/**
* place_connection_t links a Place with the global_path_fragment_t through which the most
* recent event at the Place occurred. The event is either entering or exiting and can be
* interpreted as either, depending on the particular use case.
*/
struct place_connection_t
{
    int                    placeId;
    global_path_fragment_t eventFragment;
};

/**
* TopologicalMapHypothesis represents a specific instance of a topological map. A topological map
* hypothesis includes the places, paths, and position of the robot. Each map hypothesis is a complete
* topological map. The identifiers for places and paths are unique for each hypothesis. Therefore,
* hypotheses cannot be compared based on the id of the places and paths, but rather their contents.
*/
class TopologicalMapHypothesis : public TopologicalMap
{
public:

    /**
    * Constructor for TopologicalMapHypothesis.
    *
    * Create a new map hypothesis based on being initially located at some place.
    *
    * \param    initialPlace        Initial place in the map hypothesis
    */
    TopologicalMapHypothesis(const GlobalPlace& initialPlace);

    /**
    * Constructor for TopologicalMapHypothesis.
    *
    * Create a new map hypothesis from an existing TopologicalMap. This constructor allows
    * a hypothesis to be created from a TopologicalMap loaded from a file.
    *
    * \param    map         Existing TopologicalMap to be converted into a TopologicalMapHypothesis
    */
    TopologicalMapHypothesis(const TopologicalMap& map);

    /**
    * Copy constructor for TopologicalMapHypothesis.
    */
    TopologicalMapHypothesis(const TopologicalMapHypothesis& toCopy);

    // Accessors
    const Chi& getChi(void)                   const { return chi; }
    double     getLogPosterior(void)          const { return probability.logPosterior; }
    double     getEstimatedLogPosterior(void) const { return probability.estimatedLogPosterior; }

    bool wasPruned(void) const { return pruned; }              // check validity of this hypothesis, see if it still exists in tree

    /**
    * shouldOptimize checks to see if the current hypothesis needs to have a new Chi calculated via
    * optimization. The Chi only needs to be optimized if a new loop has been closed.
    */
    bool shouldOptimize(void) const { return needOptimization; }

    /**
    * hasPlace checks to see if a place with the given id exists in this map hypothesis.
    *
    * \param    id              Id of place to query
    * \return   True if such a place exists. False otherwise.
    */
    bool hasPlace(int id) const { return places.find(id) != places.end(); }

    /**
    * hasPath checks to see if a path with the given id exists in this map hypothesis.
    *
    * \param    id              If of the path to query
    * \return   True if such a path exists. False otherwise.
    */
    bool hasPath(int id) const { return paths.find(id) != paths.end(); }

    /**
    * getPlace retrieves the Place with the specified id.
    *
    * NOTE: If no place with the id exists, expect a crash.
    *
    * \param    placeId         Id of the Place to be retrieved
    * \return   Place in the map with the given id.
    */
    GlobalPlace&       getPlace(int placeId);
    const GlobalPlace& getPlace(int placeId) const;

    /**
    * getPath retrieves the Path with the specified id.
    *
    * * NOTE: If no path with the id exists, expect a crash.
    *
    * \param    pathId          Id of the Path to be retrieved
    * \return   Path with the desired id
    */
    GlobalPath&       getPath(int pathId);
    const GlobalPath& getPath(int pathId) const;

    // Mutators
    /**
    * setGlobalLocation sets the global location of the robot in the current hypothesis. The global location can change without
    * a hypothesis branching.
    */
    void setGlobalLocation(const GlobalLocation& newLocation);

    /**
    * setChi sets the Chi value for the current hypothesis. Chi provides a layout for the places in the
    * map in a single reference frame.
    */
    void setChi(const Chi& newChi);

    /**
    * setProbability sets the probability values for this hypothesis.
    */
    void setProbability(const hypothesis_probability_t& probability) { this->probability = probability; }

    // Operations
    /**
    * toSingleReference converts the hypothesis map into a map where all places are fixed in a single,
    * absolute reference frame, as opposed to the relative relations in this map.
    */
    TopologicalMap toSingleReference(void) const;

private:

    TopologicalMapHypothesis& operator=(const TopologicalMapHypothesis& rhs) = delete;

    // INVARIANT: A hypothesis with 0 children is a leaf. Only leaf hypotheses are revealed to the outside world.

    friend class TreeOfMaps;
    friend class TopologicalMapEditor;

    // Moving to a known place
    void updateLambdaOnLastPathSegment(const Lambda& lambda);

    // Adding a new place
    int               addNewPlace        (GlobalPlace& newPlace, unsigned int entrySegmentId, const Lambda& lambda);
    path_direction_t  createPathsForPlace(GlobalPlace& newPlace, int entrySegmentId, const Lambda& lambda);
    path_transition_t addPlaceToExistingPath(GlobalPath&                   path,
                                             GlobalPlace&                  newPlace,
                                             const global_path_fragment_t& entrySegment,
                                             const global_path_fragment_t& otherSegment,
                                             const Lambda&                 lambda,
                                             path_direction_t              direction);
    int createNewPath(GlobalPlace& newPlace, const global_path_fragment_t& plusFragment, const global_path_fragment_t& minusFragment);

    // Create a new loop closure
    void connectPlaces                (const place_connection_t& to, const place_connection_t& from, const Lambda& lambda, const pose_t& transform);
    void connectPlacesOnDifferentPaths(const place_connection_t& to, const place_connection_t& from, const Lambda& lambda);
    void connectPlacesOnSamePath      (const place_connection_t& to, const place_connection_t& from, const Lambda& lambda);
    void mergePaths                   (GlobalPath&              source,
                                       GlobalPath&              destination,
                                       const path_transition_t& mergeTransition,
                                       path_direction_t         sourceDirection,
                                       path_direction_t         destinationDirection);
    void renamePath                   (GlobalPath& oldPath, int newId, bool reverseDirection);

    void setReferenceFramesBasedOnChi(void);

    Chi  chi;
    bool needOptimization;      // flag indicating if the current Chi is insufficient because a new loop closure was added

    int nextPlaceId;
    int nextPathId;

    TopoMapPtr              parent;
    std::vector<TopoMapPtr> children;

    bool pruned;        // flag indicating if the parent has been pruned away
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_TOPOLOGICAL_MAP_HYPOTHESIS_H
