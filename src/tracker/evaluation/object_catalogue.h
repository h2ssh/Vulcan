/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_catalogue.h
* \author   Collin Johnson
*
* Declaration of ObjectCatalogue.
*/

#ifndef TRACKER_EVALUATION_OBJECT_CATALOGUE_H
#define TRACKER_EVALUATION_OBJECT_CATALOGUE_H

#include <tracker/evaluation/object_track.h>
#include <tracker/types.h>
#include <boost/optional.hpp>
#include <vector>
#include <cstdint>

namespace vulcan
{
namespace tracker
{

class DynamicObjectCollection;

/**
* ObjectCatalogue maintains a catalogue of all unique DynamicObjects observed in the environment. The catalogue creates
* an ObjectTrack for each object. See ObjectTrack for details about the per-object data that is maintained.
*
* The catalogue is used by calling the addObjects() method whenever a new DynamicObjectCollection is received.
* addObjects will update the track for existing objects and create new tracks for newly observed objects. The ObjectId
* is used for determining which track an object should be assigned to.
*
* The catalogue allows iteration over all stored tracks, along with clearing existing tracks.
*
* The catalogue expects time to move forward in small increments. The catalogue also automatically resets if time is
* detected going backward or if time jumps forward more than a configurable number of milliseconds.
*/
class ObjectCatalogue
{
public:

    // Types used for the catalogue
    using size_type = std::vector<ObjectTrack>::size_type;
    using const_iterator = std::vector<ObjectTrack>::const_iterator;

    /**
    * Constructor for ObjectCatalogue.
    *
    * \param    maxForwardTimeJumpMs        Maximum time is allowed to jump forward before the catalogue is reset
    */
    explicit ObjectCatalogue(int64_t maxForwardTimeJumpMs);

    /**
    * addObjects adds new objects to the catalogue. Those objects that already exists will have their tracks extended.
    * Those objects that are new will have new tracks initialized.
    *
    * If the time of the collection goes into the past or jumps more than some number of seconds into the future, the
    * catalogue will automatically reset itself.
    *
    * \param    objects         Objects to be added to the catalogue
    * \return   The number of new objects added.
    */
    int addObjects(const DynamicObjectCollection& objects);

    /**
    * hasTrackForObject checks to see if a track is associated with the specified ObjectId.
    *
    * \param    id          Id to check for
    * \return   True if an object is contained that has the desired id.
    */
    bool hasTrackForObject(ObjectId id) const;

    /**
    * trackForObject retrieves the track associated with the specified ObjectId if one exists.
    *
    * \param    id          Id for which to retrieve the track
    * \return   The desired track if it exists. boost::none otherwise.
    */
    boost::optional<ObjectTrack> trackForObject(ObjectId id);

    // Iteration support
    size_type size(void) const { return tracks_.size(); }
    bool empty(void) const { return tracks_.empty(); }
    void clear(void) { tracks_.clear(); }

    const_iterator begin(void) const { return tracks_.begin(); }
    const_iterator end(void) const { return tracks_.end(); }

private:

    int64_t lastUpdateTime_;
    std::vector<ObjectTrack> tracks_;

    int64_t maxForwardJumpUs_;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_EVALUATION_OBJECT_CATALOGUE_H
