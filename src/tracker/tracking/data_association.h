/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     data_association.h
* \author   Collin Johnson
* 
* Declaration of DataAssociationStrategy interface and create_data_association_strategy factory.
*/

#ifndef TRACKER_TRACKING_DATA_ASSOCIATION_H
#define TRACKER_TRACKING_DATA_ASSOCIATION_H

#include "tracker/tracking/object.h"
#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }
namespace tracker
{
    
class DataAssociationStrategy;
    
/**
* create_data_association_strategy is a factory function to create instances of DataAssociationStrategy.
* 
* \param    type            Type of the strategy subclass to instantiate
* \param    config          Config file with the parameters for the various strategies
* \pre  type is a valid type string for a subclass of DataAssociationStrategy
* \return   An instance of a DataAssociationStrategy subclass.
*/
std::unique_ptr<DataAssociationStrategy> create_data_association_strategy(const std::string& type,
                                                                          const utils::ConfigFile& config);


/**
* object_association_t defines the association between two objects. Each method for association matches a query object
* with a single object in another collection of objects. The association is an index into the collection along with
* a score value, which can be used to disambiguate matches.
*/
struct object_association_t
{
    int index;      ///< Index of the associated object in the collection
    double score;   ///< Score indicating how similar the object is. Lower is better. 0 is the lowest score.
    
    explicit object_association_t(int index = -1, double score = std::numeric_limits<double>::max())
    : index(index)
    , score(score)
    {
    }
};

/**
* DataAssociationStrategy is an interface that defines the data association approach used in the TrackingObjectSet. The
* data association takes in a collection of TrackingObjects and a LaserObject and finds which TrackingObject best
* matches the LaserObject. No match is a valid answer as well.
* 
* The DataAssociationStrategy currently supports only single hypothesis data association. The overall framework is not
* multihypothesis right now, so it's suitable.
* 
* Implementers of the DataAssociationStrategy need to override one method:
* 
*       TrackingObject* associateLaserWithTracked(LaserObject, TrackingObjectCollection);
*/
class DataAssociationStrategy
{
public:
    
    virtual ~DataAssociationStrategy(void) { }

    /**
    * associateLaserWithTracked finds the association between a newly measured LaserObject and an existing collection
    * of TrackingObjects.
    * 
    * Only a single association between objects is made, and no match is a perfectly reasonable result.
    * 
    * \param    laser           Newly measured LaserObject providing data on an object in the world
    * \param    objects         Current collection of objects being tracked by the particular laser
    * \return   Best association for the laser object with one of the tracked objects.
    */
    virtual object_association_t associateLaserWithTracked(const LaserObject& laser, 
                                                           const TrackingObjectCollection& objects) = 0;
                                        
    /**
    * associateObjectWithTracked finds which object in a collection of tracked objects that best matches the provided
    * object.
    * 
    * \param    object          Object to match with an object in the tracked collection
    * \param    objects         Current collection of tracked objects
    * \return   Best association between the objects and the objects in the collection.
    */
    virtual object_association_t associateObjectWithTracked(const TrackingObject& object,
                                                            const TrackingObjectCollection& objects) = 0;
                                        
    /**
    * clone creates a deep copy of the instance of DataAssociationStrategy.
    */
    virtual std::unique_ptr<DataAssociationStrategy> clone(void) = 0;
};

}
}

#endif // TRACKER_TRACKING_DATA_ASSOCIATION_H
