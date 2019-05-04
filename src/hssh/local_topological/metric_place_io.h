/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_place_io.h
* \author   Collin Johnson
*
* Declaration of MetricPlaceIO interface and create_metric_place_io factory function.
*/

#ifndef HSSH_UTILS_METRIC_PLACE_IO_H
#define HSSH_UTILS_METRIC_PLACE_IO_H

#include <boost/shared_ptr.hpp>

namespace vulcan
{
namespace hssh
{

class LocalPlaceOld;
class MetricPlaceIO;

/**
* create_metric_place_io creates the necessary classes for doing reading and writing of metric
* places.
*
* \param    type            Type of MetricPlaceWriter/Reader to create
*/
boost::shared_ptr<MetricPlaceIO> create_metric_place_io(const std::string& type,
                                                        const std::string& directory,
                                                        const std::string& basename);

/**
* MetricPlaceIO is an interface for classes capable of writing LocalPlaceOlds to disk and
* reading LocalPlaceOlds from disk.
*
* The interface is two methods:
*
*   - LocalPlaceOld read(int placeId);
*   - void       write(const LocalPlaceOld& place)
*/
class MetricPlaceIO
{
public:

    virtual ~MetricPlaceIO(void) {}

    /**
    * write saves the provided place on the robot's hard disk.
    *
    * \param    place       Place to be saved
    */
    virtual void write(const LocalPlaceOld& place) = 0;

    /**
    * read loads the place with the given ID from the robot's hard disk.
    *
    * \param   placeId     Id of the place to be loaded
    * \return  LocalPlaceOld with the given id.
    */
    virtual LocalPlaceOld read(int placeId) = 0;
};

}
}

#endif // HSSH_UTILS_METRIC_PLACE_IO_H
