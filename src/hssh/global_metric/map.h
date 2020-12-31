/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     map.h
 * \author   Collin Johnson
 *
 * Declaration of GlobalMetricMap.
 */

#ifndef HSSH_GLOBAL_METRIC_MAP_H
#define HSSH_GLOBAL_METRIC_MAP_H

#include "hssh/metrical/occupancy_grid.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/string.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * GlobalMetricMap
 */
class GlobalMetricMap : public OccupancyGrid
{
public:
    /**
     * Default constructor for GlobalMetricMap.
     */
    GlobalMetricMap(void);

    /**
     * Constructor for GlobalMetricMap.
     *
     * Create a GlobalMetricMap via a named OccupancyGrid.
     *
     * \param    name        Name to assign the map
     * \param    map         Map to use
     */
    GlobalMetricMap(const std::string& name, const OccupancyGrid& map);

    // Other constructors should just be defaulted
    GlobalMetricMap(const GlobalMetricMap& rhs) = default;
    GlobalMetricMap(GlobalMetricMap&& rhs) = default;

    GlobalMetricMap& operator=(const GlobalMetricMap& rhs) = default;
    GlobalMetricMap& operator=(GlobalMetricMap&& rhs) = default;


    /**
     * id retrieves the unique id for the map.
     */
    int32_t id(void) const { return id_; }

    /**
     * name retrieves the name given to the map.
     */
    std::string name(void) const { return name_; }

    /**
     * saveToFile saves the map to the specified file.
     *
     * \param    filename            Name of the file in which to save the map
     * \return   True if the map is saved successfully.
     */
    bool saveToFile(const std::string& filename) const;

    /**
     * loadFromFile loads a map from the specified file. If the map can't be loaded,
     * then nothing happens.
     *
     * \param    filename            Name of the file from which to load the map
     * \return   True if the map was loaded successfully.
     */
    bool loadFromFile(const std::string& filename);

private:
    int32_t id_;
    std::string name_;

    static int32_t nextId_;

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<OccupancyGrid>(this), id_, name_);
    }
};

}   // namespace hssh
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::GlobalMetricMap, ("HSSH_GLOBAL_METRIC_MAP"))

#endif   // HSSH_GLOBAL_METRIC_MAP_H
