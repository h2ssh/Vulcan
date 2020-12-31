/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     toggle_glass_mapping.h
 * \author   Collin Johnson
 *
 * Declaration of ToggleGlassMapping, which allows turning on/off the glass mapping on-the-fly rather than only through
 * a config file.
 */

#ifndef HSSH_LOCAL_METRIC_COMMANDS_TOGGLE_GLASS_MAPPING_H
#define HSSH_LOCAL_METRIC_COMMANDS_TOGGLE_GLASS_MAPPING_H

#include "hssh/local_metric/command.h"
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * ToggleGlassMapping turns on/off the glass mapping.
 */
class ToggleGlassMapping : public LocalMetricCommand
{
public:
    /**
     * Constructor for ToggleGlassMapping.
     *
     * \param    shouldMap           Flag indicating if glass mapping will be used or not
     * \param    source              Source of the message (optional)
     */
    ToggleGlassMapping(bool shouldMap, const std::string& source = "");

    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data,
               Localizer& localizer,
               Mapper& mapper,
               MetricRelocalizer& relocalizer) const override;
    void print(std::ostream& out) const override;

private:
    bool shouldMap_;

    // Serialization support
    ToggleGlassMapping(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LocalMetricCommand>(this), shouldMap_);
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::ToggleGlassMapping)

#endif   // HSSH_LOCAL_METRIC_COMMANDS_TOGGLE_GLASS_MAPPING_H
