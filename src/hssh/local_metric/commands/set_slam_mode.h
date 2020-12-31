/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     set_slam_mode.h
 * \author   Collin Johnson
 *
 * Declaration of SetSlamModeCommand.
 */

#ifndef HSSH_LOCAL_METRIC_COMMANDS_SET_SLAM_MODE_H
#define HSSH_LOCAL_METRIC_COMMANDS_SET_SLAM_MODE_H

#include "hssh/local_metric/command.h"
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * SlamMode defines the possible modes in which the SLAM process can run:
 *
 *   - Localization only : Only run the localization. Continue to use the LPM that existed when the command was issued.
 *   - Full SLAM         : Run both the localization and mapping. Update the map using current sensor data.
 */
enum class SlamMode
{
    localization_only,
    full_slam,
    unchanged   // Leave the mode the same as whatever it was
};

/**
 * MappingMode defines the possible modes for the mapping:
 *
 *   - scrolling : the map remains a fixed size and keeps the robot approximately in the center
 *   - expanding : the map stretches to encompass all sensor data as the robot drives along
 */
enum class MappingMode
{
    scrolling,
    expanding,
    unchanged   // Leave the mode the same as whatever it was
};

/**
 * MapResolution defines the possible resolutions for the LPM:
 *
 *   - normal : output only the normal-resolution LPM
 *   - high   : output an additional high-resolution LPM of the immediate surround of the robot
 */
enum class MapResolution
{
    normal,
    high,
    unchanged   // Leave the mode the same as whatever it was
};

/**
 * SetSlamModeCommand changes the behavior and outputs of Local Metric HSSH. The command determines:
 *
 *   - if the mapping is run at all
 *   - how the mapping is performed
 *   - if a high-resolution version of the map is created
 *
 * The behaviors can be set individually or all at once. The three constructors allow for just changing one of the
 * behaviors, but the others can be changed as well.
 */
class SetSlamModeCommand : public LocalMetricCommand
{
public:
    /**
     * Constructor for SetSlamModeCommand.
     *
     * Change all modes
     *
     * \param    source          Source module for the command
     * \param    slamMode        Slam mode to use
     * \param    mapMode         Mapping mode to use
     * \param    resolutionMode  Resolution mode to use
     */
    SetSlamModeCommand(const std::string& source, SlamMode slamMode, MappingMode mapMode, MapResolution resolutionMode);

    /**
     * Constructor for SetSlamModeCommand.
     *
     * Change just the SLAM mode.
     *
     * \param    source          Source module for the command
     * \param    slamMode        Slam mode to use
     * \param    mapMode         Mapping mode to use (optional, default = unchanged)
     * \param    resolutionMode  Resolution mode to use (optional, default = unchaged)
     */
    SetSlamModeCommand(const std::string& source, SlamMode slamMode);

    /**
     * Constructor for SetSlamModeCommand.
     *
     * Change just the mapping mode.
     *
     * \param    source          Source module for the command
     * \param    mapMode         Mapping mode to use
     */
    SetSlamModeCommand(const std::string& source, MappingMode mapMode);

    /**
     * Constructor for SetSlamModeCommand.
     *
     * Change just the high-resolution mode.
     *
     * \param    source          Source module for the command
     * \param    resolutionMode  Resolution mode to use
     */
    SetSlamModeCommand(const std::string& source, MapResolution resolutionMode);

    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data,
               Localizer& localizer,
               Mapper& mapper,
               MetricRelocalizer& relocalizer) const override;
    void print(std::ostream& out) const override;

private:
    SlamMode slamMode_;
    MappingMode mappingMode_;
    MapResolution resolution_;

    // Serialization support
    SetSlamModeCommand(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LocalMetricCommand>(this), slamMode_, mappingMode_, resolution_);
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::SetSlamModeCommand)

#endif   // HSSH_LOCAL_METRIC_COMMANDS_SET_SLAM_MODE_H
