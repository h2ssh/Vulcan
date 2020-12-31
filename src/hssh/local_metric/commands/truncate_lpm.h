/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     truncate_lpm.h
 * \author   Collin Johnson
 *
 * Declaration of TruncateLpmCommand.
 */

#ifndef HSSH_LOCAL_METRIC_COMMANDS_TRUNCATE_LPM_H
#define HSSH_LOCAL_METRIC_COMMANDS_TRUNCATE_LPM_H

#include "hssh/local_metric/command.h"
#include "math/geometry/rectangle.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * TruncateLpmCommand truncates the LPM to fit within the boundary specified in the message.
 */
class TruncateLpmCommand : public LocalMetricCommand
{
public:
    /**
     * Constructor for TruncateLpmCommand.
     *
     * \param    source          Source module for the command
     * \param    boundary        Boundary to which the lpm should be truncated
     */
    TruncateLpmCommand(const std::string& source, const math::Rectangle<float>& boundary);

    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data,
               Localizer& localizer,
               Mapper& mapper,
               MetricRelocalizer& relocalizer) const override;
    void print(std::ostream& out) const override;

private:
    math::Rectangle<float> boundary_;

    // Serialization support
    TruncateLpmCommand(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LocalMetricCommand>(this), boundary_);
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::TruncateLpmCommand)

#endif   // HSSH_LOCAL_METRIC_COMMANDS_TRUNCATE_LPM_H
