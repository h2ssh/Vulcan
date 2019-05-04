/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_LASER_LASER_SCAN_LINES_H
#define SENSORS_LASER_LASER_SCAN_LINES_H

#include <core/laser_scan.h>
#include <core/line.h>
#include <system/message_traits.h>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace laser
{

/**
* laser_scan_lines_t contains a laser scan and the lines that were found in the scan.
* An additional array is included that contains the mapping of scan point to line segment
* index. This mapping allows an algorithm to know the association between the raw points
* and the lines they generate, allowing reasoning on both levels to occur. If a point has
* no associated line, the index value will be -1.
*/
struct laser_scan_lines_t
{
    polar_laser_scan_t             scan;
    std::vector<Line<float>> lines;
    std::vector<int16_t>           scanPointToLineIndices;
    pose_t                  pose;            ///< Pose at which the scan associated with the lines was taken
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, laser_scan_lines_t& lines)
{
    ar (lines.scan,
        lines.lines,
        lines.scanPointToLineIndices,
        lines.pose);
}

}
}

DEFINE_DEBUG_MESSAGE(laser::laser_scan_lines_t, ("DEBUG_LASER_SCAN_LINES"))

#endif // SENSORS_LASER_LASER_SCAN_LINES_H
