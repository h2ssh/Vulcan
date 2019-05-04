/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     target_set.h
* \author   Collin Johnson and Jong Jin Park
*
* Definition of MetricTargetSet.
*/

#ifndef MPEPC_METRIC_TARGET_SET_H
#define MPEPC_METRIC_TARGET_SET_H

#include <core/pose.h>
#include <iosfwd>
#include <string>
#include <vector>

namespace vulcan
{

namespace mpepc
{

/**
* named_pose_t defines a pose in a metric map with a name. The name is a text
* string that describes the pose.
*/
struct named_pose_t
{
    std::string   name;
    pose_t pose;
};

inline bool operator==(const named_pose_t& lhs, const named_pose_t& rhs)
{
    return (lhs.name == rhs.name) && (lhs.pose == rhs.pose);
}

/**
* MetricTargetSet is a set of targets associated with some LPM. These targets
* are used for creating a script or within the logical interface for providing
* the user with a set of possible goals for the robot to navigate to.
*
* A MetricTargetSet can either be created in the DebugUI with the Scripting
* panel or by hand. The file format for the target set is:
*
*   name target.x target.y target.theta type
*   ....
*/

class MetricTargetSet
{
public:

    using ConstIter = std::vector<named_pose_t>::const_iterator;

    /**
    * Constructor for MetricTargetSet.
    *
    * Create an empty set.
    */
    MetricTargetSet(void) { }

    /**
    * Constructor for MetricTargetSet.
    *
    * Load a target set from a file.
    *
    * \param    in          File containing the targets
    */
    MetricTargetSet(std::istream& in);

    /**
    * Constructor for MetricTargetSet.
    *
    * Create a set from an existing collection of targets. Duplicates will be removed.
    *
    * \param    targets     Targets to be added to the set.
    */
    MetricTargetSet(const std::vector<named_pose_t>& targets);

    /**
    * addTarget adds a new target to the set. If a target with the same name exists, the previous target will be erased.
    *
    *
    * \param    target      Target to add
    */
    void addTarget(const named_pose_t& target);

    /**
    * removeTarget removes a target from the set. Any target with the same name will match this target.
    *
    * \param    target      Target to remove
    * \return   True if the target was found and returned. False if no such target existed.
    */
    bool removeTarget(const named_pose_t& target);

    /**
    * saveToFile saves the target set to a file.
    *
    * \param    file        File to save the target set to
    * \return   True if the set was saved successfully.
    */
    bool saveToFile(std::ostream& out) const;

    // Iterators for the target set
    ConstIter   begin(void) const { return targets.cbegin(); }
    ConstIter   end(void)   const { return targets.cend(); }
    std::size_t size(void)  const { return targets.size(); }

private:

    std::vector<named_pose_t> targets;
};

} // mpepc
} // vulcan

#endif // MPEPC_METRIC_TARGET_SET_H
