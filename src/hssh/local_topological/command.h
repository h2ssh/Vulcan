/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef HSSH_LOCAL_TOPOLOGICAL_COMMAND_H
#define HSSH_LOCAL_TOPOLOGICAL_COMMAND_H

#include <hssh/local_topological/mode.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>

namespace vulcan
{
namespace hssh
{

/**
* LocalTopoCommand provides an interface for issuing commands to control the behavior of the local_topo_hssh module.
* Currently, it is a hack that just allows for turning on/off the event detection.
*/
class LocalTopoCommand
{
public:

    /**
    * Create a new command to set the mode for the module.
    *
    * \param    mode    Mode to switch into
    */
    LocalTopoCommand(LocalTopoMode mode)
    : mode_(mode)
    {
    }

    /**
    * Retrieve the requested mode.
    */
    LocalTopoMode mode(void) const { return mode_; }

private:

    LocalTopoMode mode_;

    // Serialization support
    friend class cereal::access;

    LocalTopoCommand(void) = default;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(mode_);
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_SYSTEM_MESSAGE(std::shared_ptr<hssh::LocalTopoCommand>, ("HSSH_LOCAL_TOPO_COMMAND"))

#endif // HSSH_LOCAL_TOPOLOGICAL_COMMAND_H
