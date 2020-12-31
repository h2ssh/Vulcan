/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     command.h
 * \author   Collin Johnson
 *
 * Definition of GlobalTopoCommand abstract base class.
 */

#ifndef HSSH_GLOBAL_TOPO_COMMAND_H
#define HSSH_GLOBAL_TOPO_COMMAND_H

#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>
#include <iostream>
#include <string>

namespace vulcan
{
namespace hssh
{

class TopologicalSLAM;

/**
 * GlobalTopoCommand is the abstract base class for commands to control the functionality of the Global Topo HSSH
 * module. Commands can be issued to the following parts of the module:
 *
 *   - TODO: Create list of possible bits that can be commanded.
 */
class GlobalTopoCommand
{
public:
    /**
     * Constructor for GlobalTopoCommand.
     *
     * \param    source          Source of the command (for informational purposes)
     */
    explicit GlobalTopoCommand(const std::string& source = "") : source_(source) { }

    /**
     * issue issues the command to the module. The components that can be controlled are all provided and the command
     * can interact with whichever components it needs to to complete its task.
     */
    virtual void issue(TopologicalSLAM& slam) const = 0;

    /**
     * print prints an informational message about the command. The print method is called from operator<<.
     *
     * \param    out         Stream to write information to.
     */
    virtual void print(std::ostream& out) const = 0;

    /**
     * source retrieves the source module of the command.
     */
    std::string source(void) const { return source_; }

private:
    std::string source_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(source_);
    }
};


using GlobalTopoCommandPtr = std::shared_ptr<GlobalTopoCommand>;


// Output operator for GlobalTopoCommand
inline std::ostream& operator<<(std::ostream& out, const GlobalTopoCommand& command)
{
    command.print(out);
    return out;
}

}   // namespace hssh
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::GlobalTopoCommandPtr, ("HSSH_GLOBAL_TOPO_COMMAND"))

#endif   // HSSH_GLOBAL_TOPO_COMMAND_H
