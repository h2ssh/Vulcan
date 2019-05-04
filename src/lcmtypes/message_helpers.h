/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_MESSAGE_HELPERS_H
#define LCMTYPES_MESSAGE_HELPERS_H

#include <string>
#include <vector>

namespace vulcan
{
namespace lcm
{
    
void verify_channel(const std::string& channel, const std::string&              defaultChannel,  bool isSubscribing);
void verify_channel(const std::string& channel, const std::vector<std::string>& defaultChannels, bool isSubscribing);
    
}
}

#endif // LCMTYPES_MESSAGE_HELPERS_H
