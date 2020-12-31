/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/message_helpers.h"
#include <algorithm>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace lcm
{
    
void verify_channel(const std::string& channel, const std::string& defaultChannel, bool isSubscribing)
{
    assert(!channel.empty());
    
    if(channel != defaultChannel)
    {
        std::cerr<<"WARNING:"
                 <<(isSubscribing ? "subscribe_to_message" : "publish_data:")
                 <<"Not "
                 <<(isSubscribing ? "subscribing to " : "publishing on ")
                 <<"the default channel. Default:"<<defaultChannel<<" Channel:"<<channel<<'\n';
    }
}


void verify_channel(const std::string& channel, const std::vector<std::string>& defaultChannels, bool isSubscribing)
{
    // If the channel can't be found, then spit out a bunch of warning messages
    if(std::find(defaultChannels.begin(), defaultChannels.end(), channel) == defaultChannels.end())
    {
        for(const auto& defaultChannel : defaultChannels)
        {
            verify_channel(channel, defaultChannel, isSubscribing);
        }
    }
}
    
}
}
