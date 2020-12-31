/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_SUBSCRIPTION_MANAGER_H
#define LCMTYPES_SUBSCRIPTION_MANAGER_H

#include <cassert>
#include <lcm/lcm.h>
#include <map>
#include <string>
#include <vector>

namespace vulcan
{
namespace lcm
{

template <typename vulcan_type>
struct channel_subscriber_t
{
    channel_subscriber_t(const std::string& channel,
                         void* userdata,
                         void (*callback_func)(const vulcan_type&, const std::string&, void*))
    : channel(channel)
    , userdata(userdata)
    , callback(callback_func)
    {
    }

    std::string channel;
    void* userdata;   // this is the custom pointer, prob. to a DataConsumer, that will be given back to callaback's
                      // last argument
    void (*callback)(const vulcan_type&, const std::string&, void*);
};

/**
 * SubscriptionManager organizes the subscriptions to a message in the LCM.
 * Each message needs to be able to handle multiple subscribers to any
 * channel. Due to the abstraction layer, the callback provided from the Vulcan
 * code cannot be directly registered in LCM. The SubscriberManager acts as
 * the glue between the two interfaces.
 */
template <typename lcm_type, typename vulcan_type>
class SubscriptionManager
{
public:
    bool isSubscribedToChannel(lcm_t* lcm, const std::string& channel);

    void addChannelSubscriber(lcm_t* lcm, const channel_subscriber_t<vulcan_type>& newSubscriber);
    void sendDataToSubscribers(lcm_t* lcm, const std::string& channel, const lcm_type& data);

private:
    typedef std::map<std::string, std::vector<channel_subscriber_t<vulcan_type>>> ChannelSubscribersMap;
    using LCMToSubscribersMap = std::map<lcm_t*, ChannelSubscribersMap>;

    LCMToSubscribersMap subscribers;
    vulcan_type vulcanData;   ///< cached data to be used, as it could need allocation and that should happen minimally

    ChannelSubscribersMap* channelSubscribers(lcm_t* lcm)
    {
        auto lcmSubIt = subscribers.find(lcm);
        return lcmSubIt != subscribers.end() ? &lcmSubIt->second : nullptr;
    }
};


template <typename lcm_type, typename vulcan_type>
void subscription_manager_callback(const lcm_recv_buf_t* rbuf, const char* channel, const lcm_type* msg, void* user)
{
    SubscriptionManager<lcm_type, vulcan_type>* manager =
      static_cast<SubscriptionManager<lcm_type, vulcan_type>*>(user);

    manager->sendDataToSubscribers(rbuf->lcm, channel, *msg);
}


template <typename lcm_type, typename vulcan_type>
bool SubscriptionManager<lcm_type, vulcan_type>::isSubscribedToChannel(lcm_t* lcm, const std::string& channel)
{
    ChannelSubscribersMap* channels = channelSubscribers(lcm);
    return channels && (channels->find(channel) != channels->end());
}


template <typename lcm_type, typename vulcan_type>
void SubscriptionManager<lcm_type, vulcan_type>::addChannelSubscriber(
  lcm_t* lcm,
  const channel_subscriber_t<vulcan_type>& newSubscriber)
{
    ChannelSubscribersMap* channels = channelSubscribers(lcm);

    if (!channels) {
        subscribers.insert(std::make_pair(lcm, ChannelSubscribersMap()));
        channels = channelSubscribers(lcm);
        assert(channels);
    }

    channels->insert(std::make_pair(newSubscriber.channel, std::vector<channel_subscriber_t<vulcan_type>>()));
    channels->at(newSubscriber.channel).push_back(newSubscriber);
}


template <typename lcm_type, typename vulcan_type>
void SubscriptionManager<lcm_type, vulcan_type>::sendDataToSubscribers(lcm_t* lcm,
                                                                       const std::string& channel,
                                                                       const lcm_type& data)
{
    if (isSubscribedToChannel(lcm, channel)) {
        convert_lcm_to_vulcan(data, vulcanData);

        std::vector<channel_subscriber_t<vulcan_type>>& subs = channelSubscribers(lcm)->at(channel);

        for (int x = subs.size(); --x >= 0;) {
            subs[x].callback(vulcanData, channel, subs[x].userdata);
        }
    }
}

}   // namespace lcm
}   // namespace vulcan

#endif   // LCMTYPES_SUBSCRIPTION_MANAGER_H
