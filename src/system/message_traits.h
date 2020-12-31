/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SYSTEM_MESSAGE_TRAITS_H
#define SYSTEM_MESSAGE_TRAITS_H

#include <boost/preprocessor/tuple/enum.hpp>
#include <boost/preprocessor/tuple/size.hpp>
#include <cassert>
#include <string>
#include <type_traits>

namespace vulcan
{
namespace system
{

struct pure_lcm_message_tag
{
};
struct old_message_tag
{
};
struct system_message_tag
{
};
struct debug_message_tag
{
};

/**
 * message_traits defines the traits that control how a message is sent to other modules. The traits for a message can
 * be divided into two categories: the type of message and the channel(s) on which it is sent.
 *
 * The type of the message defines which subnetwork it will be transmitted on and how it will be serialized.
 *
 */
template <class MessageType>
struct message_traits
{
    using type = old_message_tag;

    enum
    {
        num_channels = 0
    };   ///< num_channels specifies the number of channels that exist for the type. The majority of messages
         ///< will have a single channel.

    /**
     * channelName retrieves the name for the channel at the provided index. The index must be in the range [0,
     * num_channels) or the program will fail with an assertion immediately.
     */
    static std::string channelName(int index)
    {
        assert(false);
        return "DEFAULT";
    }
};

}   // namespace system
}   // namespace vulcan

// For most messages, the only thing that changes is the channel name. These macros will define the message_traits
// structs using the specified class and channel name

#define DEFINE_SERIALIZATION_MSG_TRAITS(MessageType, channels, tag)                       \
    namespace vulcan                                                                      \
    {                                                                                     \
    namespace system                                                                      \
    {                                                                                     \
    template <>                                                                           \
    struct message_traits<MessageType>                                                    \
    {                                                                                     \
        using type = tag;                                                                 \
                                                                                          \
        enum                                                                              \
        {                                                                                 \
            num_channels = BOOST_PP_TUPLE_SIZE(channels)                                  \
        };                                                                                \
                                                                                          \
        static std::string channelName(int index)                                         \
        {                                                                                 \
            static const char* kChannels[num_channels] = {BOOST_PP_TUPLE_ENUM(channels)}; \
            assert(index >= 0 && index < num_channels);                                   \
            return kChannels[index];                                                      \
        }                                                                                 \
    };                                                                                    \
    }                                                                                     \
    }

// Easier to use macros so the tag type doesn't need to be specified
#define DEFINE_SYSTEM_MESSAGE(MessageType, channels) \
    DEFINE_SERIALIZATION_MSG_TRAITS(MessageType, channels, system_message_tag)
#define DEFINE_DEBUG_MESSAGE(MessageType, channels) \
    DEFINE_SERIALIZATION_MSG_TRAITS(MessageType, channels, debug_message_tag)

#endif   // SYSTEM_MESSAGE_TRAITS_H
