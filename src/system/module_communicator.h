/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     module_communicator.h
* \author   Collin Johnson
*
* Definition of ModuleCommunicator.
*/

#ifndef SYSTEM_MODULE_COMMUNICATOR_H
#define SYSTEM_MODULE_COMMUNICATOR_H

#include "lcmtypes/lcm_types.h"
#include "system/message_traits.h"
#include "system/serialized_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream.hpp>

namespace vulcan
{
namespace utils { class CommandLine; }
namespace utils { class ConfigFile; }

namespace system
{

/**
* systemConnection_receiver_callback is the callback that is used for interacting with the LCM subsystem.
*
* This function is used internally and needn't ever be used by anything but DataReceiver
*/
template <class Data, class DataConsumer>
void lcm_receiver_callback(const Data& data, const std::string& channel, void* userdata)
{
    DataConsumer* consumer = static_cast<DataConsumer*>(userdata);
    consumer->handleData(data, channel);
}


/**
* ModuleCommunicator implements the DataProducer concept using the LCM library as the interprocess communication backbone.
*/
class ModuleCommunicator
{
public:

    /**
    * Default constructor for ModuleCommunicator.
    */
    ModuleCommunicator(void);

    /**
    * Constructor for ModuleCommunicator.
    */
    ModuleCommunicator(const utils::CommandLine& cmdLine, const utils::ConfigFile& config);

    /**
    * Constructor for ModuleCommunicator.
    *
    * Create a ModuleCommunicator using the provided URLs for the system and debug connections.
    *
    * \param    systemUrl       LCM URL to use for the system LCM provider
    * \param    debugUrl        LCM URL to use for the debug LCM provider
    */
    ModuleCommunicator(const std::string& systemUrl, const std::string& debugUrl);

    /**
    * sendMessage sends a message to other modules using the LCM protocol.
    *
    * If a channel name is not provided, then the default channel name for the given data will be used instead.
    * Using the default channel name is the desired behavior to ensure modules are correctly wired together.
    *
    * \param    message     Vulcan data to be sent
    * \param    channel     Channel on which to send the data (optional unless message_traits<MessageType>::num_channels > 1)
    */
    template <typename MessageType>
    void sendMessage(const MessageType& message, const std::string& channel = "")
    {
        typename message_traits<MessageType>::type msgType;
        sendMessageHelper(message, channel, msgType);
    }

    /**
    * subscribeTo subscribes a DataConsumer to receive incoming data of the Data type.
    *
    * In order to subscribe to data, the DataConsumer type needs to have a method with this signature:
    *
    *   void handleData(const Data& data, const std::string& channel);
    *
    * The data will arrive on a separate thread from the main rendering/event thread, so protections
    * will need to be applied to ensure no races occur.
    *
    * \param    consumer        Consumer to receive the messages
    * \param    channel         Channel on which to subscribe (optional, by default subscribe to all channels defined
    *                           in the message traits)
    */
    template <class MessageType, class DataConsumer>
    void subscribeTo(DataConsumer* consumer, const std::string& channel = "")
    {
        typename message_traits<MessageType>::type msgType;
        subscribeToHelper<MessageType>(consumer, msgType, channel);
    }

    /**
    * processIncoming checks for new LCM data and passes it on to any subscribers.
    *
    * \param    waitMs          Amount of time to wait for new data in the poll call (optional, default = 100ms)
    * \return   0 on success. -1 if there is some sort of error, like end-of-file, etc
    */
    int processIncoming(int waitMs = 100);

private:

    ::lcm::LCM systemConnection_;
    ::lcm::LCM debugConnection_;

    bool haveDebugSubscription_;    // Flag whether a debug subscription exists to avoid opening a read socket
                                    // which increases CPU usage of a module due to all multicast messages being received

    serialized_t outMsg_;  // Cache the outbound message so the memory can be reused

    // No value semantics
    ModuleCommunicator(const ModuleCommunicator&  toCopy) = delete;
    ModuleCommunicator(const ModuleCommunicator&& toMove) = delete;
    void operator=(const ModuleCommunicator&  rhs) = delete;
    void operator=(const ModuleCommunicator&& rhs) = delete;

    // Tag dispatch methods for doing the actual transmission based on the type of message specified in the traits
    template <class MessageType> void sendMessageHelper(const MessageType& message, const std::string& channel, pure_lcm_message_tag);
    template <class MessageType> void sendMessageHelper(const MessageType& message, const std::string& channel, old_message_tag);
    template <class MessageType> void sendMessageHelper(const MessageType& message, const std::string& channel, system_message_tag);
    template <class MessageType> void sendMessageHelper(const MessageType& message, const std::string& channel, debug_message_tag);

    // Helper for sending messages that can be serialized via Boost.Serialization
    template <class MessageType> void sendSerializedMessage(const MessageType& message, const std::string& channel, ::lcm::LCM& connection);

    // Tag dispatch methods for doing the actual subscription based on the type of message specified in the traits
    template <class MessageType, class DataConsumer> void subscribeToHelper(DataConsumer* consumer, pure_lcm_message_tag, const std::string& channel = "");
    template <class MessageType, class DataConsumer> void subscribeToHelper(DataConsumer* consumer, old_message_tag, const std::string& channel = "");
    template <class MessageType, class DataConsumer> void subscribeToHelper(DataConsumer* consumer, system_message_tag, const std::string& channel = "");
    template <class MessageType, class DataConsumer> void subscribeToHelper(DataConsumer* consumer, debug_message_tag, const std::string& channel = "");
};


/*
* distribute_message unserializes an instance of Type and distributes it to the Consumer. This is a utility function that should never be called.
*/
template <class Type, class Consumer>
void distribute_serialized(const ::lcm::ReceiveBuffer* rbuf, const std::string& channel, const serialized_t* data, Consumer* consumer)
{
    try
    {
        boost::iostreams::array_source inputBuf(data->data.data(), data->data.size());
        boost::iostreams::stream<decltype(inputBuf)> input(inputBuf);
        cereal::BinaryInputArchive in(input);

        Type t;
        in >> t;

        consumer->handleData(t, channel);
    }
    catch(std::exception& e)
    {
        std::cerr << "EXCEPTION: Failed to unserialize message of type: " << typeid(Type).name() << " Error : " << e.what() << " Message discarded!\n";
    }
}

/**
* distribute_pure_lcm_message distributes a message that doesn't need any conversions. It's just an LCM message.
*/
template <class Message, class DataConsumer>
void distribute_pure_lcm(const ::lcm::ReceiveBuffer* rbuf, const std::string& channel, const Message* data, DataConsumer* consumer)
{
    consumer->handleData(*data, channel);
}


template <class MessageType, class DataConsumer>
void subscribe_to_message(DataConsumer* consumer, ::lcm::LCM& connection, const std::string& channel)
{
    using traits = message_traits<MessageType>;

    if(channel.length() > 0)
    {
        connection.subscribeFunction(channel, distribute_serialized<MessageType, DataConsumer>, consumer);
    }
    else
    {
        assert(traits::num_channels > 0);

        for(std::size_t n = 0; n < traits::num_channels; ++n)
        {
            connection.subscribeFunction(traits::channelName(n), distribute_serialized<MessageType, DataConsumer>, consumer);
        }
    }
}


template <class MessageType>
void ModuleCommunicator::sendMessageHelper(const MessageType& message, const std::string& channel, pure_lcm_message_tag)
{
    if(channel.empty())
    {
        using traits = message_traits<MessageType>;
        assert(traits::num_channels == 1);

        systemConnection_.publish(traits::channelName(0), &message);
    }
    else
    {
        systemConnection_.publish(channel, &message);
    }
}


template <class MessageType>
void ModuleCommunicator::sendMessageHelper(const MessageType& message, const std::string& channel, old_message_tag)
{
    if(channel.length() > 0)
    {
        lcm::publish_data(systemConnection_.getUnderlyingLCM(), message, channel);
    }
    else
    {
        lcm::publish_data(systemConnection_.getUnderlyingLCM(), message);
    }
}


template <class MessageType>
void ModuleCommunicator::sendMessageHelper(const MessageType& message, const std::string& channel, system_message_tag)
{
    sendSerializedMessage(message, channel, systemConnection_);
}


template <class MessageType>
void ModuleCommunicator::sendMessageHelper(const MessageType& message, const std::string& channel, debug_message_tag)
{
    sendSerializedMessage(message, channel, debugConnection_);
}


template <class MessageType>
void ModuleCommunicator::sendSerializedMessage(const MessageType& message, const std::string& channel, ::lcm::LCM& connection)
{
    using msg_traits = message_traits<MessageType>;

    msg_traits traits;

    assert(!channel.empty() || traits.num_channels == 1);

    outMsg_.data.clear();

    try
    {
        // Put the archiving stream in a scope so the destructors will be called and automatically flush the buffered data into outMsg_.data
        {
            auto outDevice = boost::iostreams::back_inserter(outMsg_.data);
            boost::iostreams::stream<decltype(outDevice)> outStream(outDevice);
            cereal::BinaryOutputArchive out(outStream);
            out << message;
        }

        outMsg_.size = outMsg_.data.size();
        connection.publish(channel.empty() ? traits.channelName(0) : channel, &outMsg_);
    }
    catch(std::exception& e)
    {
        std::cerr << "EXCEPTION: Failed to serialize message of type: " << typeid(MessageType).name() << " Error : " << e.what() << " Message discarded!\n";
    }
}


template <class MessageType, class DataConsumer>
void ModuleCommunicator::subscribeToHelper(DataConsumer* consumer, pure_lcm_message_tag, const std::string& channel)
{
    if(channel.length() > 0)
    {
        systemConnection_.subscribeFunction(channel, distribute_pure_lcm<MessageType, DataConsumer>, consumer);
    }
    else
    {
        using traits = message_traits<MessageType>;

        assert(traits::num_channels > 0);

        for(std::size_t n = 0; n < traits::num_channels; ++n)
        {
            systemConnection_.subscribeFunction(traits::channelName(n), distribute_pure_lcm<MessageType, DataConsumer>, consumer);
        }
    }
}


template <class MessageType, class DataConsumer>
void ModuleCommunicator::subscribeToHelper(DataConsumer* consumer, old_message_tag, const std::string& channel)
{
    if(channel.length() > 0)
    {
        lcm::subscribe_to_message(systemConnection_.getUnderlyingLCM(), lcm_receiver_callback<MessageType, DataConsumer>, consumer, channel);
    }
    else
    {
        lcm::subscribe_to_message(systemConnection_.getUnderlyingLCM(), lcm_receiver_callback<MessageType, DataConsumer>, consumer);
    }
}


template <class MessageType, class DataConsumer>
void ModuleCommunicator::subscribeToHelper(DataConsumer* consumer, system_message_tag, const std::string& channel)
{
    subscribe_to_message<MessageType>(consumer, systemConnection_, channel);
}


template <class MessageType, class DataConsumer>
void ModuleCommunicator::subscribeToHelper(DataConsumer* consumer, debug_message_tag, const std::string& channel)
{
    subscribe_to_message<MessageType>(consumer, debugConnection_, channel);
    haveDebugSubscription_ = true;
}

} // namespace system
} // namespace vulcan

#endif // SYSTEM_MODULE_COMMUNICATOR_H
