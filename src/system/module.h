/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     module.h
* \author   Collin Johnson
*
* Definition of Module class, providing basic functionality for all Vulcan modules.
*/

#ifndef SYSTEM_MODULE_H
#define SYSTEM_MODULE_H

#include <utils/repeated_task.h>
#include <system/director.h>
#include <system/module_communicator.h>
#include <cstdio>
#include <signal.h>
#include <iostream>

namespace vulcan
{
namespace utils { class CommandLine; }
namespace utils { class ConfigFile; }

namespace system
{

/**
* Module is the high-level abstraction for modules that are running on Vulcan. A Module
* consists of a Communicator and a Director. These two classes handle all the data transmission
* and distribution for the module. The Module handles the task of actually running the
* Communicator and the Director. There's not much to it, but it is the exact same process
* for every module, so handling the task in a single place makes sense, thus Module.
*/
template <class Director>
class Module
{
public:

    /**
    * Constructor for Module.
    *
    * Create a new Director via Director(commandLine, config)
    *
    * \param    commandLine         Command-line provided to the Module
    * \param    config              Configuration for the module
    */
    Module(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
    : communicator_(commandLine, config)
    , director_(new Director(commandLine, config))
    {
        director_->subscribeToData(communicator_);
    }

    /**
    * Constructor for Module.
    *
    * Acquire a pointer to an already created instance of Director.
    *
    * \param    director            Director to use for running the module
    * \param    commandLine         Command-line provided to the Module
    * \param    config              Configuration for the module
    */
    Module(std::unique_ptr<Director> director, const utils::CommandLine& commandLine, const utils::ConfigFile& config)
    : communicator_(commandLine, config)
    , director_(std::move(director))
    {
        director_->subscribeToData(communicator_);
    }

    /**
    * run is the main processing function for the Module. It launches the communicator
    * thread and tells the director to keep processing data as it becomes available.
    *
    * run also sends out a module heartbeat after every processing update so the module
    * heartbeat monitor knows the status of the module, namely that it is still functioning
    * properly.
    */
    int run(void)
    {
        auto communicatorFunc = [this](bool killed) -> bool
        {
            if(!killed)
            {
                communicator_.processIncoming();
            }
            
            return !killed;
        };
        
        auto directorFunc = [this](bool killed) -> bool
        {
            if(!killed)
            {
                // Run an update only if new data has arrived
                if(director_->waitForTrigger() == TriggerStatus::ready)
                {
                    UpdateStatus status = director_->runUpdate(communicator_);

                    // If the director isn't still running, then kill the module.
                    if(status != UpdateStatus::running)
                    {
                        killed = true;
                    }
                }
            }

            if(killed)
            {
                director_->shutdown(communicator_);
            }
            
            sendHeartbeat();
            
            return !killed;
        };
        
        std::cout << "INFO::Module: Initialization complete. Starting communicator and director threads...\n";
        
        // Create an appropriate signal handler to block until a SIGTERM is
        // received from the command-line, at which point the threads will be
        // brought to a halt
        sigset_t mask;
        sigemptyset(&mask);
        sigaddset(&mask, SIGINT);
        sigaddset(&mask, SIGTERM);
        
        if(int sigerror = pthread_sigmask(SIG_BLOCK, &mask, 0))
        {
            std::cerr << "ERROR::Module:run(): Failed to block required signals with pthread_sigmask!\n";
            errno = sigerror;
            perror("pthread_sigmask");
            exit(EXIT_FAILURE);
        }
        
        // Create the threads after the signal handlers are created, so they function properly.
        utils::RepeatedTask communicatorTask(communicatorFunc);
        utils::RepeatedTask directorTask(directorFunc);
        
        int signalCaught = 0;
        int error = sigwait(&mask, &signalCaught);
        
        std::cout << "INFO::Module: Caught signal " << signalCaught << ". Shutting down...\n";

        directorTask.kill();
        directorTask.join();
        
        communicatorTask.kill();
        communicatorTask.join();
        
        return error;
    }

private:

    void sendHeartbeat(void)
    {
        // TODO: Define the nature of a heartbeat
    }

    ModuleCommunicator        communicator_;
    std::unique_ptr<Director> director_;
};

} // namespace system
} // namespace vulcan

#endif // SYSTEM_MODULE_H
