/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     command_line.cpp
* \author   Collin Johnson
*
* Implementation of CommandLine.
*/

#include "utils/command_line.h"
#include <algorithm>
#include <cstring>
#include <iostream>

#define VERBOSE_PARSER

namespace vulcan
{
namespace utils
{

typedef std::map<std::string, std::string> ArgumentMap;

// Helpers for doing the actual parsing
std::string program_name(char* arg);
ArgumentMap parse_command_line(int argc, char** argv);
bool        is_argument(const char* value);
std::string argument_name(const char* value);


CommandLine::CommandLine(int argc, char** argv)
: moduleName_(program_name(argv[0]))
, arguments_(parse_command_line(argc, argv))
{
}


CommandLine::CommandLine(int argc, char** argv, const std::vector<command_line_argument_t>& arguments)
: moduleName_(program_name(argv[0]))
, arguments_(parse_command_line(argc, argv))
, argumentDescriptions_(arguments)
{
}


bool CommandLine::verify(void) const
{
    if(argumentExists(kHelpArgumentShort) || argumentExists(kHelpArgumentLong))
    {
        printHelp();
        return false;
    }

    for(auto arg : argumentDescriptions_)
    {
        if(!arg.isOptional && !argumentExists(arg.name))
        {
            printHelp();
            return false;
        }
    }

    return true;
}


void CommandLine::printHelp(void) const
{
    std::cout << "The command-line arguments for " << moduleName_ << " are:\n";

    for(auto arg : argumentDescriptions_)
    {
        std::cout << arg.name << " : " << arg.description << " Optional:" << arg.isOptional;

        if(arg.isOptional)
        {
            std::cout << " Default:" << arg.defaultValue;
        }

        std::cout << '\n';
    }
}


std::string CommandLine::configName(void) const
{
    std::string config = argumentValue(kConfigFileArgument);
    return config.empty() ? (moduleName_ + ".cfg") : config;
}


bool CommandLine::argumentExists(const std::string& argument) const
{
    ArgumentMap::const_iterator argumentPosition = arguments_.find(argument);

    return argumentPosition != arguments_.end();
}


std::string CommandLine::argumentValue(const std::string& argument, std::string defaultValue) const
{
    ArgumentMap::const_iterator argumentPosition = arguments_.find(argument);

    // If specified, return the specified value
    if(argumentPosition != arguments_.end())
    {
        return argumentPosition->second;
    }
    // Otherwise see if a default was provided when the arguments were added to the command-line
    else if(defaultValue.empty())
    {
        auto argIt = std::find_if(argumentDescriptions_.begin(), argumentDescriptions_.end(), [&argument](const command_line_argument_t& arg) {
            return arg.name == argument;
        });

        if((argIt != argumentDescriptions_.end()) && !argIt->defaultValue.empty())
        {
            return argIt->defaultValue;
        }
    }

    // Otherwise use the default defaultValue, which is an empty string.
    return defaultValue;
}


// Helper for doing the actual parsing
std::string program_name(char* arg)
{
    std::string name(arg);
    std::size_t finalSlashPos = name.rfind('/');

    return (finalSlashPos == std::string::npos) ? name : name.substr(finalSlashPos+1);
}


ArgumentMap parse_command_line(int argc, char** argv)
{
    // Distinction here is between argument (begins with -) and values. Values
    // must be associated with an argument, but arguments don't have to have values
    ArgumentMap arguments;
    std::string currentArgument;

    // argv[0] is the process name
    for(int x = 1; x < argc; ++x)
    {
        if(is_argument(argv[x]))
        {
            if(!currentArgument.empty())
            {
                arguments.insert(std::make_pair(currentArgument, std::string("")));

                #ifdef VERBOSE_PARSER
                std::cout<<"CommandLine: Argument added: "<<currentArgument<<std::endl;
                #endif
            }

            currentArgument = argument_name(argv[x]);
        }
        else // is value
        {
            if(!currentArgument.empty())
            {
                arguments.insert(std::make_pair(currentArgument, std::string(argv[x])));

#ifdef VERBOSE_PARSER
                std::cout<<"CommandLine: Argument added: "<<currentArgument<<" = "<<argv[x]<<std::endl;
#endif
            }
            else // Value with no argument deserves a warning to the user
            {
                std::cerr<<"WARNING: CommandLine: Value "<<argv[x]<<" has no associated argument"<<std::endl;
            }

            currentArgument.clear();
        }
    }

    // If there is a lingering currentArgument when the loop finishes, then add it too
    if(!currentArgument.empty())
    {
        arguments.insert(std::make_pair(currentArgument, std::string("")));

#ifdef VERBOSE_PARSER
        std::cout<<"CommandLine: Argument added: "<<currentArgument<<std::endl;
#endif
    }

    return arguments;
}


bool is_argument(const char* value)
{
    return (strlen(value) > 2) && (value[0] == '-') && (value[1] == '-');
}


std::string argument_name(const char* value)
{
    // Know that [0] == '-' because it is an argument, so see if
    // [1] exists and is an argument too
    int argumentLength = strlen(value);

    // Invalid argument is "-" or "--" with no other characters
    if((argumentLength==2 && value[1]=='-') || (argumentLength == 1))
    {
        std::cerr<<"WARNING: CommandLine: Invalid argument name: "<<value<<std::endl;
        return std::string("");
    }

    // Length is at least 2 if at this point
    if(value[1] == '-')
    {
        return std::string(value + 2);
    }
    else
    {
        return std::string(value + 1);
    }
}

} // namespace utils
} // namespace vulcan
