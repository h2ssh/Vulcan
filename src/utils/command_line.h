/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     command_line.h
 * \author   Collin Johnson
 *
 * Definition of CommandLine.
 */

#ifndef UTILS_COMMAND_LINE_PARSER_H
#define UTILS_COMMAND_LINE_PARSER_H

#include <map>
#include <string>
#include <vector>

namespace vulcan
{
namespace utils
{

const std::string kConfigFileArgument("config-file");
const std::string kHelpArgumentShort("h");
const std::string kHelpArgumentLong("help");

struct command_line_argument_t
{
    std::string name;
    std::string description;
    bool isOptional;
    std::string defaultValue;
};

/**
 * CommandLine is a utility that handles parsing the command-line arguments supplied
 * to a process. The format of the arguments that can be parsed by this CommandLine
 * are as follows:
 *
 *   -arg
 *   --long-arg
 *   -arg value
 *   --long-arg value
 *
 * Example: laser_producer -p /dev/ttyUSB0 --use-lcm --channel-name FRONT_HOKUYO --laser-model UTM_LX30
 *
 * The help arguments are either: '-h'  or '--help'
 */
class CommandLine
{
public:
    /**
     * Constructor for CommandLine.
     *
     * \param    argc        Number of command-line arguments
     * \param    argv        Value of the command-line arguments
     */
    CommandLine(int argc, char** argv);

    /**
     * Constructor for CommandLine.
     *
     * Creates a CommandLine with a set of known arguments. Some are required and some are
     * optional.
     *
     * \param    argc        Number of command-line arguments
     * \param    argv        The argument strings
     * \param    arguments   Collection of known and/or required command-line arguments
     */
    CommandLine(int argc, char** argv, const std::vector<command_line_argument_t>& arguments);

    /**
     * verify checks the parsed command line to see if all required arguments were provided or
     * if help was requested. If all requirements aren't met or help was requested, then the
     * description for each parameter will be printed.
     *
     * \return   True if all required arguments exist and help was not requested.
     */
    bool verify(void) const;

    /**
     * printHelp prints the help message, a description of each argument.
     */
    void printHelp(void) const;

    /**
     * moduleName retrieves the name of the module as contained in argv[0].
     */
    std::string moduleName(void) const { return moduleName_; }

    /**
     * configName retrieves the name of the config file associated with the module. If
     * argumentExists(kConfigFileArgument), it will be the value from the command line. Otherwise
     * it will be the moduleName() + .cfg.
     */
    std::string configName(void) const;

    /**
     * argumentExists checks to see if an argument is defined. This method is
     * used for arguments that don't have corresponding values.
     *
     * \return   True if the specified argument was on the command line. False otherwise.
     */
    bool argumentExists(const std::string& argument) const;

    /**
     * argumentValue retrieves the value associated with the specified argument. This method
     * is used for arguments that have a value specified with them.
     *
     * \param    argument            Name of the argument to retrieve
     * \param    defaultValue        Default value of the argument if it doesn't exist (optional)
     * \return   The value of the argument. Empty string if no value or the argument didn't exist.
     */
    std::string argumentValue(const std::string& argument, std::string defaultValue = std::string("")) const;

    /**
     * numArguments retrieves the number of arguments in the command-line, including the process name.
     */
    size_t numArguments(void) const { return arguments_.size(); }

private:
    std::string moduleName_;
    std::map<std::string, std::string> arguments_;
    std::vector<command_line_argument_t> argumentDescriptions_;
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_COMMAND_LINE_PARSER_H
