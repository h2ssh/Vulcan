/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "utils/command_line.h"
#include <cassert>
#include <iostream>

//#define ASSERT_ERRORS

using vulcan::utils::CommandLine;


struct TestStatus
{
    TestStatus(int numTests) : totalTests(numTests), testsRun(0), testsPassed(0) { }

    const int totalTests;
    int testsRun;
    int testsPassed;
};


// Helpers for creation
std::string short_argument(const std::string& argument);
std::string long_argument(const std::string& argument);

// Helpers for running tests on the CommandLine
bool argument_should_exist(const CommandLine& commandLine, const std::string& argument, TestStatus& status);
bool argument_should_not_exist(const CommandLine& commandLine, const std::string& argument, TestStatus& status);
bool argument_value_should_equal(const CommandLine& commandLine,
                                 const std::string& argument,
                                 const std::string& value,
                                 TestStatus& status);

// Helpers for displaying status
void display_tests_progress(const TestStatus& status);


/**
 * command_line_test validates that the CommandLine class works correctly
 * by supplying a hand-coded command-line that consists of both correct and incorrect
 * statements and then checks to see that the arguments are included or excluded properly.
 */
int main(int argc, char** argv)
{
    const std::string PROCESS_NAME("command_line_test");
    const std::string ARG0("short");
    const std::string ARG1("long-arg");
    const std::string ARG2("-");
    const std::string ARG3("--");
    const std::string ARG4("---");
    const std::string ARG5("s");
    const std::string ARG6("l");
    const std::string ARG7("another-long");

    const std::string VALUE5("1000");
    const std::string VALUE6("booyah!");
    const std::string VALUE7("broken");

    const std::string FAKE_ARG0("--imaginary-arg");
    const std::string NO_VALUE("");

    std::string ARG0_FULL = short_argument(ARG0);
    std::string ARG1_FULL = long_argument(ARG1);
    std::string ARG4_FULL = long_argument(ARG4);
    std::string ARG5_FULL = short_argument(ARG5);
    std::string ARG6_FULL = long_argument(ARG6);
    std::string ARG7_FULL = long_argument(ARG7);

    int numArguments = 12;
    char* command[12];

    // First time ever using const_cast<>. It feels dirty.
    // Certainly a better way, but this will do the trick
    command[0] = const_cast<char*>(PROCESS_NAME.c_str());
    command[1] = const_cast<char*>(ARG0_FULL.c_str());
    command[2] = const_cast<char*>(ARG1_FULL.c_str());
    command[3] = const_cast<char*>(ARG2.c_str());
    command[4] = const_cast<char*>(ARG3.c_str());
    command[5] = const_cast<char*>(ARG4_FULL.c_str());
    command[6] = const_cast<char*>(ARG5_FULL.c_str());
    command[7] = const_cast<char*>(VALUE5.c_str());
    command[8] = const_cast<char*>(ARG6_FULL.c_str());
    command[9] = const_cast<char*>(VALUE6.c_str());
    command[10] = const_cast<char*>(VALUE7.c_str());
    command[11] = const_cast<char*>(ARG7_FULL.c_str());

    CommandLine commandLine(numArguments, command);

    TestStatus status(21);

    // Check that all valid arguments are added correctly
    argument_should_exist(commandLine, ARG0, status);
    display_tests_progress(status);

    argument_should_exist(commandLine, ARG1, status);
    display_tests_progress(status);

    argument_should_exist(commandLine, ARG4, status);
    display_tests_progress(status);

    argument_should_exist(commandLine, ARG5, status);
    display_tests_progress(status);

    argument_should_exist(commandLine, ARG6, status);
    display_tests_progress(status);

    argument_should_exist(commandLine, ARG7, status);
    display_tests_progress(status);

    // Check that values, the processs name, and non-existant arguments are not added
    argument_should_not_exist(commandLine, PROCESS_NAME, status);
    display_tests_progress(status);

    argument_should_not_exist(commandLine, ARG2, status);
    display_tests_progress(status);

    argument_should_not_exist(commandLine, ARG3, status);
    display_tests_progress(status);

    argument_should_not_exist(commandLine, VALUE5, status);
    display_tests_progress(status);

    argument_should_not_exist(commandLine, VALUE6, status);
    display_tests_progress(status);

    argument_should_not_exist(commandLine, VALUE7, status);
    display_tests_progress(status);

    argument_should_not_exist(commandLine, FAKE_ARG0, status);
    display_tests_progress(status);

    // Check that the assigned values are correct
    argument_value_should_equal(commandLine, ARG5, VALUE5, status);
    display_tests_progress(status);

    argument_value_should_equal(commandLine, ARG6, VALUE6, status);
    display_tests_progress(status);

    // All other arguments should have no value
    argument_value_should_equal(commandLine, ARG0, NO_VALUE, status);
    display_tests_progress(status);

    argument_value_should_equal(commandLine, ARG1, NO_VALUE, status);
    display_tests_progress(status);

    argument_value_should_equal(commandLine, ARG4, NO_VALUE, status);
    display_tests_progress(status);

    argument_value_should_equal(commandLine, ARG7, NO_VALUE, status);
    display_tests_progress(status);

    // Invalid arguments should have no value as well
    argument_value_should_equal(commandLine, ARG2, NO_VALUE, status);
    display_tests_progress(status);

    argument_value_should_equal(commandLine, ARG3, NO_VALUE, status);
    display_tests_progress(status);

    assert(status.testsRun == status.testsPassed);

    return 0;
}


// Helpers for creation
std::string short_argument(const std::string& argument)
{
    return "-" + argument;
}


std::string long_argument(const std::string& argument)
{
    return "--" + argument;
}


// Helpers for running tests on the CommandLine
bool argument_should_exist(const CommandLine& commandLine, const std::string& argument, TestStatus& status)
{
    bool success = commandLine.argumentExists(argument);

    if (!success) {
        std::cerr << "ERROR: Argument " << argument << " should exist but doesn't" << std::endl;
    } else {
        ++status.testsPassed;
    }

    ++status.testsRun;

    return success;
}


bool argument_should_not_exist(const CommandLine& commandLine, const std::string& argument, TestStatus& status)
{
    bool success = !commandLine.argumentExists(argument);

    if (!success) {
        std::cerr << "ERROR: Argument " << argument << " should not exist but does" << std::endl;
    } else {
        ++status.testsPassed;
    }

    ++status.testsRun;

    return success;
}


bool argument_value_should_equal(const CommandLine& commandLine,
                                 const std::string& argument,
                                 const std::string& value,
                                 TestStatus& status)
{
    std::string parsedValue = commandLine.argumentValue(argument);

    bool success = value == parsedValue;

    if (!success) {
        std::cerr << "ERROR: Parsed argument value was " << parsedValue << " but should be " << value << std::endl;
    } else {
        ++status.testsPassed;
    }

    ++status.testsRun;

    return success;
}


// Helpers for displaying status
void display_tests_progress(const TestStatus& status)
{
    std::cout << "Tests Run: [" << status.testsRun << " / " << status.totalTests << "] Passed: [" << status.testsPassed
              << " / " << status.testsRun << "]" << std::endl;
}
