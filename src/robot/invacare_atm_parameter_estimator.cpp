/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "core/matrix.h"
#include "core/vector.h"
#include "robot/invacare_atm.h"
#include "sensors/microstrain_3dmgx3.h"
#include "utils/command_line.h"
#include "utils/timestamp.h"   // added to mirror Quantum 6000 implementation
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>


using namespace vulcan;   // moved this line to be visually identical to Quantum 6000 implementation


const std::string JOYSTICK_PORT("joystick-port");
const std::string CONTROLLER_PORT("controller-port");
const std::string IMU("imu-port");
const std::string RAMP("ramp-time");
const std::string SAMPLES("samples");
const std::string FILENAME("output-file");
const std::string PARAMETERS("parameter-file");
const std::string TURN_IN_PLACE("turn-in-place");   // adding 'turn-in-place' and 'circle' options
const std::string CIRCLE("circle");
const std::string STRAIGHT("straight");   // adding 'straight' as a test option


using namespace vulcan;


struct test_configuration_t
{
    test_configuration_t(robot::InvacareATM& invacare,
                         sensors::Microstrain3DMGX3& imu,
                         int64_t ramp,
                         int samples,
                         robot::joystick_command_t& command)
    : invacare(invacare)
    , imu(imu)
    , rampTime(ramp)
    , testSamples(samples)
    , command(command)
    {
    }

    robot::InvacareATM& invacare;
    sensors::Microstrain3DMGX3& imu;

    int64_t rampTime;
    int testSamples;

    robot::joystick_command_t command;
};

struct test_sample_t
{
    test_sample_t(const robot::joystick_command_t& command, float angular) : command(command), angularVelocity(angular)
    {
    }

    robot::joystick_command_t command;

    float angularVelocity;
};


void display_help_if_needed(const utils::CommandLine& commandLine);

void run_turn_in_place(test_configuration_t& config, std::ofstream& file);   // added
void run_circle(test_configuration_t& config, std::ofstream& file);          // added
void run_straight(test_configuration_t& config, std::ofstream& file);        // added

void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples);
float estimate_angular_velocity_parameter(const std::vector<test_sample_t>& samples);
void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file);
void load_test_samples(std::ifstream& data, std::vector<test_sample_t>& samples);

/**
 * invacare_atm_parameter_estimator is used to estimate the mapping between joystick position and angular velocity
 * command.
 *
 * To find the parameter, a big chunk of data is consumed. The test runs all commands between -64 and 64 for a given
 * amount of time. Angular velocity is measured from the IMU
 *
 * The command-line parameters for invacare_atm_parameter_estimator are:
 *
 *   --joystick-port   'port'        Port to which wheelchair joystick is attached
 *   --controller-port 'port'        Port to which the wheelchair controller is attached
 *   --imu-port        'port'        Port for the IMU
 *   --ramp-time       'ms'          Milliseconds to wait for the speed to stabilize before beginning to acquire samples
 *   --samples         'number'      Number of samples to acquire for each command
 *   --output-file     'filename'    File in which the data should be written
 *   --parameter-file  'filename'    File from which to load sample data and output the corresponding parameters
 *   --turn-in-place                  Run the turn-in-place test
 *   --circle                         Run the drive in a circle test
 *   --straight                       Run the drive in a straight line test
 */
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    std::vector<test_sample_t> samples;

    if (!commandLine.argumentExists(PARAMETERS)) {
        std::string joystickPort(commandLine.argumentValue(JOYSTICK_PORT));
        std::string controllerPort(commandLine.argumentValue(CONTROLLER_PORT));
        robot::InvacareATM invacare(joystickPort, controllerPort);

        utils::Thread invacareThread;
        invacareThread.attachTask(&invacare);
        invacareThread.start();

        std::string imuPort(commandLine.argumentValue(IMU));
        sensors::Microstrain3DMGX3 imu(imuPort);

        imu.startIMU();

        sleep(1);

        robot::joystick_command_t command(0, 0, 100);

        std::cout << "Ready to run. Switch over to robot control. Waiting 5 seconds...\n";
        sleep(5);

        bool runTurnInPlace = commandLine.argumentExists(TURN_IN_PLACE);
        bool runCircle = commandLine.argumentExists(CIRCLE);
        bool runStraight = commandLine.argumentExists(STRAIGHT);

        int64_t rampTime = atoi(commandLine.argumentValue(RAMP).c_str()) * 1000;
        int testSamples = atoi(commandLine.argumentValue(SAMPLES).c_str());

        test_configuration_t base(invacare, imu, rampTime, testSamples, command);

        std::ofstream samplesFile(commandLine.argumentValue(FILENAME));

        if (runTurnInPlace) {
            run_turn_in_place(base, samplesFile);
        }

        if (runCircle) {
            run_circle(base, samplesFile);
        }

        if (runStraight) {
            run_straight(base, samplesFile);
        }
    } else {
        std::ifstream sampleData(commandLine.argumentValue(PARAMETERS));

        load_test_samples(sampleData, samples);

        float scaleParameter = estimate_angular_velocity_parameter(samples);

        std::cout << "INFO: Finished loading data from " << commandLine.argumentValue(PARAMETERS) << '\n'
                  << "Angular velocity scale parameter:" << scaleParameter << '\n'
                  << "Collected samples are in " << commandLine.argumentValue(FILENAME) << std::endl;
    }

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool helpNeeded =
      (!commandLine.argumentExists(JOYSTICK_PORT) || !commandLine.argumentExists(CONTROLLER_PORT)
       || !commandLine.argumentExists(IMU) || !commandLine.argumentExists(RAMP) || !commandLine.argumentExists(SAMPLES)
       || !(commandLine.argumentExists(TURN_IN_PLACE) || commandLine.argumentExists(CIRCLE)
            || commandLine.argumentExists(STRAIGHT))
       || !commandLine.argumentExists(FILENAME));

    if (helpNeeded) {
        std::cout << "The command-line parameters for quantum_6000_parameter_estimation are:\n"
                  << '\n'
                  << "--joystick-port  'port'        Port to which RS485 connection to joystick exists\n"
                  << "--controller-port  'port'        Port to which RS485 connection to controller exists\n"
                  << "--imu-port    'port'        Port for the IMU\n"
                  << "--ramp-time   'ms'          Milliseconds to wait for the speed to stabilize before beginning to "
                     "acquire samples\n"
                  << "--samples     'number'      Number of samples to acquire for each command\n"
                  << "--output-file 'filename'    File in which the data should be written\n"
                  << "--parameter-file  'filename'    File from which to load sample data and output the corresponding "
                     "parameters\n"
                  << "--turn-in-place             Run the turn-in-place test\n"
                  << "--circle                    Run the drive in a circle test\n"
                  << "--straight                  Run the drive straight test. WARNING!! Static test only!!\n"

                  << std::endl;
        exit(1);
    }
}

void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples)
{
    float sumOfAngularVelocity = 0.0f;

    imu_data_t data;

    std::cout << "INFO: Starting test: Command:(" << (int)config.command.forward << ',' << (int)config.command.left
              << ") Samples:" << config.testSamples << '\n';

    config.invacare.setJoystickCommand(config.command);

    int64_t startTime = utils::system_time_us() + config.rampTime;

    test_sample_t sample(config.command, 0);

    for (int n = config.testSamples; --n >= 0;) {
        config.command.timestamp = utils::system_time_us();

        if (utils::system_time_us() < startTime) {
            usleep(50000);
            ++n;
        } else {
            config.imu.getIMUData(data);

            sample.angularVelocity = data.rotationalVelocity[IMU_YAW_INDEX];

            samples.push_back(sample);

            sumOfAngularVelocity += sample.angularVelocity;
        }

        config.invacare.setJoystickCommand(config.command);
    }

    std::cout << "INFO: Finished test: Average angular:" << (sumOfAngularVelocity / config.testSamples) << "\n\n";
}

float estimate_angular_velocity_parameter(const std::vector<test_sample_t>& samples)
{
    // Simple linear regression to get the value. Right now, the calculation doesn't allow an offset
    // because 0 command = 0 angular velocity
    Vector velocities(samples.size());
    Matrix commands(samples.size(), 1);

    for (int n = samples.size(); --n >= 0;) {
        velocities(n) = samples[n].angularVelocity;
        commands(n, 0) = -samples[n].command.left;
    }

    Vector conversion = arma::solve(commands, velocities);

    return conversion(0);
}

void run_turn_in_place(test_configuration_t& config, std::ofstream& file)
{
    std::cout << "Starting the TURN-IN-PLACE test in 5 seconds. Make sure the area around the robot is clear!\n";
    sleep(5);

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    for (int x = -45; x <= -10; x += 5) {
        config.command.left = -x;
        config.command.forward = 0;

        run_test(config, positiveVelocitySamples);
    }

    for (int x = 45; x >= 10; x -= 5) {
        config.command.left = -x;
        config.command.forward = 0;

        run_test(config, negativeVelocitySamples);
    }

    config.command.forward = 0;
    config.command.left = 0;

    config.invacare.setJoystickCommand(config.command);

    float positiveScale = estimate_angular_velocity_parameter(positiveVelocitySamples);
    float negativeScale = estimate_angular_velocity_parameter(negativeVelocitySamples);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

    std::cout << "INFO: Finished with data collection for the turn-in-place test...\n"
              << "Positive angular velocity scale parameter:" << positiveScale << '\n'
              << "Negative angular velocity scale parameter:" << negativeScale << '\n'
              << std::endl;
}


void run_circle(test_configuration_t& config, std::ofstream& file)
{
    std::cout
      << "Starting the circle test. For this test, a 3-meter circle is needed. At the start of each iteration, the "
         "program "
      << "will pause and wait for the wheelchair to be positioned correctly and safely before beginning the next "
      << "iteration of the test\n";

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    const int FORWARD_VELOCITY = 15;
    const int MAX_ANGULAR = 30;
    const int MIN_ANGULAR = 22;

    for (int x = -MAX_ANGULAR; x <= -MIN_ANGULAR; x += 2) {
        config.command.left = 0;
        config.command.forward = 0;
        config.invacare.setJoystickCommand(config.command);

        config.command.left = -x;
        config.command.forward = FORWARD_VELOCITY;

        std::cout << "Running (forward,left):(" << config.command.forward << ',' << config.command.left << ")\n";
        std::cin.get();

        run_test(config, positiveVelocitySamples);
    }

    for (int x = MAX_ANGULAR; x >= MIN_ANGULAR; x -= 2) {
        config.command.left = 0;
        config.command.forward = 0;
        config.invacare.setJoystickCommand(config.command);

        config.command.left = -x;
        config.command.forward = FORWARD_VELOCITY;

        std::cout << "Running (forward,left):(" << config.command.forward << ',' << config.command.left << ")\n";
        std::cin.get();

        run_test(config, negativeVelocitySamples);
    }

    config.command.left = 0;
    config.command.forward = 0;

    config.invacare.setJoystickCommand(config.command);

    float positiveScale = estimate_angular_velocity_parameter(positiveVelocitySamples);
    float negativeScale = estimate_angular_velocity_parameter(negativeVelocitySamples);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

    std::cout << "INFO: Finished with data collection for the circle test...\n"
              << "Positive angular velocity scale parameter:" << positiveScale << '\n'
              << "Negative angular velocity scale parameter:" << negativeScale << '\n'
              << std::endl;
}

void run_straight(test_configuration_t& config, std::ofstream& file)
{
    std::cout << "Starting the Straight test. For this test the drive wheels must be off the ground!!!\n";

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    const int MAX_FORWARD_VELOCITY = 64;

    for (int y = 0; y <= MAX_FORWARD_VELOCITY; y += 4) {
        config.command.left = 0;
        config.command.forward = 0;
        config.invacare.setJoystickCommand(config.command);

        config.command.left = 0;
        config.command.forward = y;

        std::cout << "Running (forward,left):(" << config.command.forward << ',' << config.command.left << ")\n";
        std::cin.get();

        run_test(config, positiveVelocitySamples);
    }

    config.command.left = 0;
    config.command.forward = 0;

    config.invacare.setJoystickCommand(config.command);

    float positiveScale = estimate_angular_velocity_parameter(positiveVelocitySamples);
    float negativeScale = estimate_angular_velocity_parameter(negativeVelocitySamples);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

    std::cout << "INFO: Finished with data collection for the circle test...\n"
              << "Positive angular velocity scale parameter:" << positiveScale << '\n'
              << "Negative angular velocity scale parameter:" << negativeScale << '\n'
              << std::endl;
}

void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file)
{
    for (int n = samples.size(); --n >= 0;) {
        file << samples[n].command.forward << ' ' << samples[n].command.left << ' ' << samples[n].angularVelocity
             << '\n';
    }
}


void load_test_samples(std::ifstream& data, std::vector<test_sample_t>& samples)
{
    while (!data.eof()) {
        robot::joystick_command_t command;
        float angularVelocity;
        data >> command.forward >> command.left >> angularVelocity;
        samples.push_back(test_sample_t(command, angularVelocity));
    }
}
