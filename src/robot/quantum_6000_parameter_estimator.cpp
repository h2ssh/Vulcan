/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include "core/matrix.h"
#include "core/vector.h"
#include "sensors/microstrain_3dmgx2.h"
#include "robot/quantum_6000.h"
#include "utils/command_line.h"
#include "utils/timestamp.h"


using namespace vulcan;


const std::string JOYSTICK_PORT("joystick-port");
const std::string CONTROLLER_PORT("controller-port");
const std::string IMU("imu-port");
const std::string RAMP("ramp-time");
const std::string SAMPLES("samples");
const std::string FILENAME("output-file");
const std::string TURN_IN_PLACE("turn-in-place");
const std::string CIRCLE("circle");
const std::string CIRCLE_XY("circle_xy");
const std::string CIRCLE_LINEAR("circle_linear");


struct test_configuration_t
{
    test_configuration_t(robot::Quantum6000&         quantum,
                         sensors::Microstrain3DMGX2& imu,
                         int64_t                     ramp,
                         int                         samples,
                         robot::joystick_command_t&  command)
    : quantum(quantum)
    , imu(imu)
    , rampTime(ramp)
    , testSamples(samples)
    , command(command)
    {
    }

    robot::Quantum6000&       quantum;
    sensors::Microstrain3DMGX2& imu;

    int64_t rampTime;
    int     testSamples;

    robot::joystick_command_t command;
};

struct test_sample_t
{
    test_sample_t(const robot::joystick_command_t& command, float angular)
    : command(command)
    , angularVelocity(angular)
    {
    }

    robot::joystick_command_t command;

    float angularVelocity;
};


void display_help_if_needed(const utils::CommandLine& commandLine);

void run_turn_in_place(test_configuration_t& config, std::ofstream& file);
void run_circle       (test_configuration_t& config, std::ofstream& file);
void run_circleXY     (test_configuration_t& config, std::ofstream& file);
void run_circlelinear (test_configuration_t& config, std::ofstream& file);

void  run_test(test_configuration_t& config, std::vector<test_sample_t>& samples);
float estimate_angular_velocity_parameter(const std::vector<test_sample_t>& samples);
void  save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file);


/**
* quantum_6000_parameter_estimator is used to estimate the mapping between joystick position and angular velocity command.
*
* To find the parameter, a big chunk of data is consumed. The test runs all commands between -100 and 100 for a given amount
* of time. Angular velocity is measured from the IMU
*
* 10-26-2012 Ron Gaynier -- The controller within the Quantum 6000 is manufactured by Curtis Instruments. The Curtis documentation on the controller
* reveals there are several selectable parameters that influence the behavior of X (angular velocity) relative to Y (forward velocity).
* In order to better understand this relationship of X at various values of Y, and additional test has been added, CIRCLE_XY. This
* test is only intended to be run while the wheelchair drive wheels are off the ground and with an additional PC running the Curtis
* PC Programmer and logging X Y and Left Right RPMs. The function that accomplishes this is run_circleXY. The function
* is a hack of run_circle and I hope I've commented it sufficiently for anyone to understand how it is suppose to work.
*
* The command-line parameters for quantum_6000_parameter_estimation are:
*
*   --joystick-port    'port'        CANBus port to which the joystick is connected
*   --controller-port  'port'        CANBus port to which the controller is connected
*   --imu-port         'port'        Port for the IMU
*   --ramp-time        'ms'          Milliseconds to wait for the speed to stabilize before beginning to acquire samples
*   --samples          'number'      Number of samples to acquire for each command
*   --output-file      'filename'    File in which the data should be written
*   --turn-in-place                  Run the turn-in-place test
*   --circle                         Run the drive in a circle test
*   --circle_xy                      Run the drive in a circle ramping X for various values of Y
*   --circle_linear                  Run the drive in a circle ramping X and Y together from 0 to 100 
*/
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    std::string joystickPort(commandLine.argumentValue(JOYSTICK_PORT));
    std::string controllerPort(commandLine.argumentValue(CONTROLLER_PORT));
    robot::Quantum6000 quantum(controllerPort, joystickPort);

    utils::Thread quantumThread;
    quantumThread.attachTask(&quantum);
    quantumThread.start();

    std::string imuPort(commandLine.argumentValue(IMU));
    sensors::Microstrain3DMGX2 imu(imuPort);

    imu.startIMU();

    sleep(1);

    robot::joystick_command_t command(0, 0, 100);

    std::cout<<"Ready to run. Switch over to robot control. Waiting 5 seconds...\n";
    sleep(5);

    bool runTurnInPlace  = commandLine.argumentExists(TURN_IN_PLACE);
    bool runCircle       = commandLine.argumentExists(CIRCLE);
    bool runCircleXY     = commandLine.argumentExists(CIRCLE_XY);
    bool runCircleLinear = commandLine.argumentExists(CIRCLE_LINEAR);

    int64_t rampTime    = atoi(commandLine.argumentValue(RAMP).c_str())*1000;
    int     testSamples = atoi(commandLine.argumentValue(SAMPLES).c_str());

    test_configuration_t base(quantum, imu, rampTime, testSamples, command);

    std::ofstream samplesFile(commandLine.argumentValue(FILENAME));

    if(runTurnInPlace)
    {
        run_turn_in_place(base, samplesFile);
    }

    if(runCircle)
    {
        run_circle(base, samplesFile);
    }

    if(runCircleXY)
    {
        run_circleXY(base, samplesFile);
    }

    if(runCircleLinear)
    {
        run_circlelinear(base, samplesFile);
    }

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool helpNeeded = !commandLine.argumentExists(JOYSTICK_PORT)    ||
                      !commandLine.argumentExists(CONTROLLER_PORT)  ||
                      !commandLine.argumentExists(IMU)              ||
                      !commandLine.argumentExists(RAMP)             ||
                      !commandLine.argumentExists(SAMPLES)          ||
                      !commandLine.argumentExists(FILENAME)         ||
                      !(commandLine.argumentExists(TURN_IN_PLACE) ||
                        commandLine.argumentExists(CIRCLE)        ||
                        commandLine.argumentExists(CIRCLE_XY)     ||
                        commandLine.argumentExists(CIRCLE_LINEAR));

    if(helpNeeded)
    {
        std::cout<<"The command-line parameters for quantum_6000_parameter_estimation are:\n"
                 <<'\n'
                 <<"--joystick-port    'port'        CANBus port to which the joystick is connected\n"
                 <<"--controller-port  'port'        CANBus port to which the controller is connected\n"
                 <<"--imu-port         'port'        Port for the IMU\n"
                 <<"--ramp-time        'ms'          Milliseconds to wait for the speed to stabilize before beginning to acquire samples\n"
                 <<"--samples          'number'      Number of samples to acquire for each command\n"
                 <<"--output-file      'filename'    File in which the data should be written\n"
                 <<"--turn-in-place                  Run the turn-in-place test\n"
                 <<"--circle                         Run the drive in a circle test\n"
                 <<"--circle_xy                      Run the drive in a circle ramping X for various values of Y\n"
                 <<std::endl;
        exit(1);
    }
}


void run_turn_in_place(test_configuration_t& config, std::ofstream& file)
{
    std::cout<<"Starting the TURN-IN-PLACE test in 5 seconds. Make sure the area around the robot is clear!\n";
    sleep(5);

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    for(int x = -50; x <= -10; x += 2)
    {
        config.command.left = -x;
        config.command.forward = 0;

        run_test(config, positiveVelocitySamples);
    }

    for(int x = 50; x >= 10; x -= 2)
    {
        config.command.left = -x;
        config.command.forward = 0;

        run_test(config, negativeVelocitySamples);
    }

    config.command.left = 0;
    config.command.forward = 0;

    config.quantum.setJoystickCommand(config.command);

    float positiveScale = estimate_angular_velocity_parameter(positiveVelocitySamples);
    float negativeScale = estimate_angular_velocity_parameter(negativeVelocitySamples);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

    std::cout<<"INFO: Finished with data collection for the turn-in-place test...\n"
             <<"Positive angular velocity scale parameter:"<<positiveScale<<'\n'
             <<"Negative angular velocity scale parameter:"<<negativeScale<<'\n'
             <<std::endl;
}


void run_circle(test_configuration_t& config, std::ofstream& file)
{
    std::cout<<"Starting the circle test. For this test, a 3-meter circle is needed. At the start of each iteration, the program "
             <<"will pause and wait for the wheelchair to be positioned correctly and safely before beginning the next "
             <<"iteration of the test\n";

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    const int FORWARD_VELOCITY = 15;
    const int MAX_ANGULAR      = 30;
    const int MIN_ANGULAR      = 22;

    for(int x = -MAX_ANGULAR; x <= -MIN_ANGULAR; x += 2)
    {
        config.command.left = 0;
        config.command.forward = 0;
        config.quantum.setJoystickCommand(config.command);

        config.command.left = -x;
        config.command.forward = FORWARD_VELOCITY;

        std::cout<<"Running (forward,left):("<<config.command.forward<<','<<config.command.left<<")\n";
        std::cin.get();

        run_test(config, positiveVelocitySamples);
    }

    for(int x = MAX_ANGULAR; x >= MIN_ANGULAR; x -= 2)
    {
        config.command.left = 0;
        config.command.forward = 0;
        config.quantum.setJoystickCommand(config.command);

        config.command.left = -x;
        config.command.forward = FORWARD_VELOCITY;

        std::cout<<"Running (forward,left):("<<config.command.forward<<','<<config.command.left<<")\n";
        std::cin.get();

        run_test(config, negativeVelocitySamples);
    }

    config.command.left = 0;
    config.command.forward = 0;

    config.quantum.setJoystickCommand(config.command);

    float positiveScale = estimate_angular_velocity_parameter(positiveVelocitySamples);
    float negativeScale = estimate_angular_velocity_parameter(negativeVelocitySamples);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

    std::cout<<"INFO: Finished with data collection for the circle test...\n"
             <<"Positive angular velocity scale parameter:"<<positiveScale<<'\n'
             <<"Negative angular velocity scale parameter:"<<negativeScale<<'\n'
             <<std::endl;
}


void run_test(test_configuration_t& config, std::vector<test_sample_t>& samples)
{
    float sumOfAngularVelocity = 0.0f;

    imu_data_t data;

    std::cout<<"INFO: Starting test: Command:("<<(int)config.command.forward<<','<<(int)config.command.left<<") Samples:"<<config.testSamples<<'\n';

    config.quantum.setJoystickCommand(config.command);

    int64_t startTime = utils::system_time_us() + config.rampTime;

    test_sample_t sample(config.command, 0);

    for(int n = config.testSamples; --n >= 0;)
    {
        config.command.timestamp = utils::system_time_us();

        if(utils::system_time_us() < startTime)
        {
            usleep(50000);
            ++n;
        }
        else
        {
            config.imu.getIMUData(data);

            // The IMU on the Quantum6000 is mounted so turning left shows a negative angular velocity, so the sign needs to be flipped
            sample.angularVelocity = -data.rotationalVelocity[IMU_YAW_INDEX];

            samples.push_back(sample);

            sumOfAngularVelocity += sample.angularVelocity;
        }

        config.quantum.setJoystickCommand(config.command);
    }

    std::cout<<"INFO: Finished test: Average angular:"<<(sumOfAngularVelocity / config.testSamples)<<"\n\n";
}


float estimate_angular_velocity_parameter(const std::vector<test_sample_t>& samples)
{
    // Simple linear regression to get the value. Right now, the calculation doesn't allow an offset
    // because 0 command = 0 angular velocity
    Vector velocities(samples.size());
    Matrix commands(samples.size(), 1);

    for(int n = samples.size(); --n >= 0;)
    {
        velocities(n)  = samples[n].angularVelocity;
        commands(n, 0) = -samples[n].command.left;
    }

    Vector conversion = arma::solve(commands, velocities);

    return conversion(0);
}


void save_test_samples(const std::vector<test_sample_t>& samples, std::ofstream& file)
{
    for(int n = samples.size(); --n >= 0;)
    {
        file<<samples[n].command.forward<<' '<<samples[n].command.left<<' '<<samples[n].angularVelocity<<'\n';
    }
}


void run_circleXY(test_configuration_t& config, std::ofstream& file)
{
    std::cout<<"This is a special version of the circle test to evaluate the Curtis control algorithm implementation"
			 <<"of the relationship of forward and angular velocities, particularly how the gain term for X varies as"
		     <<"a function of forward velocity, therefore the ramp-time argument is ignored since we want to log all data."
             <<"It will pause briefly between each test to allow sections of zero rpm to be recorded by the PC Programmer"
			 <<"and thus provide a delimiter between tests."
             <<"\n";

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    const int MAX_FORWARD_VELOCITY = 100;
    const int MIN_FORWARD_VELOCITY = 5;
    const int MAX_ANGULAR      = 50;
    const int MIN_ANGULAR      = 5;
    const int64_t DELAY100MS = 100000;

    config.command.left = 0;
    config.command.forward = 0;

    config.quantum.setJoystickCommand(config.command);

	// After setting the XY command to zero/zero intially, from MIN_FORWORD_VELOCITY conduct a set of tests for increasing Y.
	// Y value is increased by 2 point increments until MAX_FORWARD_VELOCITY is reached

    for(int y = MIN_FORWARD_VELOCITY; y <= MAX_FORWARD_VELOCITY; y += 2)
    {
	// Before each new test set the XY to zero/zero
      config.command.left = 0;
      config.command.forward = 0;
      config.quantum.setJoystickCommand(config.command);
      sleep(1);

	// For the value of Y established in the for loop above, ramp x from the Minimum Angular Velocity to the Max Angular Velocity
      for(int x = -MIN_ANGULAR; x >= -MAX_ANGULAR; x -= 1)
      {

        config.command.left = -x;
        config.command.forward = y;

		// Hold the values of XY for 100ms before changing
        int64_t CommandInterval = utils::system_time_us() + DELAY100MS;

        	config.command.timestamp = utils::system_time_us();
         	config.quantum.setJoystickCommand(config.command);
        	std::cout<<"Command sent (forward,left):("<<config.command.forward<<','<<config.command.left<<")\n";
		while(utils::system_time_us() < CommandInterval)
		{
		// Do nothing
		}

      }
	// Go back to the neural position for one second.
      config.command.left = 0;
      config.command.forward = 0;
      config.quantum.setJoystickCommand(config.command);
      sleep(1);

	// Do the same thing as in the for loop just proceeding, but now in the opposite direction, to determine if there a difference
	// between left and right turns.
      for(int x = MIN_ANGULAR; x <= MAX_ANGULAR; x += 1)
      {

        config.command.left = -x;
        config.command.forward = y;

		// Hold the values of XY for 100ms before changing
        int64_t CommandInterval = utils::system_time_us() + DELAY100MS;

        	config.command.timestamp = utils::system_time_us();
         	config.quantum.setJoystickCommand(config.command);
        	std::cout<<"Command sent (forward,left):("<<config.command.forward<<','<<config.command.left<<")\n";

		while(utils::system_time_us() < CommandInterval)
		{
		// Do nothing
		}

      }

    }

	// This is leftover code from the original function that I copied and hacked
	// Besides setting XY back to zero/zero, the rest is probably rendered moot.
    config.command.left = 0;
    config.command.forward = 0;

    config.quantum.setJoystickCommand(config.command);

    float positiveScale = estimate_angular_velocity_parameter(positiveVelocitySamples);
    float negativeScale = estimate_angular_velocity_parameter(negativeVelocitySamples);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

	std::cout<<"INFO: Finished test runs for the circle_xy test...\n"
             <<"Positive angular velocity scale parameter:"<<positiveScale<<'\n'
             <<"Negative angular velocity scale parameter:"<<negativeScale<<'\n'
             <<std::endl;
}


void run_circlelinear(test_configuration_t& config, std::ofstream& file)
{
    std::cout<<"This is a special version of the circle test to evaluate the Curtis control algorithm implementation" 
			 <<"of the relationship of forward and angular velocities, when X and Y ramp together"
			 <<"at the same rate from zero to their maximum values."
             <<"\n";

    std::vector<test_sample_t> positiveVelocitySamples;
    std::vector<test_sample_t> negativeVelocitySamples;

    const int64_t DELAY7000MS = 7000000;
    int x = 0;
    int y = 0;

    config.command.left = 0;
    config.command.forward = 0;
    config.command.timestamp = utils::system_time_us();

    config.quantum.setJoystickCommand(config.command);
    sleep(1);

	// After setting the XY command to zero/zero intially, from MIN_FORWORD_VELOCITY conduct a set of tests for increasing Y.
	// Y value is increased by 2 point increments until MAX_FORWARD_VELOCITY is reached

     for(y = 0; y <= 100; y += 10)
     {
       for(x = 0; x <= 100; x += 10)
       {

        config.command.left = -x;
        config.command.forward = y;

		// Hold the values of XY for 100ms before changing
        int64_t CommandInterval = utils::system_time_us() + DELAY7000MS;

        	config.command.timestamp = utils::system_time_us();
         	config.quantum.setJoystickCommand(config.command);
        	std::cout<<"Command sent (forward,left):("<<config.command.forward<<','<<config.command.left<<")\n";
		while(utils::system_time_us() < CommandInterval)
		{
        	config.command.left = -x;
        	config.command.forward = y;
        	config.command.timestamp = utils::system_time_us();
         	config.quantum.setJoystickCommand(config.command);
		}

     	config.command.left = 0;
     	config.command.forward = 0;
     	config.command.timestamp = utils::system_time_us();

     	config.quantum.setJoystickCommand(config.command);
     	sleep(3);
        }
      }
	// Go back to the neural position for one second.
      config.command.left = 0;
      config.command.forward = 0;
      config.command.timestamp = utils::system_time_us();
      config.quantum.setJoystickCommand(config.command);
      sleep(1);


	// This is leftover code from the original function that I copied and hacked
	// Besides setting XY back to zero/zero, the rest is probably rendered moot.
    config.command.left = 0;
    config.command.forward = 0;

    config.quantum.setJoystickCommand(config.command);

    save_test_samples(positiveVelocitySamples, file);
    save_test_samples(negativeVelocitySamples, file);

	std::cout<<"INFO: Finished test runs for the circle_xy test...\n"
             <<std::endl;
}

