/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file   simulator_utils.cpp
* \author Zongtai Luo
* Simulator_utils contains some useful functions for genrating and sending sensor data.
*/

#include "math/geometry/rectangle.h"
#include "sensors/laser_rangefinder.h"
#include "laser/laser_io.h"
#include "core/odometry.h"
#include "utils/cell_grid_utils.h"
#include "utils/command_line.h"
#include "hssh/local_metric/lpm_io.h"
#include "simulator/simulator_utils.h"

namespace vulcan{

namespace sim{


simulator_set_up_t init_simulator_params(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    utils::CommandLine commandLine(argc, argv, arguments);

    simulator_set_up_t simulator_settings;
    simulator_settings.mode = commandLine.argumentValue("m");
    simulator_settings.case_no = std::stoi(commandLine.argumentValue("c"));
    
    if (simulator_settings.mode.empty())
    {
        simulator_settings.mode = "notslam";
    }

    if (commandLine.argumentValue("c").empty())
    {
        simulator_settings.case_no = 1; // default case 1
    }

    return simulator_settings;
}


void Map_init(std::string& map_path, hssh::LocalPerceptualMap& mylpm, int32_t Id) 
{
    hssh::load_lpm_1_0(map_path, mylpm);
    mylpm.setId(Id);
    printf("%s\n", "Map has been loaded.");
}


int find_range(std::vector<Point<double>>& endpointsgrid,
               std::vector<Point<double>>& endpointsglobal,
               Point<double> startgrid,
               Point<double> startglobal,
               utils::ray_trace_range_t& fake_ray_shot,
               polar_laser_scan_t& fake_polar_laser_scan,
               hssh::LocalPerceptualMap& lpm) {

    // double maxdist = 40.0;
    endpointsgrid = utils::trace_range_until_condition(startgrid,
                    fake_ray_shot,
                    fake_polar_laser_scan.maxRange,
                    lpm,
                    [](const hssh::LocalPerceptualMap& lpm, Point<int> cell) {
                        return lpm.getCellType(cell.x, cell.y) & (hssh::kUnsafeOccGridCell | hssh::kUnobservedOccGridCell);
                    });

    Point<double> endpointglobalbuffer;
    Point<double> endpointgridbuffer;

    for (unsigned i=0; i < (endpointsgrid.size()-1); i++) {
        endpointgridbuffer = endpointsgrid[i];
        endpointglobalbuffer = utils::grid_point_to_global_point(endpointgridbuffer,lpm); //Transfer the endpoints to global coords
        endpointsglobal.push_back(endpointglobalbuffer);
        fake_polar_laser_scan.ranges.push_back(distance_between_points(startglobal,endpointsglobal[i]));
    };
    return 0;
}


// Send the laser scan at the current pose
std::vector<Point<double>> flaser_scan_producer(pose_t& vulcan_one_state,
                                                      hssh::LocalPerceptualMap mylpm,
                                                      int64_t timestamp,
                                                      system::ModuleCommunicator& connector,
                                                      float maxRange_,
                                                      bool send_to_robot,
                                                      double angle_range,
                                                      float angularResolution,
                                                      double start_offset) {

    // Produce the laser scan with offset 0 and 360 degrees with resolution of 0.5
    polar_laser_scan_t fake_polar_laser_scan;
    fake_polar_laser_scan.maxRange = maxRange_;
    fake_polar_laser_scan.laserId = 1;
    fake_polar_laser_scan.angularResolution = angularResolution;
    fake_polar_laser_scan.timestamp = timestamp;
    fake_polar_laser_scan.scanId = timestamp;
    utils::ray_trace_range_t fake_ray_shot(vulcan_one_state.theta+start_offset, 
                                           std::size_t(angle_range/angularResolution),
                                           fake_polar_laser_scan.angularResolution);
    fake_ray_shot.range = angle_range; // Angle range of tracing

    // Calculating the range with known robot position
    Point<double> startglobal(vulcan_one_state.x,vulcan_one_state.y);
    Point<double> startgrid;

    startgrid = utils::global_point_to_grid_point(startglobal,mylpm); //transfer the global coords to grid coords
    std::vector<Point<double>> endpointsglobal;
    std::vector<Point<double>> endpointsgrid;

    // find the ridar range
    find_range(endpointsgrid,endpointsglobal,startgrid,startglobal,fake_ray_shot,fake_polar_laser_scan,mylpm);

    // Sending the range information back to control
    fake_polar_laser_scan.numRanges = endpointsgrid.size();

    if (send_to_robot)
    {
        connector.sendMessage(fake_polar_laser_scan,"SENSOR_LASER_FRONT_6DOF");
        // printf("%s \n", "laser scan sent.");
    }

    return endpointsglobal;
    
}


// Send fake encoder signal to robot
void fencoder_producer(int64_t& leftTicksTotal,
                       int64_t& rightTicksTotal,
                       motion_state_t vulcan_state,
                       motion_state_t vulcan_former_state,
                       sensors::wheel_encoders_params_t& wheel_encoder_params,
                       int64_t timestamp,
                       system::ModuleCommunicator& connector) {

    const float WHEEL_CIRCUMFERENCE  = wheel_encoder_params.configuration.rightWheelCircumference;
    const float TICKS_PER_REVOLUTION = wheel_encoder_params.configuration.rightTicksPerRevolution;
    const float Wheelbase = wheel_encoder_params.configuration.wheelbase;

    float deltat = (vulcan_state.timestamp-vulcan_former_state.timestamp)/1000;
    
    // Create the encoder data
    encoder_data_t fake_encoder_data;
    fake_encoder_data.wheelbase = Wheelbase;
    fake_encoder_data.rightWheelCircumference = fake_encoder_data.leftWheelCircumference = WHEEL_CIRCUMFERENCE;
    fake_encoder_data.rightTicksPerRevolution = fake_encoder_data.leftTicksPerRevolution = TICKS_PER_REVOLUTION;

    // Getting delta and RPM (deafult in radius instead of degree in angle)
    float theta_former = vulcan_former_state.pose.theta;
    float theta_current = vulcan_state.pose.theta;
    float d_center = (vulcan_state.pose.x - vulcan_former_state.pose.x)/std::cos(theta_former);

    // avoid nan
    if (d_center > 10000){
        d_center = (vulcan_state.pose.y - vulcan_former_state.pose.y)/std::sin(theta_former);
    }

    fake_encoder_data.deltaLeftWheel = (2*d_center-(theta_current-theta_former)*Wheelbase)/2;
    fake_encoder_data.deltaRightWheel = (2*d_center+(theta_current-theta_former)*Wheelbase)/2;
    fake_encoder_data.leftRPM = fake_encoder_data.deltaLeftWheel/deltat/WHEEL_CIRCUMFERENCE*60;
    fake_encoder_data.rightRPM = fake_encoder_data.deltaRightWheel/deltat/WHEEL_CIRCUMFERENCE*60;
    // printf("Start encoder part, The former theta is %f and the wheel base is %f\n", theta_former, Wheelbase); // checking whether in degree or radius

    // Generate encoder data and send to robot
    // Calculate Ticks for each wheel
    int32_t thisframeticks_left = fake_encoder_data.deltaLeftWheel/WHEEL_CIRCUMFERENCE * TICKS_PER_REVOLUTION;
    int32_t thisframeticks_right = fake_encoder_data.deltaRightWheel/WHEEL_CIRCUMFERENCE*TICKS_PER_REVOLUTION;

    // Cumulates the new ticks to the total ticks
    fake_encoder_data.leftTicksTotal = leftTicksTotal + thisframeticks_left;
    fake_encoder_data.rightTicksTotal = rightTicksTotal + thisframeticks_right;
    leftTicksTotal = fake_encoder_data.leftTicksTotal;
    rightTicksTotal = fake_encoder_data.rightTicksTotal;

    // Set the timestamp and id
    fake_encoder_data.timestamp = timestamp;
    fake_encoder_data.id = timestamp;

    // printf("Current state of robot is in %f, %f, %f\n", vulcan_state.pose.x,vulcan_state.pose.y ,vulcan_state.pose.theta);
    // printf("Former state of robot is in %f, %f, %f\n", vulcan_former_state.pose.x,vulcan_former_state.pose.y ,vulcan_former_state.pose.theta);
    // printf("%s the delta of left wheel is %f and right wheel is %f\n", "encoder data sent.", fake_encoder_data.deltaLeftWheel, fake_encoder_data.deltaRightWheel);

    // sleep(1);

    // Send the encoder data to the robot
    connector.sendMessage(fake_encoder_data);
    // printf("Encoder data sent.\n");
}


// Odometry updated according to the difference between two states
void fodometry_producer(motion_state_t& vulcan_state,
                        motion_state_t& vulcan_former_state,
                        int64_t timestamp,
                        system::ModuleCommunicator& connector) {

    odometry_t vulcan_odometry;
    vulcan_odometry.timestamp = timestamp;
    vulcan_odometry.id = timestamp;
    vulcan_odometry.x = vulcan_state.pose.x;
    vulcan_odometry.y = vulcan_state.pose.y;
    vulcan_odometry.theta = vulcan_state.pose.theta;
    Point<double> robot_point_now(vulcan_state.pose.x,vulcan_state.pose.y);

    if(vulcan_former_state.pose.x == vulcan_state.pose.x && vulcan_former_state.pose.y
            == vulcan_state.pose.y && vulcan_former_state.pose.theta == vulcan_state.pose.theta) {
        vulcan_odometry.translation = 0.0;
        vulcan_odometry.rotation = 0.0;
    }
    else {
        pose_t robot_former_pose = vulcan_former_state.pose;
        Point<double> robot_point_former(robot_former_pose.x,robot_former_pose.y);
        vulcan_odometry.translation = distance_between_points(robot_point_now,robot_point_former);
        vulcan_odometry.rotation = vulcan_state.pose.theta-robot_former_pose.theta;
    }

    // printf("Start odometry part, Current state of robot is in %f, %f, %f\n", vulcan_state.pose.x,vulcan_state.pose.y ,vulcan_state.pose.theta);
    // printf("Former state of robot is in %f, %f, %f\n", vulcan_former_state.pose.x,vulcan_former_state.pose.y ,vulcan_former_state.pose.theta);
    // printf("Translation and rotation are %f %f\n", vulcan_odometry.translation,vulcan_odometry.rotation);
    
    // sleep(1);

    vulcan_former_state = vulcan_state; // recording the former state

    connector.sendMessage(vulcan_odometry);

}


}// sim
}// vulcan