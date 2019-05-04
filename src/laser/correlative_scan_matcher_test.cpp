/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <iostream>
#include <cstdlib>
#include <ctime>

#include <system/module_communicator.h>

#include <utils/command_line.h>
#include <utils/config_file.h>
#include <core/point.h>
#include <utils/timestamp.h>

#include <core/pose.h>

#include <core/multivariate_gaussian.h>
#include <core/vector.h>
#include <core/angle_functions.h>

#include <core/odometry.h>
#include <core/laser_scan.h>

#include <core/laser_scan.h>
#include <laser/scan_matcher_params.h>
#include <laser/correlative_scan_matcher.h>


using vulcan::polar_laser_scan_t;
using vulcan::cartesian_laser_scan_t;
using vulcan::laser::scan_matcher_params_t;
using vulcan::pose_t;


const std::string HELP_SHORT("h");
const std::string HELP_LONG("H");
const std::string PARAMS("params");
const std::string STATIC_DATA("use-static-data");
const std::string DYNAMIC_DATA("use-dynamic-data");
const std::string LASER_CHANNEL("laser-channel");
const std::string ODOMETRY_CHANNEL("odometry-channel");
const std::string POSE_CHANNEL("pose-channel");
const std::string ITERATIONS("iterations");


const int X_INDEX     = 0;
const int Y_INDEX     = 1;
const int THETA_INDEX = 2;


class DataHandler
{
public:

    DataHandler(const std::string& laserChannel, const std::string& odometryChannel, const std::string& poseChannel) :
                                                            laserName(laserChannel), odometryName(odometryChannel), poseName(poseChannel),
                                                            usingStaticData(false)
    {
        communicator.subscribeTo<polar_laser_scan_t>(this);
    }

    void useStaticData(void)
    {
        usingStaticData = true;
    }

    void useDynamicData(void)
    {
        usingStaticData = false;
    }

    void setStaticScan(const polar_laser_scan_t& scan)
    {
        currentScan = scan;

        currentScan.timestamp = 0;
    }

    void handleData(const polar_laser_scan_t& scan, const std::string& channel)
    {
        if(!usingStaticData)
        {
            currentScan = scan;
        }
    }

    void sendOdometry(const vulcan::odometry_t& odom)
    {
        //communicator.sendMessage(odom, odometryName);
    }

    void sendPose(const pose_t& pose)
    {
        communicator.sendMessage(pose, poseName);
    }

    void getLaserScan(polar_laser_scan_t& scan)
    {
        if(!usingStaticData)
        {
            communicator.processIncoming();
        }
        else // need to keep updating the timestamp for the scan, otherwise the volume creation breaks
        {
            currentScan.timestamp += 100000;
        }

        scan = currentScan;
    }

private:

    std::string laserName;
    std::string odometryName;
    std::string poseName;

    bool usingStaticData;

    polar_laser_scan_t currentScan;

    vulcan::system::ModuleCommunicator communicator;
};


// Helpers for doing the running of the scan matcher
void display_help_if_needed(const vulcan::utils::CommandLine& commandLine);
void setup_data_handler(DataHandler& handler, const vulcan::utils::CommandLine& commandLine);
polar_laser_scan_t generate_circular_scan(float range, float angleStart, float angleRange, float angleResolution);
scan_matcher_params_t load_params(const std::string& configFilename);

void run_scan_matcher(DataHandler& handler, const vulcan::laser::scan_matcher_params_t& params, int iterations, bool shouldPerturb);
pose_t generate_random_perturbation(void);
void perturb_scan(cartesian_laser_scan_t& scan, pose_t& transform);

void do_odometry_update(pose_t& pose, const vulcan::Vector& delta);


/**
* correlative_scan_matcher_test is used to test the functionality of the correlative scan matcher.
*
* To do the testing, a sample scan is created and transformed to a new pose. The scan matcher is
* then run. Since the transform is known, the quality of the match is also known.
*
* Alternately, actual scan data can be provided to the scan matcher via the communication system.
* The scan matcher will then run on this data and produce odometry measurements for anything that
* is using the data.
*
* The command-line arguments for the scan matcher are:
*
*   --help/-h                      : display this message
*   --params           'filename'  : location of the parameters file for the scan matcher
*   --use-static-data              : test using a generated laser scan
*   --use-dynamic-data             : test using data produced by another module
*   --laser-channel    'name'      : name of the channel on which to receive laser data
*   --odometry-channel 'name'      : name of the channel on which odometry data will be sent
*   --pose-channel     'name'      : name of the channel on which estimated pose data will be sent
*   --iterations       'numiters'  : number of iterations to run the scan matcher
*/
int main(int argc, char** argv)
{
    srand48(time(0));

    vulcan::utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    DataHandler handler(commandLine.argumentValue(LASER_CHANNEL),
                        commandLine.argumentValue(ODOMETRY_CHANNEL),
                        commandLine.argumentValue(POSE_CHANNEL));

    int iterations = atoi(commandLine.argumentValue(ITERATIONS).c_str());

    setup_data_handler(handler, commandLine);

    scan_matcher_params_t params = load_params(commandLine.argumentValue(PARAMS));

    run_scan_matcher(handler, params, iterations, commandLine.argumentExists(STATIC_DATA));

    return 0;
}


void display_help_if_needed(const vulcan::utils::CommandLine& commandLine)
{
    bool helpNeeded = commandLine.argumentExists(HELP_SHORT) ||
                      commandLine.argumentExists(HELP_LONG)  ||
                      !commandLine.argumentExists(PARAMS);

    if(helpNeeded)
    {
        std::cout<<"correlative_scan_matcher_test:\n"
                <<"Command-line arguments for running a test are: \n"
                <<"--help/-h                      : display this message\n"
                <<"--params           'filename'  : location of the parameters file for the scan matcher\n"
                <<"--use-static-data              : test using a generated laser scan\n"
                <<"--use-dynamic-data             : test using data produced by another module\n"
                <<"--laser-channel    'name'      : name of the channel on which to receive laser data\n"
                <<"--odometry-channel 'name'      : name of the channel on which odometry data will be sent\n"
                <<"--pose-channel     'name'      : name of the channel on which estimated pose data will be sent\n"
                <<"--iterations       'numiters'  : number of iterations to run the scan matcher\n"
                <<std::endl;

        exit(1);
    }
}


void setup_data_handler(DataHandler& handler, const vulcan::utils::CommandLine& commandLine)
{
    if(commandLine.argumentExists(STATIC_DATA))
    {
        handler.useStaticData();

        polar_laser_scan_t staticScan = generate_circular_scan(5.0, -M_PI/2, M_PI, M_PI/720);

        handler.setStaticScan(staticScan);
    }
    else // handler.argumentExists(DYNAMIC_DATA)
    {
        handler.useDynamicData();
    }
}


polar_laser_scan_t generate_circular_scan(float range, float angleStart, float angleRange, float angleResolution)
{
    polar_laser_scan_t scan;

    scan.laserId = 0;

    scan.angularResolution = angleResolution;
    scan.startAngle        = angleStart;
    scan.maxRange          = 30.0;
    scan.timestamp         = 0;
    scan.numRanges         = static_cast<uint16_t>(angleRange/angleResolution) + 1;

    scan.ranges.resize(scan.numRanges);

    for(int x = scan.numRanges; --x >= 0;)
    {
        scan.ranges[x] = range + (drand48()-0.5)/5.0;
    }

    return scan;
}


scan_matcher_params_t load_params(const std::string& configFilename)
{
    vulcan::utils::ConfigFile config(configFilename);

    return vulcan::laser::load_scan_matcher_params(config);
}


void run_scan_matcher(DataHandler& handler, const scan_matcher_params_t& params, int iterations, bool shouldPerturb)
{
    int64_t startTime = 0;
    int64_t endTime   = 0;

    polar_laser_scan_t scan;
    cartesian_laser_scan_t cartesian;

    vulcan::laser::CorrelativeScanMatcher matcher(params);

    handler.getLaserScan(scan);
    vulcan::polar_scan_to_cartesian_scan_in_robot_frame(scan, cartesian, true);
    cartesian.timestamp -= 50;
    matcher.setReferenceScan(cartesian.scanPoints, cartesian.timestamp);

    pose_t estimatedPose;
    pose_t actualPose;

    pose_t perturb;

    int referenceScanCount = 0;

    while(--iterations >= 0)
    {
        handler.getLaserScan(scan);
        vulcan::polar_scan_to_cartesian_scan_in_robot_frame(scan, cartesian, true);

        if(shouldPerturb)
        {
            perturb = generate_random_perturbation();
            perturb_scan(cartesian, perturb);

            std::cout<<"\nPerturbation: ("<<perturb.x<<","<<perturb.y<<","<<perturb.theta<<")\n";
        }

        startTime = vulcan::utils::system_time_us();
        vulcan::MultivariateGaussian distribution = matcher.findMostLikelyTransform(cartesian.scanPoints, cartesian.timestamp);
        endTime = vulcan::utils::system_time_us();

        vulcan::Vector transform = distribution.getMean();

        ++referenceScanCount;

        // Don't update the reference if there has been no motion otherwise small motions will go unnoticed
        if((fabs(transform(X_INDEX)) < 0.0001 && fabs(transform(Y_INDEX)) < 0.0001 && fabs(transform(THETA_INDEX)) < M_PI/36.0f) &&
           referenceScanCount < 1000)
        {
            continue;
        }

        referenceScanCount = 0;

        std::cout<<"Transform: ("<<transform(X_INDEX)<<","<<transform(Y_INDEX)<<","<<transform(THETA_INDEX)<<")\n";

        if(shouldPerturb)
        {
            std::cout<<"Difference: ("<<(transform(X_INDEX)+perturb.x)<<','<<(transform(Y_INDEX)+perturb.y)<<','<<(transform(THETA_INDEX)+perturb.theta)<<')'<<std::endl;
        }

        std::cout<<"Time: "<<(endTime-startTime)/1000<<std::endl;

        do_odometry_update(estimatedPose, transform);

        handler.sendPose(estimatedPose);

        std::cout<<"Estimated Pose: ("<<estimatedPose.x<<','<<estimatedPose.y<<','<<estimatedPose.theta<<")\n";

        if(shouldPerturb)
        {
            transform(X_INDEX) = -perturb.x;
            transform(Y_INDEX) = -perturb.y;
            transform(THETA_INDEX) = -perturb.theta;
            do_odometry_update(actualPose, transform);

            std::cout<<"Actual Pose: ("<<actualPose.x<<','<<actualPose.y<<','<<actualPose.theta<<")\n";
        }
        std::cout<<'\n';

        if(!shouldPerturb)
        {
            matcher.setReferenceScan(cartesian.scanPoints, cartesian.timestamp);
        }
    }
}


pose_t generate_random_perturbation(void)
{
    pose_t pose;

    static int modifier = -1;

    pose.x = 0.05 * modifier;
    pose.y = 0.05 * modifier;
    pose.theta = M_PI/90.0f * modifier;

    modifier *= -1;

    return pose;
}


void perturb_scan(cartesian_laser_scan_t& scan, pose_t& transform)
{
    std::vector<vulcan::Point<float>>& scanPoints = scan.scanPoints;

    for(int i = scan.numPoints; --i >= 0;)
    {
        float tempX = scanPoints[i].x*cos(transform.theta) - scanPoints[i].y*sin(transform.theta);
        float tempY = scanPoints[i].x*sin(transform.theta) + scanPoints[i].y*cos(transform.theta);

        scanPoints[i].x = tempX + transform.x;
        scanPoints[i].y = tempY + transform.y;
    }
}


void do_odometry_update(pose_t& pose, const vulcan::Vector& delta)
{
    pose.x += delta(X_INDEX); //delta.deltaX * cos(delta.deltaTheta/2 + pose.theta);
    pose.y += delta(Y_INDEX); //delta.deltaX * sin(delta.deltaTheta/2 + pose.theta);
    pose.theta = vulcan::angle_sum(pose.theta, delta(THETA_INDEX));
}
