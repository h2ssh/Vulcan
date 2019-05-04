/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <boost/shared_ptr.hpp>
#include <system/module_communicator.h>
#include <sensors/camera.h>
#include <core/image.h>
#include <vision/image_utils.h>
#include <sensors/avt_camera.h>
#include <utils/command_line.h>
#include <utils/config_file.h>
#include <utils/timestamp.h>


using namespace vulcan;

// Command-line arguments
const std::string HELP_SHORT("h");
const std::string HELP_LONG("help");
const std::string MODEL("camera-model");
const std::string CONFIG("config-file");
const std::string CHANNEL("channel");
const std::string BASENAME("output-basename");
const std::string PERIOD("publish-period-ms");

const std::string CAMERA_MODEL_AVT("avt");

// Default values for the command-lines
const std::string DEFAULT_CAMERA_MODEL(CAMERA_MODEL_AVT);
const std::string DEFAULT_CHANNEL("SENSOR_IMAGE");
const std::string DEFAULT_CONFIG("image_producer.cfg");
const std::string DEFAULT_PERIOD("0");


void display_help_if_needed(const utils::CommandLine& commandLine);

boost::shared_ptr<sensors::Camera> load_camera    (const utils::CommandLine& commandLine, const utils::ConfigFile& config);
boost::shared_ptr<sensors::Camera> load_avt_camera(const utils::ConfigFile& config);

boost::shared_ptr<system::ModuleCommunicator> load_data_transmitter(void);

void produce_images(boost::shared_ptr<sensors::Camera> camera,
                    boost::shared_ptr<system::ModuleCommunicator> transmitter,
                    const std::string& channel,
                    const std::string& baseFilename = "",
                    int64_t            period = 0);


/**
* image_producer is a program that produces camera images. The images that are captured are sent out on the specified
* channel to all subscribers. It should be noted that camera images can be large, beastly creations that suck all the
* CPU and bandwidth just through transmission. This problem is something to keep in mind when using the image_producer.
* If there is only a single image processing module, it might make more sense to create a local Camera instance and
* capture the images within the module. Of course, to visualize, they'll need to be transmitted, so perhaps you don't
* gain anything.
*
* The command-line arguments for the image_producer are:
*
*   -h, --help                  Display the help message
*   --camera-model 'model'      Model of camera being used : Default = avt
*           Currently supported cameras:
*               avt
*   --config-file 'filename'    Filename with the specific settings for the camera (optional)
*   --channel 'name'            Channel name on which to publish the images : Default = SENSOR_IMAGE
*   --output-basename 'name'    Base name of output images. Images will be written in format: basenameXXXX.ppm/pgm
*   --publish-period-ms 'ms'    Number of milliseconds between publishing frames (optional, default = 0, i.e. all frames)
*/
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    utils::ConfigFile config(commandLine.argumentValue(CONFIG, DEFAULT_CONFIG));

    boost::shared_ptr<sensors::Camera> camera = load_camera(commandLine, config);

    boost::shared_ptr<system::ModuleCommunicator> transmitter = load_data_transmitter();

    std::string channel  = commandLine.argumentValue(CHANNEL, DEFAULT_CHANNEL);
    std::string basename = commandLine.argumentValue(BASENAME);
    int64_t     period   = std::atol(commandLine.argumentValue(PERIOD, DEFAULT_PERIOD).c_str()) * 1000l;

    produce_images(camera, transmitter, channel, basename, period);

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool helpNeeded = commandLine.argumentExists(HELP_SHORT) ||
                      commandLine.argumentExists(HELP_LONG);

    if(helpNeeded)
    {
        std::cout<<"The command-line arguments for the image_producer are:\n"
                 <<'\n'
                 <<"-h, --help                  Display the help message\n"
                 <<"--camera-model 'model'      Model of camera being used  Default:avt\n"
                 <<"           Currently supported cameras:\n"
                 <<"               avt\n"
                 <<"   --config-file 'filename'    Filename with the specific settings for the camera (optional)\n"
                 <<"   --channel 'name'            Channel name on which to publish the images\n"
                 <<"   --output-basename 'name'    Base name of output images. Images will be written in format: basenameXXXX.png"
                 <<"   --publish-period-ms 'ms'    Number of milliseconds between publishing frames (optional, default = 0, i.e. all frames)"
                 <<'\n';
        exit(1);
    }
}


boost::shared_ptr<sensors::Camera> load_camera(const utils::CommandLine& commandLine, const utils::ConfigFile& config)
{
    std::string cameraModel = commandLine.argumentValue(MODEL, DEFAULT_CAMERA_MODEL);

    if(cameraModel == CAMERA_MODEL_AVT)
    {
        return load_avt_camera(config);
    }
    else
    {
        std::cerr<<"ERROR: Unknown camera model: "<<cameraModel<<'\n';
        assert(false);
    }
}


boost::shared_ptr<sensors::Camera> load_avt_camera(const utils::ConfigFile& config)
{
    sensors::initialize_pvapi_library(true);

    sensors::avt_camera_parameters_t params = sensors::load_avt_camera_parameters_from_config_file(config);

    return boost::shared_ptr<sensors::Camera>(new sensors::AVTCamera(params));
}


boost::shared_ptr<system::ModuleCommunicator> load_data_transmitter(void)
{
    return boost::shared_ptr<system::ModuleCommunicator>(new system::ModuleCommunicator());
}


void produce_images(boost::shared_ptr<sensors::Camera> camera,
                    boost::shared_ptr<system::ModuleCommunicator> transmitter,
                    const std::string& channel,
                    const std::string& baseFilename,
                    int64_t            period)
{
    Image image;

    camera->startCapture();

    int64_t startTime  = utils::system_time_us();
    int imagesCaptured = 0;

    int imageNumber = 0;

    std::ofstream timeLog;

    if(!baseFilename.empty())
    {
        timeLog.open((baseFilename + "_timestamps.log").c_str());
    }

    int64_t lastPublishTime = utils::system_time_us();

    while(true)
    {
        // If capture fails, then restart the camera and keep going
        // Hack needed for Point Grey flyCapture, that occasionally gets stuck
        if(!camera->getNextImage(image))
        {
            camera->stopCapture();
            camera->startCapture();
            continue;
        }

        if(utils::system_time_us() - lastPublishTime > period)
        {
            transmitter->sendMessage(image, channel);
            lastPublishTime = utils::system_time_us();
        }

        ++imagesCaptured;

        if(utils::system_time_us() - startTime > utils::sec_to_usec(1))
        {
            std::cout<<"Image frames per second: "<<imagesCaptured<<'\n';

            imagesCaptured = 0;
            startTime      = utils::system_time_us();
        }

        if(!baseFilename.empty())
        {
            std::ostringstream filename;
            if(image.getColorspace() == RGB)
            {
                filename<<baseFilename<<std::setw(4)<<std::setfill('0')<<imageNumber<<".ppm";
            }
            else
            {
                filename<<baseFilename<<std::setw(4)<<std::setfill('0')<<imageNumber<<".pgm";
            }

            timeLog<<imageNumber<<' '<<image.getTimestamp()<<std::endl;

            ++imageNumber;

            vision::save_image_to_file(image, filename.str());
        }
    }

    camera->stopCapture();
}
