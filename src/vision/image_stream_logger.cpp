/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "system/module_communicator.h"
#include "core/motion_state.h"
#include "utils/command_line.h"
#include "utils/auto_mutex.h"
#include "utils/mutex.h"
#include "utils/pose_trace.h"
#include "utils/repeated_task.h"
#include "core/image.h"
#include "vision/image_utils.h"
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unistd.h>


using namespace vulcan;


const std::string HELP_SHORT("h");
const std::string HELP_LONG("help");

const std::string BASENAME("basename");


class ImageStreamLogger
{
public:

    /**
    * Constructor for ImageStreamLogger.
    *
    * \param    basename            Base filename for the image being saved
    */
    ImageStreamLogger(const std::string& basename)
    : basename(basename)
    , imageNumber(0)
    {
        std::ostringstream filename;
        filename<<basename<<"_timestamps.log";
        timeLog.open(filename.str());

        assert(timeLog.is_open());
    }

    /**
    * handleData deals with image data arriving from the system.
    */
    void handleData(const Image& image, const std::string& channel)
    {
        std::ostringstream filename;
        filename << basename << '-' << imageNumber << ".ppm";
        
        vision::save_image_to_file(image, filename.str());
        
        timeLog << filename.str() << ' ' << image.getTimestamp() << '\n';
        imageTimestamps_.push_back(std::make_pair(filename.str(), image.getTimestamp()));
        ++imageNumber;
        
        std::cout << "Saved " << filename.str() << " at time " << image.getTimestamp() << '\n';
    }
    
    /**
    * handleData acquires the pose information from the robot and saves the timestamps associated with each image as
    * output by the state_estimator.
    */
    void handleData(const motion_state_t& state, const std::string& channel)
    {
        // If pose data is arriving, then open the log and save it
        if(!poseLog.is_open())
        {
            std::ostringstream filename;
            filename << basename << "_poses.log";
            poseLog.open(filename.str());
        }
        
        poses_.addPose(state.pose);
        
        // Wait until a pose has arrived after the image time, so the actual pose can be interpolated.
        while(!imageTimestamps_.empty())
        {
            auto t = imageTimestamps_.front();
            
            // If the image pose can be interpolated, do so
            if(poses_.hasPoseAfterTime(t.second))
            {
                auto pose = poses_.poseAt(t.second);
                poseLog << t.first << ' ' << pose.x << ' ' << pose.y << ' ' << pose.theta << '\n';
                
                imageTimestamps_.pop_front();
            }
            // Otherwise, since the timestamps arrive in order, jump out of the loop so new poses can be read
            // which will be used for stamping this image.
            else
            {
                break;
            }
        }
    }

private:
    
    using ImageTime = std::pair<std::string, int64_t>;

    std::string   basename;
    int           imageNumber;
    std::ofstream timeLog;
    std::ofstream poseLog;
    
    utils::PoseTrace poses_;
    std::deque<ImageTime> imageTimestamps_;
};


void display_help_if_needed(const utils::CommandLine& commandLine);


/**
* image_stream_logger stores images received from the provided channel to sequentially numbered
* files of the form: basenameXXXX.png.
*
* The command line arguments are:
*
*   -h/--help       Display help message                      (optional)
*   --basename      Base filename to use for the saved images (required)
*/
int main(int argc, char** argv)
{
    system::ModuleCommunicator receiver;
    utils::CommandLine commandLine(argc, argv);

    display_help_if_needed(commandLine);

    std::string basename = commandLine.argumentValue(BASENAME);

    ImageStreamLogger logger(basename);

    receiver.subscribeTo<Image>(&logger);
    receiver.subscribeTo<motion_state_t>(&logger);

    std::cout << "Beginning to save images to " << basename << '\n';
    std::cout << "Press 'ENTER' to stop program\n";
    sleep(2);

    auto receiverFunc = [&receiver](bool killed) -> bool { receiver.processIncoming(); return !killed; };
    
    utils::RepeatedTask receiverTask(receiverFunc);

    std::cin.get();

    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool helpNeeded = commandLine.argumentExists(HELP_SHORT) ||
                      commandLine.argumentExists(HELP_LONG)  ||
                      !commandLine.argumentExists(BASENAME);

    if(helpNeeded)
    {
        std::cout<<"The command line arguments are:\n"
                 <<'\n'
                 <<"    -h/--help       Display help message                      (optional)\n"
                 <<"    --basename      Base filename to use for the saved images (required)\n"
                 <<std::endl;
        exit(1);
    }
}
