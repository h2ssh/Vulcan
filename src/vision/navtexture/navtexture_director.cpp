/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/navtexture/navtexture_director.h"
#include "core/image.h"
#include "core/pose.h"
#include "utils/auto_mutex.h"
#include "utils/timestamp.h"
#include "vision/image_utils.h"
#include "vision/pixel_histograms.h"
#include <sstream>


#define DEBUG_SEGMENTER
#define DEBUG_TIME


using namespace vulcan;
using namespace vulcan::vision;


NavTextureDirector::NavTextureDirector(const navtexture_params_t& params)
: objectIdentifier(params)
, params(params)
, haveImage(false)
, haveLaser(false)
, haveDynamic(false)
, dataTrigger(false)
{
}


NavTextureDirector::~NavTextureDirector(void)
{
}

// Handlers for the input consumer interface
void NavTextureDirector::handleData(const Image& image, const std::string& channel)
{
    dataLock.lock();

    std::ostringstream filename;
    filename << "eecs_hallway" << std::setw(4) << std::setfill('0') << imageNumber << ".png";
    ++imageNumber;
    vision::save_image_to_file(image, filename.str());

    if (currentImage.getWidth() != image.getWidth() || currentImage.getHeight() != image.getHeight() / 2) {
        currentImage = Image(image.getWidth(), image.getHeight() / 2, image.getColorspace());
    }

    unsigned char pixel[3];

    for (size_t y = image.getHeight() / 2; y < image.getHeight(); ++y) {
        for (size_t x = 0; x < image.getWidth(); ++x) {
            image.getPixel(x, y, pixel[0], pixel[1], pixel[2]);
            currentImage.setPixel(x, image.getHeight() - 1 - y, pixel[0], pixel[1], pixel[2]);
        }
    }

    haveImage = true;

    dataTrigger.setPredicate(haveLaser && haveImage && haveDynamic);
    dataLock.unlock();
    dataTrigger.broadcast();
}


void NavTextureDirector::handleData(const polar_laser_scan_t& scan, const std::string& channel)
{
    dataLock.lock();

    currentScan = scan;

    haveLaser = true;

    dataTrigger.setPredicate(haveLaser && haveImage && haveDynamic);
    dataLock.unlock();
    dataTrigger.broadcast();
}


void NavTextureDirector::handleData(const pose_t& pose, const std::string& channel)
{
}


void NavTextureDirector::handleData(const laser::dynamic_laser_points_t& dynamicPoints, const std::string& channel)
{
    dataLock.lock();

    this->dynamicPoints = dynamicPoints;

    haveDynamic = true;

    dataTrigger.setPredicate(haveLaser && haveImage && haveDynamic);
    dataLock.unlock();
    dataTrigger.broadcast();
}


void NavTextureDirector::handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel)
{
}


// Director interface implementation
void NavTextureDirector::waitForData(void)
{
    dataTrigger.wait();
}


void NavTextureDirector::processAvailableData(void)
{
    imageObjects.clear();

    dataLock.lock();

    int64_t startTime = utils::system_time_us();

    objectIdentifier.identifyObjects(currentImage, currentScan, dynamicPoints, imageObjects);

#ifdef DEBUG_SEGMENTER
    std::cout << "INFO: NavTextureDirector:Found " << objectIdentifier.getImageSegments().size() << " segments in "
              << ((utils::system_time_us() - startTime) / 1000) << "ms\n";
#endif

    haveImage = false;
    haveLaser = false;
    haveDynamic = false;

    dataTrigger.setPredicate(false);   // used up the data
    dataLock.unlock();
}


void NavTextureDirector::transmitCalculatedOutput(void)
{
    for (auto consumerIt = consumers.begin(), endIt = consumers.end(); consumerIt != endIt; ++consumerIt) {
        (*consumerIt)->handleImageSegments(objectIdentifier.getImageSegments());
    }
}


bool NavTextureDirector::haveRequiredDataForCalculation(void)
{
    return haveImage;
    //     return haveImage && haveLaser && haveDynamic;
}
