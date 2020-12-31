/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "sensors/avt_camera.h"
#include "utils/timestamp.h"
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>   // for converting the strings to integer format
#include <cassert>
#include <cstring>
#include <iostream>


namespace vulcan
{
namespace sensors
{

// Commands to issue
const std::string ACQUISITION_MODE_CMD("AcquisitionMode");
const std::string CONTINUOUS_CAPTURE_CMD("Continuous");
const std::string START_ACQUISITION_CMD("AcquisitionStart");
const std::string STOP_ACQUISITION_CMD("AcquisitionStop");

// Attributes to access
const std::string CONFIG_INDEX_ATTR("ConfigFileIndex");
const std::string LOAD_CONFIG_ATTR("ConfigFileLoad");
const std::string PIXEL_FORMAT_ATTR("PixelFormat");
const std::string FRAME_SIZE_ATTR("TotalBytesPerFrame");
const std::string PACKET_SIZE_ATTR("PacketSize");
const std::string GAMMA_ATTR("Gamma");

// Enumeration strings for config index
const std::string CONFIG_FACTORY("Factory");
const std::string CONFIG_1("1");
const std::string CONFIG_2("2");
const std::string CONFIG_3("3");
const std::string CONFIG_4("4");

// Enumeration strings for pixel formats
const std::string FORMAT_MONO8("Mono8");
const std::string FORMAT_MONO16("Mono16");
const std::string FORMAT_BAYER8("Bayer8");
const std::string FORMAT_BAYER16("Bayer16");
const std::string FORMAT_RGB24("Rgb24");

// Dealing with the various types of pixel formats
void copy_frame_to_image(const tPvFrame& frame, utils::SensorTime& time, Image& image, bool subsample);
void copy_mono8_frame   (const tPvFrame& frame, Image& image, bool subsample);
void copy_bayer8_frame  (const tPvFrame& frame, Image& image, bool subsample);
void copy_bayer16_frame (const tPvFrame& frame, Image& image, bool subsample);
void copy_rgb24_frame   (const tPvFrame& frame, Image& image, bool subsample);

// Initialization information printers
void print_all_connected_cameras_information(void);
void print_camera_information(const tPvCameraInfoEx& camera);

// Helpers for converting to the PvAPI format
std::string convert_config_index_to_enum_string(avt_config_file_index_t index);
std::string convert_pixel_format_to_enum_string(avt_pixel_format_t      format);

// Error code printers
void print_camera_open_error   (tPvErr error, unsigned long uniqueId, std::string ipAddress);
void print_camera_capture_error(tPvErr error);
void print_attribute_error     (tPvErr error, const std::string& attribute);
void print_generic_error       (tPvErr error);  // many errors are shared between functions, so shared messages are displayed with this function


// Initialization
void initialize_pvapi_library(bool verbose)
{
    // Initialization requires calling PvInitialize()
    unsigned long majorVersion = 0;
    unsigned long minorVersion = 0;
    PvVersion(&majorVersion, &minorVersion);
    
    if(PvInitialize() != ePvErrSuccess)
    {
        std::cerr<<"ERROR! Failed to initialize PvAPI library"<<std::endl;
        assert(false);
    }
    
    std::cout<<"INFO: Initialized PvAPI library version:"<<majorVersion<<'.'<<minorVersion<<'\n';

    // Wait for a camera to appear
    while(!PvCameraCount())
    {
        sleep(1);
    }
    
    if(verbose)
    {
        print_all_connected_cameras_information();
    }
}


AVTCamera::AVTCamera(const avt_camera_parameters_t& params)
: uniqueId(params.uniqueId)
, ipAddress(params.ipAddress)
, params(params)
, queuedFrames(params.queueSize)
, currentFrameIndex(0)
{
    tPvErr error;

    if(params.ipAddress.length() > 0)
    {
        struct in_addr ipNumber;
        inet_pton(AF_INET, ipAddress.c_str(), &ipNumber);

        error = PvCameraOpenByAddr(ipNumber.s_addr, ePvAccessMaster, &cameraHandle);
    }
    else
    {
        error = PvCameraOpen(uniqueId, ePvAccessMaster, &cameraHandle);
    }
    
    if(error != ePvErrSuccess)
    {
        print_camera_open_error(error, uniqueId, ipAddress);
        assert(false);
    }

    // Set the buffer and size for all queued frames to be 0 as they don't have constructors to do it
    // automatically and the buffers will be undefined otherwise
    for(auto frameIt = queuedFrames.begin(), frameEnd = queuedFrames.end(); frameIt != frameEnd; ++frameIt)
    {
        frameIt->ImageBuffer     = 0;
        frameIt->ImageBufferSize = 0;
    }
    
    setCameraParameters();
    allocateImageFrames();
}


// Camera interface implementation
void AVTCamera::startCapture(void)
{
    PvAttrEnumSet(cameraHandle, ACQUISITION_MODE_CMD.c_str(), CONTINUOUS_CAPTURE_CMD.c_str());
    PvCommandRun(cameraHandle, START_ACQUISITION_CMD.c_str());

    // Some parameters will only have an effect after acquisition has started, like Gamma, so set those
    // specifically
    setPostAcquisitionStartParams();
}


void AVTCamera::stopCapture(void)
{
    PvCommandRun(cameraHandle, STOP_ACQUISITION_CMD.c_str());
}


bool AVTCamera::getNextImage(Image& image)
{
    // Wait until the next image in the queue is ready, then copy the pixels appropriately
    tPvErr error = PvCaptureWaitForFrameDone(cameraHandle, &queuedFrames[currentFrameIndex], PVINFINITE);
    
    if(error == ePvErrSuccess)
    {
        copy_frame_to_image(queuedFrames[currentFrameIndex], time_, image, params.subsample);
    }
    else
    {
        print_camera_capture_error(error);
    }

    // The frame has been processed, so add it to the back of the queue and increment the index at the front of the queue
    PvCaptureQueueFrame(cameraHandle, &queuedFrames[currentFrameIndex], 0);

    currentFrameIndex = (currentFrameIndex+1) % queuedFrames.size();

    return error == ePvErrSuccess;
}

// AVTCamera specific methods for controlling the capture parameters

void AVTCamera::loadConfiguration(avt_config_file_index_t config)
{
    // To load the a particular configuration, the file index has to be set and then
    // config load command has to be issued
    std::string configString = convert_config_index_to_enum_string(config);
    
    tPvErr error = PvAttrEnumSet(cameraHandle, CONFIG_INDEX_ATTR.c_str(), configString.c_str());
    
    if(error != ePvErrSuccess)
    {
        print_attribute_error(error, CONFIG_INDEX_ATTR);
    }
    
    PvCommandRun(cameraHandle, LOAD_CONFIG_ATTR.c_str());
}


void AVTCamera::setPixelFormat(avt_pixel_format_t format)
{
    std::string formatString = convert_pixel_format_to_enum_string(format);
    
    tPvErr error = PvAttrEnumSet(cameraHandle, PIXEL_FORMAT_ATTR.c_str(), formatString.c_str());
    
    if(error != ePvErrSuccess)
    {
        print_attribute_error(error, PIXEL_FORMAT_ATTR);
    }
}


// Internal methods for doing initialization
void AVTCamera::setCameraParameters(void)
{
    setPacketSize();  // packet size has to be set before the camera capture is started
    PvCaptureStart(cameraHandle);
    loadConfiguration(params.config);
    setPixelFormat(params.format);
}


void AVTCamera::setPacketSize(void)
{
    tPvErr error = PvAttrUint32Set(cameraHandle, PACKET_SIZE_ATTR.c_str(), params.maxPacketSize);
    
    if(error != ePvErrSuccess)
    {
        print_attribute_error(error, PACKET_SIZE_ATTR);
    }
}


void AVTCamera::allocateImageFrames(void)
{
    // After the parameters are loaded, the image frame size can be queried from the camera
    // and the frames can be allocated to the proper size
    tPvUint32 frameSize = 0;
    
    tPvErr error = PvAttrUint32Get(cameraHandle, FRAME_SIZE_ATTR.c_str(), &frameSize);
    
    if(error != ePvErrSuccess)
    {
        print_attribute_error(error, FRAME_SIZE_ATTR);
    }

    // Clear the queue before queuing the new chunk of frames
    PvCaptureQueueClear(cameraHandle);

    for(auto frameIt = queuedFrames.begin(), frameEnd = queuedFrames.end(); frameIt != frameEnd; ++frameIt)
    {
        if(frameIt->ImageBufferSize < frameSize)
        {
            // I wish the image buffer wasn't declared at void*
            // Need to cast pixels to value they were actually allocated as in order for delete to be defined
            unsigned char* pixels = static_cast<unsigned char*>(frameIt->ImageBuffer);
            delete [] pixels;
            frameIt->ImageBuffer     = new char[frameSize];
            frameIt->ImageBufferSize = frameSize;
        }

        PvCaptureQueueFrame(cameraHandle, &(*frameIt), 0);
    }

    currentFrameIndex = 0;
}


void AVTCamera::setPostAcquisitionStartParams(void)
{
    float gamma = 1.0f;
    PvAttrFloat32Get(cameraHandle, GAMMA_ATTR.c_str(), &gamma);
    PvAttrFloat32Set(cameraHandle, GAMMA_ATTR.c_str(), gamma);

    PvAttrEnumSet(cameraHandle, "ExposureMode", "Auto");
}


// Functions for copying the various types of images available
void copy_frame_to_image(const tPvFrame& frame, utils::SensorTime& time, Image& image, bool subsample)
{
    switch(frame.Format)
    {
    case ePvFmtMono8:
        copy_mono8_frame(frame, image, subsample);
        break;
        
    case ePvFmtBayer8:
        copy_bayer8_frame(frame, image, subsample);
        break;
        
    case ePvFmtBayer16:
        copy_bayer16_frame(frame, image, subsample);
        break;
        
    case ePvFmtRgb24:
        copy_rgb24_frame(frame, image, subsample);
        break;
        
    default:
        std::cerr<<"ERROR! Unable to copy frame. Unknown image format: "<<frame.Format<<'\n';
    }
    
    uint64_t cameraTime = frame.TimestampHi;
    cameraTime <<= 32;
    cameraTime |= frame.TimestampLo;
    image.setTimestamp(time.timestamp(cameraTime));
}


void copy_mono8_frame(const tPvFrame& frame, Image& image, bool subsample)
{
    // If the image is not the correct size, assign it an image with the correct size
    std::size_t outputWidth  = subsample ? frame.Width/2  : frame.Width;
    std::size_t outputHeight = subsample ? frame.Height/2 : frame.Height;
    
    if((image.getWidth() != outputWidth) || (image.getHeight() != outputHeight) || (image.getColorspace() != MONO))
    {
        image = Image(outputWidth, outputHeight, MONO);
    }
    
    unsigned char* sourceBuffer = static_cast<unsigned char*>(frame.ImageBuffer);
    unsigned char* outputBuffer = image.getPixelBuffer();
    
    // Only want every other pixel for subsampling, so simple memcpy doesn't work
    if(subsample)
    {
        int rowStride    = outputWidth;
        int columnStride = 1;
        
        int sourceIndex = 0;
        int outputIndex = 0;
        
        for(std::size_t y = 0; y < outputHeight; ++y)
        {
            for(std::size_t x = 0; x < outputWidth; ++x)
            {
                outputBuffer[outputIndex++] = sourceBuffer[sourceIndex++];
                
                sourceIndex += columnStride;
            }
            
            sourceIndex += rowStride;
        }
    }
    else // if not subsampling, outputBuffer and sourceBuffer need to have the same contents
    {
        memcpy(outputBuffer, sourceBuffer, outputWidth*outputHeight);
    }
}


void copy_bayer8_frame(const tPvFrame& frame, Image& image, bool subsample)
{
    std::cerr<<"WARNING: Bayer8 pixel format not currently supported."<<std::endl;
}


void copy_bayer16_frame(const tPvFrame& frame, Image& image, bool subsample)
{
    std::cerr<<"WARNING: Bayer16 pixel format not currently supported."<<std::endl;
}


void copy_rgb24_frame(const tPvFrame& frame, Image& image, bool subsample)
{
    // If the image is not the correct size, assign it an image with the correct size
    std::size_t outputWidth  = subsample ? frame.Width/2  : frame.Width;
    std::size_t outputHeight = subsample ? frame.Height/2 : frame.Height;
    
    if((image.getWidth() != outputWidth) || (image.getHeight() != outputHeight) || (image.getColorspace() != RGB))
    {
        image = Image(outputWidth, outputHeight, RGB);
    }
    
    unsigned char* sourceBuffer = static_cast<unsigned char*>(frame.ImageBuffer);
    unsigned char* outputBuffer = image.getPixelBuffer();
    
    // Only want every other pixel for subsampling, so simple memcpy doesn't work
    // 3 bytes per pixel with RGB24
    if(subsample)
    {
        int rowStride    = frame.Width*3;
        int columnStride = 3;
        
        int sourceIndex = 0;
        int outputIndex = 0;
        
        for(std::size_t y = 0; y < outputHeight; ++y)
        {
            for(std::size_t x = 0; x < outputWidth; ++x)
            {
                outputBuffer[outputIndex++] = sourceBuffer[sourceIndex++];
                outputBuffer[outputIndex++] = sourceBuffer[sourceIndex++];
                outputBuffer[outputIndex++] = sourceBuffer[sourceIndex++];
                
                sourceIndex += columnStride;
            }
            
            sourceIndex += rowStride;
        }
    }
    else // if not subsampling, outputBuffer and sourceBuffer need to have the same contents
    {
        memcpy(outputBuffer, sourceBuffer, outputWidth*outputHeight*3);
    }
}


// Initialization code
void print_all_connected_cameras_information(void)
{
    std::cerr<<"WARNING: Printing connected cameras information not currently implemented\n";
}


void print_camera_information(const tPvCameraInfoEx& camera)
{
    std::cerr<<"WARNING: Printing camera information not currently implemented\n";
}

// Helpers for setting the parameters
std::string convert_config_index_to_enum_string(avt_config_file_index_t index)
{
    switch(index)
    {
    case AVT_CONFIG_1:
        return CONFIG_1;
        
    case AVT_CONFIG_2:
        return CONFIG_2;
        
    case AVT_CONFIG_3:
        return CONFIG_3;
        
    case AVT_CONFIG_4:
        return CONFIG_4;
        
    case AVT_CONFIG_FACTORY:
    default:
        return CONFIG_FACTORY;
    }
}


std::string convert_pixel_format_to_enum_string(avt_pixel_format_t format)
{
    switch(format)
    {
    case AVT_PIXEL_MONO_8:
        return FORMAT_MONO8;
        
    case AVT_PIXEL_BAYER_8:
        return FORMAT_BAYER8;
        
    case AVT_PIXEL_BAYER_16:
        return FORMAT_BAYER16;
        
    case AVT_PIXEL_RGB_24:
    case AVT_PIXEL_DEFAULT:
    default:
        return FORMAT_RGB24;
    }
}

// Error printing
void print_camera_open_error(tPvErr error, long unsigned int uniqueId, std::string ipAddress)
{
    // The text describing the error messages is taken from PvApi.h
    
    std::cerr<<"ERROR! Unable to open camera: id:"<<uniqueId<<" ip:"<<ipAddress<<" : ";
    
    switch(error)
    {
    case ePvErrAccessDenied:
        std::cerr<<"ePvErrAccessDenied -- the camera couldn't be open in the requested mode";
        break;
        
    case ePvErrUnplugged:
        std::cerr<<"ePvErrUnplugged -- the camera was found but unplugged during the function call";
        break;
        
    case ePvErrNotFound:
        std::cerr<<"ePvErrNotFound -- the camera was not found (unplugged)";
        break;
        
    case ePvErrBadParameter:
        std::cerr<<"ePvErrBadParameter -- a valid pointer for pCamera was not supplied";
        break;
        
    case ePvErrResources:
        std::cerr<<"ePvErrResources -- resources requested from the OS were not available";
        break;
        
    case ePvErrBadSequence:
        std::cerr<<"ePvErrBadSequence -- API isn't initialized or camera is already open";
        break;
        
    default:
        print_generic_error(error);
    }
    
    std::cout<<std::endl;  // force the text to print because the program might be exiting after this error
}


void print_camera_capture_error(tPvErr error)
{
    // Error descriptions takens from PvApi.h
    
    std::cerr<<"ERROR! Failed to capture image: ";
    
    switch(error)
    {
    case ePvErrTimeout:
        std::cerr<<"ePvErrTimeout -- timeout while waiting for the frame";
        break;
        
    default:
        print_generic_error(error);
    }
    
    std::cout<<std::endl;
}


void print_attribute_error(tPvErr error, const std::string& attribute)
{
    std::cerr<<"ERROR! Unable to access attribute "<<attribute<<": ";
    
    switch(error)
    {
    case ePvErrNotFound:
        std::cerr<<"ePvErrNotFound -- the requested attribute doesn't exist";
        break;
        
    case ePvErrWrongType:
        std::cerr<<"ePvErrWrongType -- the requested attribute is not of the correct type";
        break;
        
    case ePvErrBadParameter:
        std::cerr<<"ePvBadParameter -- a valid pointer to hold the attribute value was not supplied";
        break;
        
    case ePvErrForbidden:
        std::cerr<<"ePvErrForbidden -- the requested attribute forbids this operation";
        break;
        
    case ePvErrOutOfRange:
        std::cerr<<"ePvErrOutOfRange -- the supplied value is out of range";
        break;
        
    default:
        print_generic_error(error);
    }
    
    std::cout<<std::endl;
}


void print_generic_error(tPvErr error)
{
    switch(error)
    {
    case ePvErrBadHandle:
        std::cerr<<"ePvErrBadHandle -- the handle of the camera is invalid";
        break;
        
    case ePvErrUnplugged:
        std::cerr<<"ePvErrUnplugged -- the camera has been unplugged";
        break;
        
    case ePvErrInternalFault:
        std::cerr<<"ePvErrInternalFault -- an internal fault occurred";
        break;
        
    case ePvErrBadSequence:
        std::cerr<<"ePvErrBadSequence -- API isn't initialized";
        break;
        
    default:
        std::cerr<<"Unknown error type: "<<error;
    }
};

} // namespace sensors
} // namespace vulcan
