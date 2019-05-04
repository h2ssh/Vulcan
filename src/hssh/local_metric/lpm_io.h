/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_io.h
* \author   Collin Johnson
*
* Declaration of functions for saving/loading LPMs.
*/

#ifndef HSSH_UTILS_LPM_IO_H
#define HSSH_UTILS_LPM_IO_H

#include <string>
#include <fstream>

namespace vulcan
{
namespace hssh
{

class LocalPerceptualMap;


const std::string kLPMExtension(".lpm");    // File extension to use for all saved LPMs


/**
* image_import_properties_t defines the properties needed for creating an LPM from a PGM image.
*/
struct image_import_properties_t
{
    double  scale;
    uint8_t freeThreshold;
    uint8_t occupiedThreshold;
};

/**
* export_lpm_to_image writes the LPM cost to a PPM. The metatdata is stored at the beginning of the image.
* The cost is stored to the green channel to allow editing in other channels.
*
* \param        filename    Name of the image
* \param        lpm         LPM to save
* \return   True if the image was saved successfully.
*/
bool export_lpm_to_image(const std::string& filename, const LocalPerceptualMap& lpm);

/**
* import_lpm_labels_from_image reads evaluation labels from an image originally 
* created with export_lpm_to_image and hand labelled by coloring. The labels are
* stored in the typeGrid of the LPM
*
* \param        filename    Name of the image
* \param        lpm         LPM to load labels to
* \return   True if the labels were loaded successfully.
*/
bool import_lpm_labels_from_image(const std::string& filename, LocalPerceptualMap& lpm);

/**
* import_lpm_from_image imports an LPM from a PGM image. The properties of the new LPM are provided
* in the properties parameter.
*
* \param        filename    Name of the image
* \param        properties  Properties to use for converting the image into an LPM
* \param[out]   lpm         LPM that was loaded
* \return   True if the LPM was created successfully.
*/
bool import_lpm_from_image(const std::string& filename, const image_import_properties_t& properties, LocalPerceptualMap& lpm);

/**
* save_lpm_1_0 saves an LPM to disk using the version 1.0 format.
*
* \param    lpm         LPM to be saved
* \param    filename    Name of the file in which to save the LPM
*/
void save_lpm_1_0(const LocalPerceptualMap& lpm, const std::string& filename);

/**
* save_lpm_occupancy_text saves an LPM to disk in a text format where written value
* is either 0 or 1, based on whether a particular cell is occupied. The values are
* saved in row-major order.
*
* \param    lpm         LPM to be saved
* \param    stream      Stream in which to save the LPM
*/
void save_lpm_occupancy_text(const LocalPerceptualMap& lpm, std::ofstream& stream);

/**
* load_lpm_1_0 loads an LPM from disk using the version 1.0 format.
*
* \param    filename    Name of the file in which the LPM exists
* \param    lpm         Location in which to store the LPM data (output)
*/
void load_lpm_1_0(const std::string& filename, LocalPerceptualMap& lpm);

}
}

#endif // HSSH_UTILS_LPM_IO_H
