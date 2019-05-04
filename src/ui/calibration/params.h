/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
*
* Declaration of the params for the Calibration UI.
*/

#ifndef UI_CALIBRATION_PARAMS_H
#define UI_CALIBRATION_PARAMS_H

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace ui
{

struct calibration_ui_params_t
{

};

/**
* load_calibration_ui_params loads the parameters for the Calibration UI from
* the provided ConfigFile.
*
* \param    config          ConfigFile with the parameters
* \return   Parameters pulled from the file.
*/
calibration_ui_params_t load_calibration_ui_params(const utils::ConfigFile& config);

}
}

#endif // UI_CALIBRATION_PARAMS_H
