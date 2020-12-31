/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     file_dialog_settings.h
 * \author   Collin Johnson
 *
 * Definition of the default flags to be used when creating open or save file dialogs.
 */

#ifndef UI_COMMON_FILE_DIALOG_SETTINGS_H
#define UI_COMMON_FILE_DIALOG_SETTINGS_H

#include <wx/filedlg.h>

namespace vulcan
{
namespace ui
{

const int kFileOpenFlags = wxFD_OPEN | wxFD_CHANGE_DIR | wxFD_FILE_MUST_EXIST;
const int kFileSaveFlags = wxFD_SAVE | wxFD_CHANGE_DIR | wxFD_OVERWRITE_PROMPT;

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_FILE_DIALOG_SETTINGS_H
