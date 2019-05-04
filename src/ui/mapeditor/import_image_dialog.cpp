/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     import_image_dialog.cpp
* \author   Collin Johnson
*
* Implementation of ImportImageDialog.
*/

#include <ui/mapeditor/import_image_dialog.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/lpm_io.h>
#include <wx/filedlg.h>

namespace vulcan
{
namespace ui
{
    
BEGIN_EVENT_TABLE(ImportImageDialog, ImportImageDialogBase)
    EVT_BUTTON(ID_SELECT_IMAGE_BUTTON, ImportImageDialog::selectImagePressed)
    EVT_BUTTON(ID_IMPORT_IMAGE_BUTTON, ImportImageDialog::importPressed)
    EVT_CLOSE(ImportImageDialog::cancelledDialog)
END_EVENT_TABLE()


ImportImageDialog::ImportImageDialog(wxWindow* parent)
: ImportImageDialogBase(parent)
, haveValidParameters(false)
{
    SetEscapeId(ID_CANCEL_IMPORT_BUTTON);
}


hssh::LocalPerceptualMap ImportImageDialog::getImportedLPM(void) const
{
    if(!haveValidParameters)
    {
        std::cerr<<"ERROR::ImportImage: Invalid parameters for importing. Cannot create LPM.\n";
        return hssh::LocalPerceptualMap();
    }
    
    hssh::image_import_properties_t properties = createImportPropertiesFromInput();
    
    hssh::LocalPerceptualMap lpm;
    if(!hssh::import_lpm_from_image(fullImagePath, properties, lpm))
    {
        std::cerr<<"ERROR::ImportImage: Failed to load LPM: "<<fullImagePath<<'\n';
    }
    
    return lpm;
}


bool ImportImageDialog::validateUserInput(void)
{
    double scale = 0.0;
    if(!cellScaleText->GetValue().ToDouble(&scale) || (scale <= 0.0))
    {
        std::cerr<<"ERROR::ImportImage: Invalid scale value:"<<scale<<". Scale must be greater than 0.0\n";
        return false;
    }
    
    long freeThreshold     = 0;
    long occupiedThreshold = 0;
    
    if(!freeThresholdText->GetValue().ToLong(&freeThreshold) || (freeThreshold <= 0) || (freeThreshold > 255))
    {
        std::cerr<<"ERROR::ImportImage: Invalid free threshold:"<<freeThreshold<<" Threshold must be in range (0, 255].\n";
        return false;
    }
    
    if(!occupiedThresholdText->GetValue().ToLong(&occupiedThreshold) || (occupiedThreshold < 0) || (occupiedThreshold >= 255))
    {
        std::cerr<<"ERROR::ImportImage: Invalid occupied threshold:"<<freeThreshold<<" Threshold must be in range [0, 255).\n";
        return false;
    }
    
    if(occupiedThreshold >= freeThreshold)
    {
        std::cerr<<"ERROR::ImportImage: Invalid thresholds. Occupied must be less than free. Free:"<<freeThreshold<<" Occupied:"<<occupiedThreshold<<'\n';
        return false;
    }
    
    return true;
}


hssh::image_import_properties_t ImportImageDialog::createImportPropertiesFromInput(void) const
{
    hssh::image_import_properties_t properties;
    long threshold = 0;
    
    cellScaleText->GetValue().ToDouble(&(properties.scale));
    freeThresholdText->GetValue().ToLong(&threshold);
    properties.freeThreshold = threshold;
    occupiedThresholdText->GetValue().ToLong(&threshold);
    properties.occupiedThreshold = threshold;
    
    return properties;
}


void ImportImageDialog::selectImagePressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(this, wxT("Select PGM file..."), wxT(""), wxT(""), wxT("*.pgm"), wxFD_OPEN);
    
    if(loadDialog.ShowModal() == wxID_OK)
    {
        imageFilenameText->SetValue(loadDialog.GetFilename());
        fullImagePath = std::string(loadDialog.GetPath().mb_str());
    }
}


void ImportImageDialog::importPressed(wxCommandEvent& event)
{
    haveValidParameters = validateUserInput();
    
    EndModal(haveValidParameters ? wxID_OK : wxID_CANCEL);
}


void ImportImageDialog::cancelledDialog(wxCloseEvent& event)
{
    haveValidParameters = false;
}

} // namespace ui
} // namespace vulcan
