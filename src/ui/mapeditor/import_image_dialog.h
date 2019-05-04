/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     import_image_dialog.h
* \author   Collin Johnson
*
* Definition of ImportImageDialog.
*/

#ifndef UI_MAPEDITOR_IMPORT_IMAGE_DIALOG_H
#define UI_MAPEDITOR_IMPORT_IMAGE_DIALOG_H

#include <ui/mapeditor/map_editor.h>
#include <cassert>

namespace vulcan
{
namespace hssh { struct image_import_properties_t; }
namespace hssh { class LocalPerceptualMap; }
namespace ui
{

/**
* ImportImageDialog is a simple dialog for importing an LPM from a PGM file. The dialog requires the following
* from the user:
*
*   - The image filename
*   - The scale
*   - The threshold for free cells
*   - The threshold for occupied cells
*
* If the LPM is created successfully, then the LPM can be accessed via the getImportedLPM() method. The dialog will
* validate all inputs. The following are necessary:
*
*   - scale > 0
*   - occupied threshold < free threshold
*   - 0 <= occupied threshold < 255
*   - 0 <  free threshold <= 255
*/
class ImportImageDialog : public ImportImageDialogBase
{
public:

    /**
    * Constructor for ImportImageDialog.
    *
    * \param    parent          Parent window for the dialog
    */
    ImportImageDialog(wxWindow* parent);

    /**
    * getImportedLPM retrieves the LPM that was imported. This method is only valid to be called if the
    * return from ShowModal is wxID_OK.
    */
    hssh::LocalPerceptualMap getImportedLPM(void) const;

private:

    bool        haveValidParameters;
    std::string fullImagePath;

    bool validateUserInput(void);
    hssh::image_import_properties_t createImportPropertiesFromInput(void) const;

    // Event handlers
    void selectImagePressed(wxCommandEvent& event);
    void importPressed     (wxCommandEvent& event);
    void cancelledDialog   (wxCloseEvent&   event);

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_MAPEDITOR_IMPORT_IMAGE_DIALOG_H
