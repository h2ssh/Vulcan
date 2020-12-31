/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_classifier_test_dialog.h
 * \author   Collin Johnson
 *
 * Declaration of GatewayClassifierTestResultsDialog.
 */

#ifndef UI_MAPEDITOR_GATEWAY_CLASSIFIER_TEST_DIALOG_H
#define UI_MAPEDITOR_GATEWAY_CLASSIFIER_TEST_DIALOG_H

#include "hssh/local_topological/training/gateway_classifier_test.h"
#include "ui/mapeditor/map_editor.h"

namespace vulcan
{
namespace ui
{

/**
 * GatewayClassifierTestResultsDialog calculates and displays the results of a training/test cycle. The classifier
 * learned for the results can be saved to a file for later use if the results are good enough via the Save Classifier
 * button. Otherwise, the results are thrown away after the dialog is closed.
 */
class GatewayClassifierTestResultsDialog : public ClassificationTestResultsDialogBase
{
public:
    /**
     * Constructor for GatewayClassifierTestResultsDialog.
     *
     * \param    results     Results to display
     * \param    parent      Parent window for the dialog
     */
    GatewayClassifierTestResultsDialog(const hssh::GeneratorResults& results, wxWindow* parent);

private:
    std::shared_ptr<hssh::GatewayClassifier> classifier_;
    bool haveSavedClassifier_;

    void setupDetailsGrid(void);
    void addResultsToDetailsGrid(const hssh::GatewayMapResults& results);
    void addTotalsToDetailsGrid(const hssh::GatewayMapResults& overall);
    void createSummary(const hssh::GatewayMapResults& testResults, const hssh::GatewayMapResults& trainingResults);
    void setResultsText(const hssh::GatewayMapResults& overall,
                        wxStaticText* numTestsText,
                        wxStaticText* numCorrectText,
                        wxStaticText* accuracyText);

    // Event handlers
    void saveClassifierPressed(wxCommandEvent& event);
    void closePressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_MAPEDITOR_GATEWAY_CLASSIFIER_TEST_DIALOG_H
