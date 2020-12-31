/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     classification_test_results_dialog.h
 * \author   Collin Johnson
 *
 * Declaration of ClassificationTestResultsDialog.
 */

#ifndef UI_MAPEDITOR_CLASSIFICATION_TEST_RESULTS_DIALOG_H
#define UI_MAPEDITOR_CLASSIFICATION_TEST_RESULTS_DIALOG_H

#include "hssh/local_topological/training/hypothesis_classifier_test.h"
#include "ui/mapeditor/map_editor.h"

namespace vulcan
{
namespace ui
{

/**
 * ClassificationTestResultsDialog calculates and displays the results of a training/test cycle. The classifier learned
 * for the results can be saved to a file for later use if the results are good enough via the Save Classifier button.
 * Otherwise, the results are thrown away after the dialog is closed.
 */
class ClassificationTestResultsDialog : public ClassificationTestResultsDialogBase
{
public:
    /**
     * Constructor for ClassificationTestResultsDialog.
     *
     * \param    classifierType      Type of classifier to create
     * \param    dataType            Type of data being used (initial or simplified)
     * \param    trainingSet         Training set of data
     * \param    testSet             Test set of data
     * \param    parent              Parent window for the dialog
     */
    ClassificationTestResultsDialog(const std::string& classifierType,
                                    const std::string& dataType,
                                    const hssh::LabeledAreaData& trainingSet,
                                    const hssh::LabeledAreaData& testSet,
                                    wxWindow* parent);

private:
    hssh::HypothesisClassifierTest test_;
    wxString classifierType_;
    wxString dataType_;
    bool haveSavedClassifier_;

    void setupDetailsGrid(void);
    void addResultsToDetailsGrid(const hssh::MapTestResults& results);
    void addTotalsToDetailsGrid(void);
    void createSummary(void);
    void setResultsText(const hssh::MapTestResults& overall,
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

#endif   // UI_MAPEDITOR_CLASSIFICATION_TEST_RESULTS_DIALOG_H
