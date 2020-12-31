/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_classifier_test_dialog.cpp
 * \author   Collin Johnson
 *
 * Definition of GatewayClassifierTestResultsDialog.
 */

#include "ui/mapeditor/gateway_classifier_test_dialog.h"
#include "hssh/local_topological/area_detection/gateways/gateway_classifier.h"
#include "ui/common/file_dialog_settings.h"
#include <boost/range/iterator_range.hpp>
#include <wx/variant.h>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(GatewayClassifierTestResultsDialog, ClassificationTestResultsDialogBase)
EVT_BUTTON(ID_SAVE_CLASSIFIER_BUTTON, GatewayClassifierTestResultsDialog::saveClassifierPressed)
EVT_BUTTON(ID_CLOSE_CLASSIFIER_BUTTON, GatewayClassifierTestResultsDialog::closePressed)
END_EVENT_TABLE()


std::pair<double, double> precision_and_recall(const hssh::GatewayMapResults& results);


GatewayClassifierTestResultsDialog::GatewayClassifierTestResultsDialog(const hssh::GeneratorResults& results,
                                                                       wxWindow* parent)
: ClassificationTestResultsDialogBase(parent)
, classifier_(results.classifier)
, haveSavedClassifier_(false)
{
    classificationResultsSummaryLabel->SetLabel(wxString::Format("Summary: %s", results.generatorName));

    // Change the text associated with the labels for the data to be specific to the desired format for gateways
    trainingResultsAccuracyLabel->SetLabel(wxString::Format("Precision"));
    trainingResultsCorrectLabel->SetLabel(wxString::Format("Recall"));
    classificationAccuracyLabel->SetLabel(wxString::Format("Precision"));
    correctClassificationTestsLabel->SetLabel(wxString::Format("Recall"));

    setupDetailsGrid();
    for (auto& result : results.testResults) {
        addResultsToDetailsGrid(result);
    }
    addResultsToDetailsGrid(results.overallTest);
    createSummary(results.overallTest, results.overallTraining);

    // Turn off the save button if there's isn't anything to save
    if (!classifier_) {
        saveClassifierButton->Enable(false);
    }
}


void GatewayClassifierTestResultsDialog::setupDetailsGrid(void)
{
    classificationDetailsList->AppendTextColumn("Map Name");
    classificationDetailsList->AppendTextColumn("Total");
    classificationDetailsList->AppendTextColumn("TP");
    classificationDetailsList->AppendTextColumn("FP");
    classificationDetailsList->AppendTextColumn("FN");
    classificationDetailsList->AppendTextColumn("Precision");
    classificationDetailsList->AppendTextColumn("Recall");

    classificationDetailsList->Fit();
}


void GatewayClassifierTestResultsDialog::addResultsToDetailsGrid(const hssh::GatewayMapResults& results)
{
    double precision = 0.0;
    double recall = 0.0;
    std::tie(precision, recall) = precision_and_recall(results);

    wxVector<wxVariant> resultData;
    resultData.push_back(wxVariant(wxString(results.mapName)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numActualGateways)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numTruePositives)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numFalsePositives)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numFalseNegatives)));
    resultData.push_back(wxVariant(wxString::Format("%.2f", precision)));
    resultData.push_back(wxVariant(wxString::Format("%.2f", recall)));
    classificationDetailsList->AppendItem(resultData);
}


void GatewayClassifierTestResultsDialog::createSummary(const hssh::GatewayMapResults& testResults,
                                                       const hssh::GatewayMapResults& trainingResults)
{
    setResultsText(testResults,
                   totalClassificationTestsText,
                   correctClassificationTestsText,
                   classificationAccuracyText);
    setResultsText(trainingResults, trainingResultsTotalText, trainingResultsCorrectText, trainingResultsAccuracyText);
}


void GatewayClassifierTestResultsDialog::setResultsText(const hssh::GatewayMapResults& overall,
                                                        wxStaticText* numTestsText,
                                                        wxStaticText* numCorrectText,
                                                        wxStaticText* accuracyText)
{
    double precision = 0.0;
    double recall = 0.0;
    std::tie(precision, recall) = precision_and_recall(overall);

    int totalTests = overall.numActualGateways;
    numTestsText->SetLabel(wxString::Format("%i", totalTests));
    numCorrectText->SetLabel(wxString::Format("%.2f", recall));
    accuracyText->SetLabel(wxString::Format("%.2f", precision));
}


void GatewayClassifierTestResultsDialog::saveClassifierPressed(wxCommandEvent& event)
{
    assert(classifier_);

    wxFileDialog saveDialog(this, wxT("Select output file..."), wxT(""), wxT(""), wxT(""), kFileSaveFlags);

    if (saveDialog.ShowModal() == wxID_OK) {
        wxString filename = saveDialog.GetPath();
        haveSavedClassifier_ = classifier_->save(filename.ToStdString());

        if (!haveSavedClassifier_) {
            std::cerr << "ERROR:GatewayClassifierTestResultsDialog: Failed to save classifier to " << filename << '\n';
        }
    }
}


void GatewayClassifierTestResultsDialog::closePressed(wxCommandEvent& event)
{
    if (IsModal()) {
        EndModal((!classifier_ || haveSavedClassifier_ ? wxID_OK : wxID_CANCEL));
    } else {
        Destroy();
    }
}


std::pair<double, double> precision_and_recall(const hssh::GatewayMapResults& results)
{
    double recall = 0.0;
    int totalExamples = results.numActualGateways;
    if (totalExamples > 0) {
        recall = results.numTruePositives / static_cast<double>(totalExamples);
    }

    double precision = 0.0;
    int totalPositives = results.numTruePositives + results.numFalsePositives;
    if (totalPositives > 0) {
        precision = results.numTruePositives / static_cast<double>(totalPositives);
    }

    return std::make_pair(precision, recall);
}

}   // namespace ui
}   // namespace vulcan
