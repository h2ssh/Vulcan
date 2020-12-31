/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     classification_test_results_dialog.cpp
* \author   Collin Johnson
*
* Definition of ClassificationTestResultsDialog.
*/

#include "ui/mapeditor/classification_test_results_dialog.h"
#include "ui/common/file_dialog_settings.h"
#include <wx/variant.h>
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(ClassificationTestResultsDialog, ClassificationTestResultsDialogBase)
    EVT_BUTTON(ID_SAVE_CLASSIFIER_BUTTON,   ClassificationTestResultsDialog::saveClassifierPressed)
    EVT_BUTTON(ID_CLOSE_CLASSIFIER_BUTTON, ClassificationTestResultsDialog::closePressed)
END_EVENT_TABLE()


ClassificationTestResultsDialog::ClassificationTestResultsDialog(const std::string& classifierType,
                                                                 const std::string& dataType,
                                                                 const hssh::LabeledAreaData& trainingSet,
                                                                 const hssh::LabeledAreaData& testSet,
                                                                 wxWindow* parent)
: ClassificationTestResultsDialogBase(parent)
, test_(classifierType, trainingSet, testSet)
, classifierType_(classifierType)
, dataType_(dataType)
, haveSavedClassifier_(false)
{
    test_.run();
    
    setupDetailsGrid();
    for(auto& result : boost::make_iterator_range(test_.beginTest(), test_.endTest()))
    {
        addResultsToDetailsGrid(result);
    }
    addTotalsToDetailsGrid();
    createSummary();
}


void ClassificationTestResultsDialog::setupDetailsGrid(void)
{
    /*
    * The data view for the details is a grid with 9 columns:
    *
    *   name num-areas correct-label dec->dest dec->path dest->path dest->dec path->dest path->dec
    */

    classificationDetailsList->AppendTextColumn("Map Name");
    classificationDetailsList->AppendTextColumn("Total Areas");
    classificationDetailsList->AppendTextColumn("Correct");
    classificationDetailsList->AppendTextColumn("Dec->Dest");
    classificationDetailsList->AppendTextColumn("Dec->Path");
    classificationDetailsList->AppendTextColumn("Dest->Path");
    classificationDetailsList->AppendTextColumn("Dest->Dec");
    classificationDetailsList->AppendTextColumn("Path->Dest");
    classificationDetailsList->AppendTextColumn("Path->Dec");
}


void ClassificationTestResultsDialog::addResultsToDetailsGrid(const hssh::MapTestResults& results)
{
    wxVector<wxVariant> resultData;
    resultData.push_back(wxVariant(wxString(results.mapName)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numTests)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numCorrectTests)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numDecisionAsDest)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numDecisionAsPath)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numDestAsPath)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numDestAsDecision)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numPathAsDest)));
    resultData.push_back(wxVariant(wxString::Format("%i", results.numPathAsDecision)));
    classificationDetailsList->AppendItem(resultData);
}


void ClassificationTestResultsDialog::addTotalsToDetailsGrid(void)
{
    auto overall = test_.overallTestResults();
    
    wxVector<wxVariant> totalData;
    totalData.push_back(wxVariant(wxString("Total")));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numTests)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numCorrectTests)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numDecisionAsDest)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numDecisionAsPath)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numDestAsPath)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numDestAsDecision)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numPathAsDest)));
    totalData.push_back(wxVariant(wxString::Format("%i", overall.numPathAsDecision)));
    classificationDetailsList->AppendItem(totalData);
}


void ClassificationTestResultsDialog::createSummary(void)
{
    classificationResultsSummaryLabel->SetLabel(wxString::Format("Summary: Data: %s Classifier: %s", dataType_, classifierType_));
    
    setResultsText(test_.overallTestResults(), 
                   totalClassificationTestsText, 
                   correctClassificationTestsText,
                   classificationAccuracyText);
    setResultsText(test_.overallTrainingResults(),
                   trainingResultsTotalText,
                   trainingResultsCorrectText,
                   trainingResultsAccuracyText);
}


void ClassificationTestResultsDialog::setResultsText(const hssh::MapTestResults& overall, 
                                                     wxStaticText* numTestsText,
                                                     wxStaticText* numCorrectText,
                                                     wxStaticText* accuracyText)
{
    numTestsText->SetLabel(wxString::Format("%i", overall.numTests));
    numCorrectText->SetLabel(wxString::Format("%i", overall.numCorrectTests));

    double accuracy = 0.0;

    if(overall.numTests > 0)
    {
        accuracy = (100.0 * overall.numCorrectTests) / overall.numTests;
    }

    accuracyText->SetLabel(wxString::Format("%.2f", accuracy));
}


void ClassificationTestResultsDialog::saveClassifierPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(this, wxT("Select output file..."), wxT(""), wxT(""), wxT(""), kFileSaveFlags);

    if(saveDialog.ShowModal() == wxID_OK)
    {
        wxString filename = saveDialog.GetPath();
        haveSavedClassifier_ = test_.saveClassifier(filename.ToStdString());

        if(!haveSavedClassifier_)
        {
            std::cerr << "ERROR:ClassificationTestResultsDialog: Failed to save classifier to " << filename << '\n';
        }
    }
}


void ClassificationTestResultsDialog::closePressed(wxCommandEvent& event)
{
    if(IsModal())
    {
        EndModal((haveSavedClassifier_ ? wxID_OK : wxID_CANCEL));
    }
    else
    {
        Destroy();
    }
}

} // namespace ui
} // namespace vulcan
