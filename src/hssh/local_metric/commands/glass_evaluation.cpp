/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_evaluation.cpp
* \author   Collin Johnson
* 
* Definition of GlassEvaluationCommand.
*/

#include <hssh/local_metric/commands/glass_evaluation.h>
#include <hssh/metrical/mapping/mapper.h>
#include <utils/serialized_file_io.h>
#include <utils/stub.h>

namespace vulcan
{
namespace hssh
{
    
std::unique_ptr<GlassEvaluationCommand> GlassEvaluationCommand::CreateSaveScansMessage(std::string source, 
                                                                                       std::string filename)
{
    return std::unique_ptr<GlassEvaluationCommand>(new GlassEvaluationCommand(kSaveScans, filename, source));
}


std::unique_ptr<GlassEvaluationCommand> GlassEvaluationCommand::CreateSavePosesMessage(std::string source, 
                                                                                       std::string filename)
{
    return std::unique_ptr<GlassEvaluationCommand>(new GlassEvaluationCommand(kSavePoses, filename, source));
}


std::unique_ptr<GlassEvaluationCommand> GlassEvaluationCommand::CreateSaveMapMessage(std::string source, 
                                                                                     std::string filename)
{
    return std::unique_ptr<GlassEvaluationCommand>(new GlassEvaluationCommand(kSaveMap, filename, source));
}


void GlassEvaluationCommand::issue(const metric_slam_data_t& data, 
                                   Localizer& localizer, 
                                   Mapper& mapper, 
                                   MetricRelocalizer& relocalizer) const
{
    switch(command_)
    {
    case kSaveMap:
        if(utils::save_serializable_to_file(filename_, mapper.getGlassMap()))
        {
            std::cout << "INFO: GlassEvaluationCommand: Saved glass map to " << filename_ << '\n';
        }
        else
        {
            std::cout << "ERROR: GlassEvaluationCommand: Failed to save glass map to " << filename_ << '\n';
        }
        break;

    case kSavePoses:
        PRINT_STUB("GlassEvaluationCommand::issue: Save Poses")
        break;

    case kSaveScans:
        PRINT_STUB("GlassEvaluationCommand::issue: Save Scans")
        break;
    }
}


void GlassEvaluationCommand::print(std::ostream& out) const
{
    out << "GlassEvaluationCommand from " << source();
}


GlassEvaluationCommand::GlassEvaluationCommand(Command command, std::string filename, std::string source)
: LocalMetricCommand(source)
, command_(command)
, filename_(filename)
{
}

}
}
