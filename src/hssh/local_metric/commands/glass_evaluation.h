/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_evaluation.h
* \author   Collin Johnson and Paul Foster
* 
* Declaration of GlassEvaluationCommand.
*/

#ifndef HSSH_LOCAL_METRIC_COMMANDS_GLASS_EVALUATION_H
#define HSSH_LOCAL_METRIC_COMMANDS_GLASS_EVALUATION_H

#include <hssh/local_metric/command.h>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace hssh
{

/**
* GlassEvaluationCommand
*/
class GlassEvaluationCommand : public LocalMetricCommand
{
public:
    
    // Factories for creating the messages
    static std::unique_ptr<GlassEvaluationCommand> CreateSaveScansMessage(std::string source, std::string filename);
    static std::unique_ptr<GlassEvaluationCommand> CreateSavePosesMessage(std::string source, std::string filename);
    static std::unique_ptr<GlassEvaluationCommand> CreateSaveMapMessage(std::string source, std::string filename);
    
    // LocalMetricCommand interface
    void issue(const metric_slam_data_t& data, 
               Localizer& localizer, 
               Mapper& mapper, 
               MetricRelocalizer& relocalizer) const override;
    void print(std::ostream& out) const override;

private:
    
    enum Command
    {
        kSaveMap,
        kSavePoses,
        kSaveScans,
    };
    
    Command command_;
    std::string filename_;
    
    GlassEvaluationCommand(Command command, std::string filename, std::string source);
    
    // Serialization support
    GlassEvaluationCommand(void) { }

    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<LocalMetricCommand>(this),
            command_,
            filename_);
    }
};

}
}

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::GlassEvaluationCommand)

#endif // HSSH_LOCAL_METRIC_COMMANDS_GLASS_EVALUATION_H
