/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <mpepc/evaluation/mpepc_log.h>
#include <boost/range/iterator_range.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <time.h>

using namespace vulcan;

static bool gUseDateTime = false;
static std::string gDateOpt("date");

void save_mpepc_debug_info(const mpepc::MPEPCLog& log, const std::string& filename);
void save_mpepc_planned_poses(const mpepc::MPEPCLog& log, const std::string& filename);
void save_motion_state(const mpepc::MPEPCLog& log, const std::string& filename);
void save_joystick(const mpepc::MPEPCLog& log, const std::string& filename);

void print_time(int64_t time, std::ostream& out);

int main(int argc, char** argv)
{
    const std::string kDataHeader("text_data_");
    const std::string kMPEPCName("mpepc_debug_info_");
    const std::string kMPEPCPlannedPosesName("mpepc_planned_poses_");
    const std::string kMotionStateName("motion_state_");
    const std::string kJoystickName("joystick_");
    
    if(argc < 2)
    {
        std::cout << "Expected input: mpepc_info_to_text 'log_filename'\n";
        return -1;
    }
    
    if((argc > 2) && (std::string(argv[2]) == gDateOpt))
    {
        gUseDateTime = true;
        
        std::cout << "Saving data using wall-clock dates in the format HHMMSS.SSS, where SS.SSS is two characters for seconds and three characters for milliseconds.\n";
    }

    std::string logFilename(argv[1]);

    mpepc::MPEPCLog log(logFilename);

    save_mpepc_debug_info(log, kDataHeader + kMPEPCName + logFilename);
    save_mpepc_planned_poses(log, kDataHeader + kMPEPCPlannedPosesName + logFilename);
    save_motion_state(log, kDataHeader + kMotionStateName + logFilename);
    save_joystick(log, kDataHeader + kJoystickName + logFilename);

    return 0;
}


void save_mpepc_debug_info(const mpepc::MPEPCLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for(auto& mpepcInfo : boost::make_iterator_range(log.beginMPEPCInfo(), log.endMPEPCInfo()))
    {
        if(mpepcInfo.iteration != 0)
        {
            print_time(mpepcInfo.updateStartTimeUs, out);
            out << mpepcInfo.clearanceToStaticObs << ' '
                << mpepcInfo.clearanceToDynObs << ' '
                << mpepcInfo.iteration << ' ';
            print_time(mpepcInfo.planReleaseTimeUs, out);
            out << mpepcInfo.plannedTrajectory.motionTarget.velocityGain;
            
    //         out << ' ' << mpepcInfo.haveGoalPose
    //             << ' ' << mpepcInfo.goalPose.x
    //             << ' ' << mpepcInfo.goalPose.y
    //             << ' ' << mpepcInfo.goalPose.theta;
            
            out << '\n';
        }
    }
    
    out.close();
}


void save_mpepc_planned_poses(const mpepc::MPEPCLog& log, const std::string& filename)
{
    std::ofstream out(filename);
    
    for(auto& mpepcInfo : boost::make_iterator_range(log.beginMPEPCInfo(), log.endMPEPCInfo()))
    {
        if(mpepcInfo.iteration != 0)
        {
            print_time(mpepcInfo.planReleaseTimeUs, out);
            for(auto& pose : mpepcInfo.plannedTrajectory.poses)
            {
                out << ' ' << pose.x
                    << ' ' << pose.y
                    << ' ' << pose.theta;
            }
            out << '\n';
        }
    }
    
    out.close();
}


void save_motion_state(const mpepc::MPEPCLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for(auto& state : boost::make_iterator_range(log.beginMotionState(), log.endMotionState()))
    {
        print_time(state.pose.timestamp, out);
        out << state.pose.x << ' '
            << state.pose.y << ' '
            << state.pose.theta << ' ';
            
        print_time(state.velocity.timestamp, out);
        out << state.velocity.timestamp << ' '
            << state.velocity.linear << ' '
            << state.velocity.angular << ' ';
        
        print_time(state.acceleration.timestamp, out);
        out << state.acceleration.timestamp << ' '
            << state.acceleration.linear << ' '
            << state.acceleration.angular << ' ';
        
        print_time(state.differentialWheels.rightWheel.timestamp, out);
        out << state.differentialWheels.rightWheel.timestamp << ' '
            << state.differentialWheels.rightWheel.speed << ' '
            << state.differentialWheels.rightWheel.motorAccel << ' ';
        
        print_time(state.differentialWheels.leftWheel.timestamp, out);
        out << state.differentialWheels.leftWheel.timestamp << ' '
            << state.differentialWheels.leftWheel.speed << ' '
            << state.differentialWheels.leftWheel.motorAccel << '\n';
    }
    
    out.close();
}


void save_joystick(const mpepc::MPEPCLog& log, const std::string& filename)
{
    std::ofstream out(filename);

    for(auto& joystick : boost::make_iterator_range(log.beginCommandedJoystick(), log.endCommandedJoystick()))
    {
        print_time(joystick.timestamp, out);
        out << joystick.forward << ' '
            << joystick.left << '\n';
    }
    
    out.close();
}

//  (Time column of data contains hour, minute and second. e.g.
// > 205201025 means 20(hour):52(minute):01.025(second))

void print_time(int64_t time, std::ostream& out)
{
    if(gUseDateTime)
    {
        time_t wallTime = time / 1000000; 
        auto date = gmtime(&wallTime);
        out << std::setfill('0') 
            << std::setw(2) << date->tm_hour 
            << std::setw(2) << date->tm_min
            << std::setw(2) << date->tm_sec 
            << '.'
            << std::setw(3) << ((time % 1000000) / 1000) << ' ';
    }
    else
    {
        out << time << ' ';
    }
}
