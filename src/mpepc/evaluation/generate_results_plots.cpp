/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     generate_results_plots.cpp
* \author   Collin Johnson
*
* generate_results_plots is a utility program to take advantage of gnuplot-iostream to create plots of a variety of
* data sources. The plots that can be generated will be both displayed on the screen and output to images that can then
* be directly included in a paper.
* 
* The general use of the program is to pass a file that describes the plots to create. You run the program and pass the
* log file in on the command line. A bunch of plots will pop up on the screen and files will be saved to the specified
* directory.
* 
* If no plot file is provided, one of each type of plot will be generated for the entire log file.
* 
* The file format that describes the plots has the following format:
* 
*   'start' 'end' 'plotA' 'plotB' . . .
*
* Each line defines the start and end time of the plot duration in seconds elapsed since the start of the log file. To
* use the whole log file, specify 0 as the start and -1 as the end. Following the start and end time are zero or 
* more identifiers for the types of plots to produce. The names of the plots and a description of
* the data are described below:
* 
*   comfort : The comfort plot shows total jerk squared integrated over time for forward and lateral motion.
*   accel : The accel plot shows the raw forward and lateral accelerations.
*   safety : The safety plots shows linear and angular velocity and distance to nearest static and dynamic obstacle.
*   stability : The stability plots shows the total distance between the estimated state of the robot for consecutive
*       optimal trajectory estimates.
*   collision : The collision plot is a scatter plot showing (1-p_c(0)) vs |v| + |omega|
*   timing : The timing plot shows the computation time for each update using the mpepc_timing_info_t message.
*/

#include <mpepc/evaluation/metrics.h>
#include <mpepc/evaluation/mpepc_log.h>
#include <utils/plot2d.h>
#include <boost/optional.hpp>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

using namespace vulcan;


const std::string kComfortPlot("comfort");
const std::string kAccelPlot("accel");
const std::string kSafetyPlot("safety");
const std::string kStabilityPlot("stability");
const std::string kCollisionPlot("collision");
const std::string kTimingPlot("timing");


enum class PlotType
{
    comfort,
    accel,
    safety,
    stability,
    collision,
    timing,
    unknown,
};


struct DurationInfo
{
    int64_t startUs;        ///< Relative start time of the duration
    int64_t endUs;          ///< Relative end time of the duration
    
    std::vector<PlotType> plots;    ///< Plots to produce for this duration
};

std::ostream& operator<<(std::ostream& out, const DurationInfo& duration);
std::ostream& operator<<(std::ostream& out, PlotType type);


std::vector<DurationInfo> parse_duration_file(const std::string& filename);
boost::optional<DurationInfo> parse_duration(const std::string& durationStr);
PlotType plot_type_from_string(const std::string& type);
std::vector<PlotType> all_plots(void);
void generate_plots_for_duration(const DurationInfo& duration, mpepc::MPEPCLog& log);


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr << "ERROR: generate_results_plots expected command-line takes 'log_file' 'duration_file' as "
            << "the expected arguments.\n";
        return -1;
    }
    else if(argc > 3) // more arguments than expected -- just have this as a warning
    {
        std::cerr << "WARNING: More command line arguments than expected! Ignoring: ";
        for(int n = 3; n < argc; ++n)
        {
            std::cerr << ' ' << argv[n];
        }
        std::cerr << '\n';
    }
    
    mpepc::MPEPCLog log(argv[1]);
    std::vector<DurationInfo> durations;
    
    if(argc == 2)
    {
        std::cout << "INFO: Generating all plots for entire duration of log " << argv[1] << 'n';
        
        DurationInfo fullLog;
        fullLog.startUs = 0;
        fullLog.endUs = log.durationUs();
        fullLog.plots = all_plots();
        durations.push_back(fullLog);
    }
    else if(argc > 2)
    {
        std::cout << "INFO: Loading duration information from " << argv[2] << '\n';
        durations = parse_duration_file(argv[2]);
    }
    
    for(auto& d : durations)
    {
        std::cout << "Generating plots for " << d << '\n';
        generate_plots_for_duration(d, log);
    }
    
    std::cin.get();
    
    return 0;
}


std::vector<DurationInfo> parse_duration_file(const std::string& filename)
{
    std::vector<DurationInfo> durations;
    
    std::ifstream in(filename);
    
    if(!in.is_open())
    {
        std::cerr << "ERROR: Failed to read duration file: " << filename << '\n';
        return durations;
    }
    
    for(std::string line; std::getline(in, line);)
    {
        auto nextDuration = parse_duration(line);
        if(nextDuration)
        {
            durations.push_back(*nextDuration);
        }
    }
    
    // Sort durations in order of increasing start time to ensure they can all be displayed via loadTimeRange
    std::sort(durations.begin(), durations.end(), [](auto& lhs, auto& rhs) {
        return lhs.startUs < rhs.startUs;
    });
    
    return durations;
}


boost::optional<DurationInfo> parse_duration(const std::string& durationStr)
{
    if(durationStr.empty())
    {
        return boost::none;
    }
    
    std::istringstream in(durationStr);
    
    DurationInfo duration;
    in >> duration.startUs >> duration.endUs;
    
    duration.startUs *= 1000000;
    duration.endUs *= 1000000;
    
    std::string typeStr;
    while(in >> typeStr)
    {
        auto type = plot_type_from_string(typeStr);
        if(type != PlotType::unknown)
        {
            duration.plots.push_back(type);
        }
    }
    
    // If no plots were provided, then use all plots
    if(duration.plots.empty())
    {
        duration.plots = all_plots();
    }
    
    // If there is a valid duration, then 
    if((duration.startUs < duration.endUs) || (duration.endUs == -1))
    {
        return duration;
    }
    else 
    {
        return boost::none;
    }
}


PlotType plot_type_from_string(const std::string& type)
{
    if(type == kComfortPlot)
    {
        return PlotType::comfort;
    }
    else if(type == kAccelPlot)
    {
        return PlotType::accel;
    }
    else if(type == kSafetyPlot)
    {
        return PlotType::safety;
    }
    else if(type == kStabilityPlot)
    {
        return PlotType::stability;
    }
    else if(type == kCollisionPlot)
    {
        return PlotType::collision;
    }
    else if(type == kTimingPlot)
    {
        return PlotType::timing;
    }
    
    // Else an unknown plot type
    std::cerr << "ERROR: Unknown plot type: " << type << '\n';
    return PlotType::unknown;
}


std::vector<PlotType> all_plots(void)
{
    return {
        PlotType::comfort,
        PlotType::accel,
        PlotType::safety,
        PlotType::stability,
        PlotType::collision,
        PlotType::timing
    };
}


void generate_plots_for_duration(const DurationInfo& duration, mpepc::MPEPCLog& log)
{
    log.loadTimeRange(duration.startUs, duration.endUs);
    
    for(auto& plot : duration.plots)
    {
        switch(plot)
        {
        case PlotType::comfort:
            mpepc::plot_comfort(log.beginImu(), log.endImu());
            break;
        case PlotType::accel:
            mpepc::plot_acceleration(log.beginImu(), log.endImu());
            break;
        case PlotType::safety:
            mpepc::plot_safety(log.beginMotionState(), log.endMotionState(), log.beginMPEPCInfo(), log.endMPEPCInfo());
            break;
        case PlotType::stability:
            mpepc::plot_stability();
            break;
        case PlotType::collision:
            mpepc::plot_collision();
            break;
        case PlotType::timing:
            mpepc::plot_timing();
            break;
        case PlotType::unknown:
            break;
        }
    }
}


std::ostream& operator<<(std::ostream& out, const DurationInfo& duration)
{
    out << "Duration: " << duration.startUs << " to " << duration.endUs << " Plots: ";
    std::copy(duration.plots.begin(), duration.plots.end(), std::ostream_iterator<PlotType>(out, " "));
    return out;
}


std::ostream& operator<<(std::ostream& out, PlotType type)
{
    switch(type)
    {
    case PlotType::comfort:
        out << kComfortPlot;
        break;
    case PlotType::accel:
        out << kAccelPlot;
        break;
    case PlotType::safety:
        out << kSafetyPlot;
        break;
    case PlotType::stability:
        out << kStabilityPlot;
        break;
    case PlotType::collision:
        out << kCollisionPlot;
        break;
    case PlotType::timing:
        out << kTimingPlot;
        break;
    case PlotType::unknown:
        out << "unknown";
        break;
    }
    
    return out;
}
