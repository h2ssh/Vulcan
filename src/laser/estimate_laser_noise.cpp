/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "math/regression.h"
#include "math/statistics.h"
#include "sensors/sensor_log.h"
#include <boost/range/iterator_range.hpp>
#include <cassert>
#include <gnuplot-iostream.h>
#include <iostream>

using namespace vulcan;

using LaserIt = sensors::SensorLog::laser_iterator;

void estimate_sensor_noise(const std::string& laserName, LaserIt begin, LaserIt end);


int main(int argc, char** argv)
{
    sensors::SensorLog log(argv[1]);

    if (log.sizeFrontLaser() == 0) {
        return -1;
    }

    if (log.sizeBackLaser() == 0) {
        return -1;
    }

    estimate_sensor_noise("front", log.beginFrontLaser(), log.endFrontLaser());
    estimate_sensor_noise("back", log.beginBackLaser(), log.endBackLaser());

    return 0;
}


void estimate_sensor_noise(const std::string& laserName, LaserIt begin, LaserIt end)
{
    using ValueVec = std::vector<double>;
    std::vector<ValueVec> values(begin->ranges.size());

    for (auto& scan : boost::make_iterator_range(begin, end)) {
        assert(values.size() == scan.ranges.size());
        for (std::size_t n = 0; n < scan.ranges.size(); ++n) {
            if ((scan.ranges[n] > 0.0f) && (scan.ranges[n] < 40.0f)) {
                values[n].push_back(scan.ranges[n]);
            }
        }
    }

    std::vector<double> means;
    std::vector<double> variances;

    for (auto& v : values) {
        means.push_back(math::mean(v.begin(), v.end()));
        variances.push_back(std::sqrt(math::variance(v.begin(), v.end())));
    }

    std::vector<Point<float>> meanVsVar;
    for (std::size_t n = 0; n < means.size(); ++n) {
        // Toss out high std dev as they must have hit something dynamic
        //         if(variances[n] < 0.2)
        //         {
        meanVsVar.emplace_back(means[n], variances[n]);
        //         }
    }

    auto line = math::total_least_squares(meanVsVar.begin(), meanVsVar.end());
    std::cout << "Line fit:" << line << '\n';

    Gnuplot plot;
    plot << "plot '-' using 1:2 with points title 'Laser: " << laserName << "'\n";
    plot.send1d(boost::make_tuple(means, variances));
}
