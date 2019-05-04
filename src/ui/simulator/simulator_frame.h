/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_frame.h
* \author   Collin Johnson and Zongtai Luo
*
* Declaration of SimulatorFrame.
*/

#ifndef UI_SIMULATOR_SIMULATOR_FRAME_H
#define UI_SIMULATOR_SIMULATOR_FRAME_H

#include <ui/simulator/simulator_ui.h>

namespace vulcan
{
namespace ui
{

// class SimulatorControl;
class SimulatorRobotControl;

class SimulatorFrame : public SimulatorUI
{
public:

    SimulatorFrame(void);
    virtual ~SimulatorFrame(void);

private:

    SimulatorRobotControl* robot_control_;

    void setupSimulatorUI(void);
};

}
}

#endif // UI_SIMULATOR_SIMULATOR_FRAME_H
