/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_transition.h
* \author   Collin Johnson
* 
* Declaration of AreaTransitionEvent.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_TRANSITION_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_TRANSITION_H

#include <hssh/local_topological/event.h>
#include <hssh/local_topological/gateway.h>
#include <boost/optional.hpp>
#include <cereal/types/base_class.hpp>
#include <memory>

namespace vulcan
{
namespace hssh
{
    
class LocalArea;

/**
* AreaTransitionEvent is an event the occurs when the robot moves from one area into a new area.
* The event provides the exited area and the entered area. This event will currently fire only once the entered
* area has achieved full visibility.
* 
* The initial area in which the robot is located will not have an associated entry gateway, as that is unknown
* information.
* 
* For the AreaTransitionEvent, the area associated with the event is the area that is entered. Hence,
* getEnteredArea() == getArea().
*/
class AreaTransitionEvent : public LocalAreaEvent
{
public:
    
    /**
    * Default constructor for AreaTransitionEvent.
    */
    AreaTransitionEvent(void) = default;
    
    /**
    * Constructor for AreaTransitionEvent.
    * 
    * \param    timestamp       Time at which the event occurred
    * \param    pose            Pose where the transition occurred
    * \param    exited          Area that was exited
    * \param    entered         Area that was entered
    * \param    gateway         Gateway joining the two areas
    */
    AreaTransitionEvent(int64_t                    timestamp,
                        const LocalPose&           pose,
                        std::shared_ptr<LocalArea> exited, 
                        std::shared_ptr<LocalArea> entered, 
                        const Gateway&             gateway);
    
    /**
    * Constructor for AreaTransitionEvent.
    * 
    * Create the initial event for entering an area.
    * 
    * \param    timestamp       Time at which the event occurred
    * \param    pose            Pose where the transition occurred
    * \param    entered         Area that was entered
    */
    AreaTransitionEvent(int64_t                    timestamp,
                        const LocalPose&           pose,
                        std::shared_ptr<LocalArea> entered);
    
    virtual ~AreaTransitionEvent(void) { }
    
    /**
    * exitedId retrieves the id of the area that was exited.
    */
    Id exitedId(void) const;

    /**
    * enteredId retrieves the id of the area that was entered.
    */
    Id enteredId(void) const;

    /**
    * enteredArea retrieves the area that was entered by the robot.
    */
    std::shared_ptr<LocalArea> enteredArea(void) const { return entered_; }
    
    /**
    * exitedArea retrieves the area that was exited by the robot.
    *
    * For the initial area transition, i.e. the event caused by the robot waking up in some area, there is no exited
    * area. Otherwise, the exited area always exists.
    */
    std::shared_ptr<LocalArea> exitedArea(void) const { return exited_; }
    
    /**
    * transitionGateway retrieves the gateway through which the area transition occurred. If this is the initial area
    * visited, there will not be a transition gateway.
    */
    boost::optional<Gateway> transitionGateway(void) const { return gateway_; }
    
    // LocalAreaEvent interface
    IdIter beginAreaIds(void) const override { return areaIds_.begin(); }
    IdIter endAreaIds(void) const override { return areaIds_.end(); }
    std::string       description(void) const override;
    void              accept     (LocalAreaEventVisitor& visitor) const override;
    LocalAreaEventPtr clone      (void) const override;
    
private:

    std::vector<Id> areaIds_;
    std::shared_ptr<LocalArea> entered_;
    std::shared_ptr<LocalArea> exited_;
    boost::optional<Gateway> gateway_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        bool loading = !entered_ && !exited_;
        
        ar (cereal::base_class<LocalAreaEvent>(this));
        ar (areaIds_,
            entered_,
            exited_);
        
        if(!loading)
        {
            if(gateway_)
            {
                ar (true,
                    gateway_.get());
            }
            else 
            {
                Gateway g;
                ar (false,
                    g);
            }
        }
        else
        {
            bool haveGateway;
            Gateway g;
            ar (haveGateway,
                g);
            
            if(haveGateway)
            {
                gateway_ = g;
            }
        }
    }
    
//     template <class Archive>
//     void save(Archive& ar) const
//     {
//         ar (cereal::base_class<LocalAreaEvent>(this));
//         ar (entered_,
//             exited_);
//         
//         if(gateway_)
//         {
//             ar (true,
//                 gateway_.get());
//         }
//         else 
//         {
//             ar (false);
//         }
//     }
//     
//     template <class Archive>
//     void load(Archive& ar)
//     {
//         ar (cereal::base_class<LocalAreaEvent>(this));
//         ar (entered_,
//             exited_);
//         
//         bool haveGateway;
//         ar (haveGateway);
//         
//         if(haveGateway)
//         {
//             Gateway g;
//             ar (g);
//             gateway_ = g;
//         }
//     }
};

}
}

// Polymorphic serialization support 
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
CEREAL_REGISTER_TYPE(vulcan::hssh::AreaTransitionEvent);

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_TRANSITION_H
