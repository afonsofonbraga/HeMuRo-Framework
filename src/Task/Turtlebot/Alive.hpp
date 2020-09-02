//
//  Alive.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Alive_hpp
#define Alive_hpp

#include <string.h> /* memset() */
#include <chrono> /* select() */
#include <iostream>
#include <thread>
#include <sys/time.h>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Color.h"

class Alive: public Module
{
protected:
    s_ROSBridgeMessage* vROSBridgeMessage;
    void callbackPosition(const turtlesim::Pose::ConstPtr& msg);
    void callbackColor(const turtlesim::Color::ConstPtr& msg);
    void callbackBatteryPercentage(const std_msgs::Int32::ConstPtr& msg);
    virtual void run() override;
    std::map <std::string, ros::Publisher> publishersList;
    std::map <std::string, ros::Subscriber> subscribersList;
    ros::NodeHandle& node;
    std::string vName;
public:
    
    Alive(BlackBoard* monitor, ros::NodeHandle& vNode);
    ~Alive();

    //void error(const char* msg);                    // Print an error
};

#endif /* Alive_hpp */
