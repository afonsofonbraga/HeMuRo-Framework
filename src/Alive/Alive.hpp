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
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"
#include "turtlesim/Color.h"



class Alive: public Module
{
protected:
    s_ROSBridgeMessage* vROSBridgeMessage;
    virtual void run() override;
public:
    
    Alive(BlackBoard* monitor);
    ~Alive();
     void chatterCallbackPosition(const turtlesim::Pose::ConstPtr& msg);
     void chatterCallbackColor(const turtlesim::Color::ConstPtr& msg);
    //void error(const char* msg);                    // Print an error
};

#endif /* Alive_hpp */

