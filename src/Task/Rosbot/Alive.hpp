//
//  Alive.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Alive_hpp
#define Alive_hpp

#include <string.h> /* memset() */
#include <chrono> /* select() */
#include <iostream>
#include <thread>
#include <cmath>
#include <math.h>
#include <sys/time.h>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"

#include "ros/ros.h"
#include <ros/duration.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>


class Alive: public Module
{
protected:
    s_ROSBridgeMessage* vROSBridgeMessage;
    void chatterCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
    virtual void run() override;
public:
    
    Alive(BlackBoard* monitor);
    ~Alive();
    
    //void error(const char* msg);                    // Print an error
};
#endif /* Alive_hpp */
