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
#include <cmath>
#include <math.h>
#include <sys/time.h>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"

#include "ros/ros.h"
#include <ros/duration.h>
//#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <clover/Navigate.h>
#include <clover/GetTelemetry.h>
class Alive: public Module
{
protected:
    s_ROSBridgeMessage* vROSBridgeMessage;
    virtual void run() override;
    std::map <std::string, ros::Publisher> publishersList;
    std::map <std::string, ros::Subscriber> subscribersList;
    ros::NodeHandle& node;
    std::string vName;
    
    //MAVROS global variables
    mavros_msgs::State current_state;
    //nav_msgs::Odometry current_pose;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped pose;
    std_msgs::Float64 current_heading;
    float GYM_OFFSET;
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void heading_cb(const std_msgs::Float64::ConstPtr& msg);
    
    void setDestination(float x, float y, float z);
    void setHeading(float heading);
    
    
public:
    
    Alive(BlackBoard* monitor, ros::NodeHandle& vNode);
    ~Alive();
    //void error(const char* msg);                    // Print an error
};

#endif /* Alive_hpp */
