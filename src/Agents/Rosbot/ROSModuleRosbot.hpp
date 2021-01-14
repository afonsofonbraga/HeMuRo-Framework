//
//  ROSModuleRosbot.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ROSModuleRosbot_hpp
#define ROSModuleRosbot_hpp

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
#include <nav_msgs/GetPlan.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ROSModuleRosbot: public Module
{
protected:
    s_ROSBridgeMessage* vROSBridgeMessage;
    MoveBaseClient* ac;
    ros::ServiceClient* check_path;
    void callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
    void callbackBatteryPercentage(const std_msgs::Int32::ConstPtr& msg);
    virtual void run() override;
    std::map <std::string, ros::Publisher> publishersList;
    std::map <std::string, ros::Subscriber> subscribersList;
    ros::NodeHandle& node;
    std::string vName;
public:
    
    ROSModuleRosbot(BlackBoard* monitor, ros::NodeHandle& vNode);
    ~ROSModuleRosbot();
    //void error(const char* msg);                    // Print an error
};
#endif /* ROSModuleRosbot_hpp */