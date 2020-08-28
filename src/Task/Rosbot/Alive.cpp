//
//  Alive.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Alive.hpp"
#include <map>


Alive::Alive(BlackBoard *monitor): Module(monitor)
{
    
}

Alive::~Alive()
{
    ros::shutdown();
}

void Alive::chatterCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("TurtleBot Position: [%f, %f, %f]", msg->x,msg->y,msg->theta);
    s_pose pose;
    
/*
    pose.x = msg->twist.twist.linear.x;
    pose.y = msg->twist.twist.linear.y;
    pose.z = msg->twist.twist.linear.z;
    pose.roll = msg->twist.twist.angular.x;
    pose.pitch = msg->twist.twist.angular.y;
    pose.yaw = msg->twist.twist.angular.z;
*/
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.z = msg->pose.pose.position.z;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
    
    this->monitor->setPosition(pose);
}


void Alive::run()
{
    std::map<std::string, std::string> map;
    std::string vname;
    this->monitor->getRobotsName(vname);
    std::string node = vname + "_rosbot";
    ros::init(map,node);
    ros::NodeHandle n;
    
    ros::AsyncSpinner spinner(0); //This Will use as many threads as there are processors
    spinner.start();
    
    std::string topic = vname + "/odom";
    ros::Subscriber sub1 = n.subscribe(topic, 1000, &Alive::chatterCallbackOdometry,this);
    
    // Publishers
    topic = vname + "/cmd_vel";
    ros::Publisher set_vel_pub = n.advertise<geometry_msgs::Twist>(topic, 10);
    
    std::map <std::string, ros::Publisher> publisherList;
    while (this->isRunning)
    {
        vROSBridgeMessage = new s_ROSBridgeMessage;
        this->monitor->getROSBridgeMessage(*vROSBridgeMessage);
        
        if (vROSBridgeMessage != nullptr && this->isRunning == true)
        {
            if(strcmp(vROSBridgeMessage->topicName, "GoTo") == 0)
            {
                //move forward
                s_cmdvel vCmdvel = ((s_cmdvel*) vROSBridgeMessage->buffer)[0];
                geometry_msgs::Twist msg;
                msg.linear.x = vCmdvel.x;
                msg.angular.z = vCmdvel.theta;
                set_vel_pub.publish(msg);
            }
        }
    }
    spinner.stop();
}

