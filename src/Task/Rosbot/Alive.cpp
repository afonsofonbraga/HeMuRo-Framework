//
//  Alive.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright � 2020 Afonso Braga. All rights reserved.
//

#include "Alive.hpp"
#include <map>


Alive::Alive(BlackBoard *monitor,ros::NodeHandle& vNode): Module(monitor), node(vNode)
{
    this->monitor->getRobotsName(vName);
    std::string topic = vName + "/odom";
    subscribersList["odom"] = node.subscribe(topic, 1000, &Alive::chatterCallbackOdometry,this);
    
    topic = vName + "/cmd_vel";
    publishersList["cmd_vel"] = node.advertise<geometry_msgs::Twist>(topic, 10);
}

Alive::~Alive()
{
    this->stop();
    this->monitor->conditional_ROSBridgeMessageList.notify_one();

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
                this->publishersList["cmd_vel"].publish(msg);
            }
        }
}


