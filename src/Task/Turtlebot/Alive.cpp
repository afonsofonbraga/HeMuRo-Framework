//
//  Alive.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Alive.hpp"
#include <map>

void Alive::chatterCallbackPosition(const turtlesim::Pose::ConstPtr& msg)
{
    //ROS_INFO("TurtleBot Position: [%f, %f, %f]", msg->x,msg->y,msg->theta);
    s_pose pose;
    pose.x = msg->x;
    pose.y = msg->y;
    pose.theta = msg->theta;
    
    this->monitor->setPosition(pose);
}

void Alive::chatterCallbackColor(const turtlesim::Color::ConstPtr& msg)
{
    ROS_INFO("TurtleBot Color: [%i, %i, %i]", msg->r,msg->g,msg->b);
}

Alive::Alive(BlackBoard *monitor,ros::NodeHandle& vNode): Module(monitor), node(vNode)
{
    this->monitor->getRobotsName(vName);
    
    // Subscribers
    std::string topic = vname + "/pose";
    subscribersList["pose"] = node.subscribe(topic, 1000, &Alive::chatterCallbackPosition,this);
    
    // Publishers
    topic = vName + "/cmd_vel";
    publishersList["cmd_vel"] = node.advertise<geometry_msgs::Twist>(topic, 10);
}

Alive::~Alive()
{
    this->stop();
    this->monitor->conditional_ROSBridgeMessageList.notify_one();
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
