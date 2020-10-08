//
//  Alive.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright ? 2020 Afonso Braga. All rights reserved.
//

#include "Alive.hpp"
#include <map>





Alive::Alive(BlackBoard *monitor,ros::NodeHandle& vNode): Module(monitor), node(vNode)
{
    this->monitor->getRobotsName(vName);
    
    //Subscribers
    std::string topic = vName + "/odom";
    subscribersList["odom"] = node.subscribe(topic, 1000, &Alive::callbackOdometry,this);
    
    topic = vName + "/battery/percent";
    subscribersList["battery/percent"] = node.subscribe(topic, 1, &Alive::callbackBatteryPercentage,this);
    
    //Publishers
    topic = vName + "/cmd_vel";
    publishersList["cmd_vel"] = node.advertise<geometry_msgs::Twist>(topic, 10);
    
    topic = vName + "/battery/recharge";
    publishersList["battery/recharge"] = node.advertise<std_msgs::Bool>(topic, 10);
    
    topic = vName + "/move_base";
    ac = new MoveBaseClient(topic.c_str(),true);
    //wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
}

Alive::~Alive()
{
    this->stop();
    this->monitor->conditional_ROSBridgeMessageList.notify_one();
    
}

void Alive::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
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

void Alive::callbackBatteryPercentage(const std_msgs::Int32::ConstPtr& msg)
{
    this->monitor->setBatteryLevel(msg->data);
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
        if(strcmp(vROSBridgeMessage->topicName, "ChargeBattery") == 0)
        {
            //recharge battery
            bool rechargeStatus = ((bool*)vROSBridgeMessage->buffer)[0];
            std_msgs::Bool msg;
            msg.data = rechargeStatus;
            std::cout << "Sending ROS command! " << msg.data << std::endl;
            this->publishersList["battery/recharge"].publish(msg);
        }
        if(strcmp(vROSBridgeMessage->topicName, "Move_base/Goal") == 0)
        {
            s_pose vPose = ((s_pose*) vROSBridgeMessage->buffer)[0];
            
            move_base_msgs::MoveBaseGoal msg;
            
            msg.target_pose.header.frame_id = "map";
            msg.target_pose.header.stamp = ros::Time::now();
            
            msg.target_pose.pose.position.x = vPose.x;
            msg.target_pose.pose.position.y = vPose.y;
            msg.target_pose.pose.position.z = vPose.z;
            msg.target_pose.pose.orientation.w = 1.0;
            
            ROS_INFO("Sending goal");
            this->ac->sendGoal(msg);
            
            /*
             this->ac->waitForResult();
             if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
             ROS_INFO("Hooray, the base moved 1 meter forward");
             else
             ROS_INFO("The base failed to move forward 1 meter for some reason");
             */
        }
        if(strcmp(vROSBridgeMessage->topicName, "Move_base/Cancel") == 0)
        {
            ROS_INFO("Canceling goal");
            this->ac->cancelGoal();
        }
    }
}




