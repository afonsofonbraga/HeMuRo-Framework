//
//  ROSModuleRosbot.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright ? 2020 Afonso Braga. All rights reserved.
//

#include "ROSModuleRosbot.hpp"
#include <map>





ROSModuleRosbot::ROSModuleRosbot(Blackboard *monitor,ros::NodeHandle& vNode): Module(monitor), node(vNode)
{
    this->monitor->getRobotsName(vName);
    
    //Subscribers
    std::string topic = vName + "/odom";
    subscribersList["odom"] = node.subscribe(topic, 1000, &ROSModuleRosbot::callbackOdometry,this);
    
    topic = vName + "/battery/percent";
    subscribersList["battery/percent"] = node.subscribe(topic, 1, &ROSModuleRosbot::callbackBatteryPercentage,this);
    
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
    
    topic = vName + "/move_base/make_plan";
    check_path = new ros::ServiceClient(node.serviceClient<nav_msgs::GetPlan>(topic));
}

ROSModuleRosbot::~ROSModuleRosbot()
{
    this->stop();
    this->monitor->conditional_ROSModuleMessageList.notify_one();
    
}

void ROSModuleRosbot::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
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

void ROSModuleRosbot::callbackBatteryPercentage(const std_msgs::Int32::ConstPtr& msg)
{
    this->monitor->setBatteryLevel(msg->data);
}

void ROSModuleRosbot::run()
{
    vROSModuleMessage = new s_ROSModuleMessage;
    
    this->monitor->getROSModuleMessage(*vROSModuleMessage);
    if (vROSModuleMessage != nullptr && this->isRunning == true)
    {
        if(strcmp(vROSModuleMessage->topicName, "GoTo") == 0)
        {
            //move forward
            s_cmdvel vCmdvel = ((s_cmdvel*) vROSModuleMessage->buffer)[0];
            geometry_msgs::Twist msg;
            msg.linear.x = vCmdvel.x;
            msg.angular.z = vCmdvel.theta;
            this->publishersList["cmd_vel"].publish(msg);
        }
        if(strcmp(vROSModuleMessage->topicName, "ChargeBattery") == 0)
        {
            //recharge battery
            bool rechargeStatus = ((bool*)vROSModuleMessage->buffer)[0];
            std_msgs::Bool msg;
            msg.data = rechargeStatus;
            std::cout << "Sending ROS command! " << msg.data << std::endl;
            this->publishersList["battery/recharge"].publish(msg);
        }
        if(strcmp(vROSModuleMessage->topicName, "Move_base/Goal") == 0)
        {
            s_pose vPose = ((s_pose*) vROSModuleMessage->buffer)[0];
            
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
        if(strcmp(vROSModuleMessage->topicName, "Move_base/Cancel") == 0)
        {
            ROS_INFO("Canceling goal");
            this->ac->cancelGoal();
        }
        if(strcmp(vROSModuleMessage->topicName, "Move_base/Cost") == 0)
        {
            s_pose vPose = ((s_pose*) vROSModuleMessage->buffer)[0];
            s_pose robotPosition;
            this->monitor->getPosition(robotPosition);
            
            geometry_msgs::PoseStamped Start;
            Start.header.seq = 0;
            Start.header.stamp = ros::Time::now();
            Start.header.frame_id = "map";
            Start.pose.position.x = robotPosition.x;
            Start.pose.position.y = robotPosition.y;
            Start.pose.position.z = robotPosition.z;
            Start.pose.orientation.x = robotPosition.roll;
            Start.pose.orientation.y = robotPosition.pitch;
            Start.pose.orientation.w = robotPosition.yaw;
            
            geometry_msgs::PoseStamped Goal;
            Goal.header.seq = 0;
            Goal.header.stamp = ros::Time::now();
            Goal.header.frame_id = "map";
            Goal.pose.position.x = vPose.x;
            Goal.pose.position.y = vPose.y;
            Goal.pose.position.z = vPose.z;
            Goal.pose.orientation.x = vPose.roll;
            Goal.pose.orientation.y = vPose.pitch;
            Goal.pose.orientation.w = vPose.yaw;
            
            
            nav_msgs::GetPlan srv;
            srv.request.start = Start;
            srv.request.goal = Goal;
            srv.request.tolerance = 1.5;
            
            ROS_INFO("Make plan: %d", (check_path->call(srv) ? 1 : 0));
            ROS_INFO("Plan size: %d", srv.response.plan.poses.size());
            
            float distance = 0;
            for(int i = 1; i< srv.response.plan.poses.size(); i++)
            {
                float diffX = (srv.response.plan.poses[i].pose.position.x - srv.response.plan.poses[i-1].pose.position.x);
                float diffY = (srv.response.plan.poses[i].pose.position.y - srv.response.plan.poses[i-1].pose.position.y);
                float diffZ = (srv.response.plan.poses[i].pose.position.z - srv.response.plan.poses[i-1].pose.position.z);
                distance += sqrtf(diffX*diffX + diffY*diffY + diffZ*diffZ);
            }
            //ROS_INFO("Plan Costs: %f", distance);
            this->monitor->print("Plan costs: " + std::to_string(distance));
            //float* cost;
            //memcpy(&cost, vROSModuleMessage->buffer + sizeof(s_pose), sizeof(float *));
            //*cost = distance;
        }
    }
}



