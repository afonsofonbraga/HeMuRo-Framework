//
//  Alive.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ROSModuleMavros.hpp"
#include <map>


ROSModuleMavros::ROSModuleMavros(Blackboard *monitor,ros::NodeHandle& vNode): Module(monitor), node(vNode)
{
    this->monitor->getRobotsName(vName);
    
    // Subscribers
    std::string topic = "/mavros/state";
    subscribersList["mavros/state"] = node.subscribe<mavros_msgs::State>(topic, 10, &ROSModuleMavros::state_cb, this);
    
    /*topic = "/mavros/local_position/pose";
    subscribersList["/mavros/local_position/pose"] = node.subscribe<geometry_msgs::PoseStamped>(topic, 10, &ROSModuleMavros::pose_cb, this);*/

    topic = "/mavros/global_position/local";
    subscribersList["/mavros/global_position/local"] = node.subscribe<nav_msgs::Odometry>(topic, 10, &ROSModuleMavros::pose_cb2,this);
    
    topic = "/mavros/global_position/compass_hdg";
    subscribersList["/mavros/global_position/compass_hdg"] = node.subscribe<std_msgs::Float64>(topic, 10, &ROSModuleMavros::heading_cb, this);

    topic = "/mavros/battery";
    subscribersList["/mavros/battery"] = node.subscribe<sensor_msgs::BatteryState>(topic, 10, &ROSModuleMavros::battery, this);
    
    // Publishers
    topic = "mavros/setpoint_raw/local";
    publishersList["mavros/setpoint_raw/local"] = node.advertise<mavros_msgs::PositionTarget>(topic, 10);
    
    topic = "mavros/setpoint_position/local";
    publishersList["mavros/setpoint_position/local"] = node.advertise<geometry_msgs::PoseStamped>(topic, 10);
    
}

ROSModuleMavros::~ROSModuleMavros()
{
    this->stop();
    this->monitor->conditional_ROSModuleMessageList.notify_one();
}

//get armed state
void ROSModuleMavros::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    this->current_state = *msg;
    bool connected = current_state.connected;
    bool armed = current_state.armed;
}

// Battery callback
void ROSModuleMavros::battery(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    this->monitor->setBatteryLevel(msg->percentage*100);
}

//get current position of drone
void ROSModuleMavros::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    this->current_pose = *msg;
    s_pose pose;
    //pose.x = this->current_pose.pose.position.x;
    //pose.y = this->current_pose.pose.position.x;
    //pose.z = this->current_pose.pose.position.z;
    //pose.yaw = this->current_heading.data;
    //this->monitor->setPosition(pose);
    
    //ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    // ROS_INFO("y: %f", current_pose.pose.position.y);
    // ROS_INFO("z: %f", current_pose.pose.position.z);
    std::string service = "/get_telemetry";
    ros::ServiceClient telemetry_client = node.serviceClient<clover::GetTelemetry>(service);
    clover::GetTelemetry tt;
    tt.request.frame_id = "map";
    telemetry_client.call(tt);
    pose.x = tt.response.x;
    pose.y = tt.response.y;
    pose.z = tt.response.z;
    pose.roll = tt.response.roll;
    pose.pitch = tt.response.pitch;
    pose.yaw = tt.response.yaw;
    this->monitor->setPosition(pose);
}

 void ROSModuleMavros::pose_cb2(const nav_msgs::Odometry::ConstPtr& msg)
 {
  //current_pose = *msg;
    //ROS_INFO("x: %f y: %f z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
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


//get compass heading
void ROSModuleMavros::heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
    this->current_heading = *msg;
    //ROS_INFO("current heading: %f", current_heading.data);
    s_pose pose;
    pose.x = this->current_pose.pose.position.x;
    pose.y = this->current_pose.pose.position.x;
    pose.z = this->current_pose.pose.position.z;
    pose.yaw = this->current_heading.data;
    //this->monitor->setPosition(pose);
}

//set orientation of the drone (drone should always be level)
void ROSModuleMavros::setHeading(float heading)
{
    heading = -heading + 90 - GYM_OFFSET;
    float yaw = heading*(M_PI/180);
    float pitch = 0;
    float roll = 0;
    
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    
    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;
    
    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    
}
// set position to fly to in the gym frame
void ROSModuleMavros::setDestination(float x, float y, float z)
{
    float deg2rad = (M_PI/180);
    float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
    float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
    float Z = z;
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;
    ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}


void ROSModuleMavros::run()
{
    vROSModuleMessage = new s_ROSModuleMessage;
    this->monitor->getROSModuleMessage(*vROSModuleMessage);
    
    if (vROSModuleMessage != nullptr && this->isRunning == true)
    {
        if (strcmp(vROSModuleMessage->topicName, "Arm" )== 0)
        {
            GYM_OFFSET = 0;
            for (int i = 1; i <= 1; ++i)
            {
                GYM_OFFSET += current_heading.data;
                ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
            }
            GYM_OFFSET /= 30;
            ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
            std::cout << GYM_OFFSET << "\n" << std::endl;
            
            //std::string topic =  "mavros/cmd/arming";
            std::string topic = "mavros/cmd/arming";
            ros::ServiceClient arming_client_i = node.serviceClient<mavros_msgs::CommandBool>(topic);
            mavros_msgs::CommandBool srv_arm_i;
            srv_arm_i.request.value = true;
            if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
                ROS_INFO("ARM sent %d", srv_arm_i.response.success);
            else
                ROS_ERROR("Failed arming");
        }
        if (strcmp(vROSModuleMessage->topicName, "Disarm" )== 0)
        {
            /*GYM_OFFSET = 0;
            for (int i = 1; i <= 1; ++i)
            {
                GYM_OFFSET += current_heading.data;
                ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
            }
            GYM_OFFSET /= 30;
            ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
            std::cout << GYM_OFFSET << "\n" << std::endl;
            */
            std::string topic = "mavros/cmd/disarming";
            ros::ServiceClient arming_client_i = node.serviceClient<mavros_msgs::CommandBool>(topic);
            mavros_msgs::CommandBool srv_arm_i;
            srv_arm_i.request.value = false;
            if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
                ROS_INFO("Disarm sent %d", srv_arm_i.response.success);
            else
                ROS_ERROR("Failed disarming");
        }
        else if(strcmp(vROSModuleMessage->topicName, "TakeOff") == 0)
        {
            s_pose vPose = ((s_pose*) vROSModuleMessage->buffer)[0];
            
            std::string service =  "/navigate";
            ros::ServiceClient nav_client = node.serviceClient<clover::Navigate>(service);
            clover::Navigate tt;
            tt.request.x = vPose.x;
            tt.request.y = vPose.y;
            tt.request.z = vPose.z;
            tt.request.speed = 1;
            tt.request.frame_id = "map";
            tt.request.auto_arm = true;
            if (nav_client.call(tt) && tt.response.success){
                ROS_INFO("Taking Off %d", tt.response.success);
            } else
            {
                ROS_ERROR("Failed Takeoff");
            }
        }
        else if(strcmp(vROSModuleMessage->topicName, "Land") == 0)
        {
            std::string service =  "/mavros/cmd/land";
            ros::ServiceClient land_client = node.serviceClient<mavros_msgs::CommandTOL>(service);
            mavros_msgs::CommandTOL srv_land;
            if (land_client.call(srv_land) && srv_land.response.success)
                ROS_INFO("land sent %d", srv_land.response.success);
            else
            {
                ROS_ERROR("Landing failed");
            }
        }
        else if(strcmp(vROSModuleMessage->topicName, "Navigate") == 0)
        {
            //move forward
            s_pose vPose = ((s_pose*) vROSModuleMessage->buffer)[0];
            
            std::string service =  "/navigate";
            ros::ServiceClient nav_client = node.serviceClient<clover::Navigate>(service);
            clover::Navigate tt;
            tt.request.x = vPose.x;
            tt.request.y = vPose.y;
            tt.request.z = vPose.z;
            tt.request.speed = 1;
            tt.request.frame_id = "map";
            tt.request.auto_arm = true;
            
            if (nav_client.call(tt) && tt.response.success)
                ROS_INFO("Going to Destination %d", tt.response.success);
            else
            {
                ROS_ERROR("Going to destination failed!");
            }
            /*
             setDestination(vPose.x, vPose.y, vPose.z);
             float tollorance = .35;
             if (local_pos_pub)
             {
             
             for (int i = 10000; ros::ok() && i > 0; --i)
             {
             
             local_pos_pub.publish(pose);
             // float percentErrorX = abs((pose.pose.position.x - current_pose.pose.position.x)/(pose.pose.position.x));
             // float percentErrorY = abs((pose.pose.position.y - current_pose.pose.position.y)/(pose.pose.position.y));
             // float percentErrorZ = abs((pose.pose.position.z - current_pose.pose.position.z)/(pose.pose.position.z));
             // std::cout << " px " << percentErrorX << " py " << percentErrorY << " pz " << percentErrorZ << std::endl;
             // if(percentErrorX < tollorance && percentErrorY < tollorance && percentErrorZ < tollorance)
             // {
             //   break;
             // }
             float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
             float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
             float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
             //std::cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << std::endl;
             float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
             std::cout << dMag << std::endl;
             if( dMag < tollorance)
             {
             break;
             }
             //ros::spinOnce();
             //ros::Duration(0.5).sleep();
             //if(i == 1)
             //{
             //    ROS_INFO("Failed to reach destination. Stepping to next task.");
             //}
             }
             ROS_INFO("Going to destination!!");
             //ROS_INFO("Done moving foreward.");
             }*/
            
            
            
            
        }
    }
}
