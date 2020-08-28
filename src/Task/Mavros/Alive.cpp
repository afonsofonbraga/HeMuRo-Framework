//
//  Alive.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Alive.hpp"
#include <map>

std::map<std::string, std::string> map;
std::string vname;


Alive::Alive(BlackBoard *monitor): Module(monitor)
{
    
}

Alive::~Alive()
{
    ros::shutdown();
}

//get armed state
void Alive::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    this->current_state = *msg;
    bool connected = current_state.connected;
    bool armed = current_state.armed;
}

//get current position of drone
void Alive::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
    ros::NodeHandle n;
    std::string service = "/get_telemetry";
    ros::ServiceClient telemetry_client = n.serviceClient<clover::GetTelemetry>(service);
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
// void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//  current_pose = *msg;
//  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
// }


//get compass heading
void Alive::heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
    this->current_heading = *msg;
    //ROS_INFO("current heading: %f", current_heading.data);
    s_pose pose;
    pose.x = this->current_pose.pose.position.x;
    pose.y = this->current_pose.pose.position.x;
    pose.z = this->current_pose.pose.position.z;
    pose.yaw = this->current_heading.data;
    this->monitor->setPosition(pose);
}

//set orientation of the drone (drone should always be level)
void Alive::setHeading(float heading)
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
void Alive::setDestination(float x, float y, float z)
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


void Alive::run()
{
    this->monitor->getRobotsName(vname);
    std::string node = vname + "_mavros";

    vname = "";
    ros::init(map,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0); //This Will use as many threads as there are processors
    spinner.start();

    // Subscribers
    std::string topic = vname + "mavros/state";
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>(topic, 10, &Alive::state_cb, this);
    
    topic = vname + "/mavros/local_position/pose";
    ros::Subscriber currentPos = n.subscribe<geometry_msgs::PoseStamped>(topic, 10, &Alive::pose_cb, this);
    
    topic = vname + "/mavros/global_position/compass_hdg";
    ros::Subscriber currentHeading = n.subscribe<std_msgs::Float64>(topic, 10, &Alive::heading_cb, this);
    
    // Publishers
    topic = vname + "mavros/setpoint_raw/local";
    ros::Publisher set_vel_pub = n.advertise<mavros_msgs::PositionTarget>(topic, 10);
    
    topic = vname + "mavros/setpoint_position/local";
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>(topic, 10);
    
    std::map <std::string, ros::Publisher> publisherList;
    
    while (this->isRunning)
    {
        vROSBridgeMessage = new s_ROSBridgeMessage;
        this->monitor->getROSBridgeMessage(*vROSBridgeMessage);
        
        if (vROSBridgeMessage != nullptr && this->isRunning == true)
        {
        
            if (strcmp(vROSBridgeMessage->topicName, "Arm" )== 0)
            {
                GYM_OFFSET = 0;
                for (int i = 1; i <= 1; ++i)
                {
                    //ros::spinOnce();
                    //ros::Duration(0.1).sleep();
                    GYM_OFFSET += current_heading.data;
                    ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
                }
                GYM_OFFSET /= 30;
                ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
                std::cout << GYM_OFFSET << "\n" << std::endl;
                
                std::string topic = vname + "mavros/cmd/arming";
                ros::ServiceClient arming_client_i = n.serviceClient<mavros_msgs::CommandBool>(topic);
                mavros_msgs::CommandBool srv_arm_i;
                srv_arm_i.request.value = true;
                if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
                    ROS_INFO("ARM sent %d", srv_arm_i.response.success);
                else
                    ROS_ERROR("Failed arming");
            }
            else if(strcmp(vROSBridgeMessage->topicName, "TakeOff") == 0)
            {
                std::string service = vname + "/mavros/cmd/takeoff";
                ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>(service);
                mavros_msgs::CommandTOL srv_takeoff;
                
                s_pose vPose = ((s_pose*) vROSBridgeMessage->buffer)[0];
                srv_takeoff.request.altitude = vPose.z;
                if(takeoff_cl.call(srv_takeoff))
                {
                    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
                } else
                {
                    ROS_ERROR("Failed Takeoff");
                }
            }
            else if(strcmp(vROSBridgeMessage->topicName, "Land") == 0)
            {
                std::string service = vname + "/mavros/cmd/land";
                ros::ServiceClient land_client = n.serviceClient<mavros_msgs::CommandTOL>(service);
                mavros_msgs::CommandTOL srv_land;
                if (land_client.call(srv_land) && srv_land.response.success)
                    ROS_INFO("land sent %d", srv_land.response.success);
                else
                {
                    ROS_ERROR("Landing failed");
                }
            }
            else if(strcmp(vROSBridgeMessage->topicName, "GoTo") == 0)
            {
                
                //move forward
                setHeading(0);
                s_pose vPose = ((s_pose*) vROSBridgeMessage->buffer)[0];

                std::string service = vname + "/navigate";
                ros::ServiceClient nav_client = n.serviceClient<clover::Navigate>(service);
                clover::Navigate tt;
                tt.request.x = vPose.x;
                tt.request.y = vPose.y;
                tt.request.z = vPose.z;
                tt.request.speed = 1;
                tt.request.frame_id = "map";
                tt.request.auto_arm = true;

                if (nav_client.call(tt) && tt.response.success)
                    ROS_INFO("VAAAAI DEMOIN %d", tt.response.success);
                else
                {
                    ROS_ERROR("Landing failed");
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
    spinner.stop();
 //   ros::waitForShutdown();
}
