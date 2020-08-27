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
  pose.x = msg->x;
  pose.y = msg->y;
  pose.theta = msg->theta;

  this->monitor->setPosition(pose);
}

Alive::Alive(BlackBoard *monitor): Module(monitor)
{

}

Alive::~Alive()
{
    ros::shutdown();
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
    topic = vname + "cmd_vel";
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
                publisherList[vROSBridgeMessage->topicName].publish(vROSBridgeMessage->buffer);
            }
        }
    }
    spinner.stop();
}


