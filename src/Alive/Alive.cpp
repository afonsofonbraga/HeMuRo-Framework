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
    std::string node = vname + "_pose";
    ros::init(map,node);
    ros::NodeHandle n;
    
    ros::AsyncSpinner spinner(0); //This Will use as many threads as there are processors
    spinner.start();
    
    std::string topic = vname + "/pose";
    ros::Subscriber sub1 = n.subscribe(topic, 1000, &Alive::chatterCallbackPosition,this);
    //ros::Subscriber sub2 = n.subscribe("turtle1/color_sensor", 1000, &Alive::chatterCallbackColor, this);
    //ros::spin();
    
    std::map <std::string, ros::Publisher> publisherList;
    while (this->isRunning)
    {
        vROSBridgeMessage = new s_ROSBridgeMessage;
        this->monitor->getROSBridgeMessage(*vROSBridgeMessage);
        
        if (vROSBridgeMessage != nullptr && this->isRunning == true)
        {
            //Ainda esta com erro aqui, não sei como resolver.
            
            geometry_msgs::Twist msg;
            s_cmdvel vCmdvel = ((s_cmdvel*) vROSBridgeMessage)[0];
            msg.linear.x = vCmdvel.x;
            msg.angular.z = vCmdvel.theta;
            publisherList[vROSBridgeMessage->topicName].publish(vROSBridgeMessage->buffer);
        }
    }
    ros::waitForShutdown();
}
