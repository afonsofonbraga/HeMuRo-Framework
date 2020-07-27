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

    ros::init(map,"listener");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("turtle1/pose", 1000, &Alive::chatterCallbackPosition,this);
    //ros::Subscriber sub2 = n.subscribe("turtle1/color_sensor", 1000, &Alive::chatterCallbackColor, this);
  
    ros::spin();
}
