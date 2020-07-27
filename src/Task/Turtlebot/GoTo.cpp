//
//  GoTo.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "GoTo.hpp"
#include <map>

GoTo::GoTo(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

GoTo::~GoTo(){}

void GoTo::run()
{
    std::map<std::string, std::string> map;
    ros::init(map,"speed");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("turtle1/cmd_vel", 1000);
    ros::Rate loop_rate(10);
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            std::cout << "Going to the location."<< std::endl;
            this->status = enum_AtomicTaskStatus::running;
            break;
            
        case enum_AtomicTaskStatus::running:
        {
            s_pose p;
            this->monitor->getPosition(p);
            s_pose deltaError;
            
            deltaError.x = this->endPosition.x - p.x;
            deltaError.y = this->endPosition.y - p.y;
            deltaError.theta = this->endPosition.theta - p.theta;
            
            if(deltaError.x <= 0.1 && deltaError.y <= 0.1 && deltaError.theta <= 0.1)
            {
                std::cout << "Arrived at the destination!"<< std::endl;
                this->status = enum_AtomicTaskStatus::completed;
            } else
            {
                float ro = sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2));
                alpha_t_old = alpha_t;
                alpha_t = atan2(deltaError.y, deltaError.x);
                if(fmod(alpha_t, 2*M_PI) > M_PI)
                    alpha_t= alpha_t - 2*M_PI;
                v = fminf(ro, 1.0);
                sum_Alpha_t += alpha_t;
                omega = kp * alpha_t + ki * sum_Alpha_t + kd * (alpha_t - alpha_t_old);
            }
            geometry_msgs::Twist msg;
            msg.linear.x = v;
            msg.angular.z = omega;
            chatter_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
            break;
        case enum_AtomicTaskStatus::completed:
            break;
    }
}

void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

