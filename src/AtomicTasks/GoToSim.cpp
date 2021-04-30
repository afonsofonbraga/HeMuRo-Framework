//
//  GoToSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "GoToSim.hpp"

GoToSim::GoToSim(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    costFactor = factor * (battery_discharge /(robots_max_speed*3600))/battery_capacity;
    this->timeFactor = this->robots_max_speed;
    
    calculateCost();
    calculateTime();
}

GoToSim::~GoToSim(){}

void GoToSim::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            this->monitor->print("Going to the location -> x: " + std::to_string(this->endPosition.x) + " Y: " + std::to_string(this->endPosition.y));
            t0 = std::chrono::system_clock::now();
            this->status = enum_AtomicTaskStatus::running;
            break;
            
        case enum_AtomicTaskStatus::running:
        {
            std::this_thread::sleep_until(t0 + this->tick);
            
            if(this->status == enum_AtomicTaskStatus::running){
                s_pose p;
                this->monitor->getPosition(p);
                s_pose deltaError;
                
                deltaError.x = this->endPosition.x - p.x;
                deltaError.y = this->endPosition.y - p.y;
                deltaError.yaw = adjustAngle(this->endPosition.yaw - p.yaw);
                s_cmdvel vCmdvel;
                
                if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2)) <= 0.1)
                {
                    vCmdvel.x = 0;
                    vCmdvel.theta = 0;
                    //this->monitor->consumeBattery(this->cost);
                    this->status = enum_AtomicTaskStatus::completed;
                } else
                {
                    float ro = sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2));
                    float gama = atan2(deltaError.y, deltaError.x);
                    alpha_t = adjustAngle(gama - p.yaw);
                    float beta = adjustAngle(this->endPosition.yaw - gama);
                    
                    v = fmin(kp*ro, this->robots_max_speed);
                    
                    if((alpha_t > - M_PI && alpha_t < -M_PI/2) || (alpha_t > M_PI/2 && alpha_t >= M_PI))
                    {
                        v = -v;
                        alpha_t = adjustAngle(alpha_t + M_PI);
                        beta = adjustAngle(beta + M_PI);
                    }
                    vCmdvel.x = v;
                    vCmdvel.theta = ka*alpha_t + kb*beta;
                }
                float delta_x = vCmdvel.x*cos(p.yaw)*tick.count()/1000;
                float delta_y = vCmdvel.x*sin(p.yaw)*tick.count()/1000;
                this->monitor->consumeBattery(sqrt(pow(delta_x, 2) + pow(delta_y, 2))*this->costFactor);
                p.yaw += vCmdvel.theta*tick.count()/1000;
                p.x += delta_x;
                p.y += delta_y;
                
                this->monitor->setPosition(p);
            }
            
            t0 = t0 + this->tick;
            break;
        }
            
        case enum_AtomicTaskStatus::completed:
            break;
            
        default:
            break;
    }
}

void GoToSim::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costFactor;
}

void GoToSim::calculateTime()
{
    int time_seconds = round(sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2))/this->timeFactor);
    this->time = std::chrono::seconds(time_seconds);
}

float GoToSim::adjustAngle(float angle)
{
    angle = fmod(angle,2*M_PI);
    if (angle > M_PI)
        angle -= 2*M_PI;
    return angle;
}
