//
//  GoTo.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
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
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
        {
            this->monitor->print("Going to the location -> x: " + std::to_string(this->endPosition.x) + " Y: " + std::to_string(this->endPosition.y));
            this->status = enum_AtomicTaskStatus::running;
            t0 = std::chrono::system_clock::now();
            //std::cout << "x: " << this->endPosition.x << " Y: " << this->endPosition.y << std::endl;
            break;
        }
            
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
                
                //std::cout << "Erro: X: " << deltaError.x << " Y: "<< deltaError.y << " total: "<< sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2))<<std::endl;
                
                if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2)) <= 0.1)
                {
                    vCmdvel.x = 0;
                    vCmdvel.theta = 0;
                    this->status = enum_AtomicTaskStatus::completed;
                } else
                {
                    float ro = sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2));
                    float gama = atan2(deltaError.y, deltaError.x);
                    alpha_t = adjustAngle(gama - p.yaw);
                    float beta = adjustAngle(this->endPosition.yaw - gama);
                    
                    v = fmin(kp*ro, 0.5);
                    
                    if((alpha_t > - M_PI && alpha_t < -M_PI/2) || (alpha_t > M_PI/2 && alpha_t >= M_PI))
                    {
                        // I2
                        v = -v;
                        alpha_t = adjustAngle(alpha_t + M_PI);
                        beta = adjustAngle(beta + M_PI);
                    }
                    
                    vCmdvel.x = v;
                    vCmdvel.theta = ka*alpha_t + kb*beta;
                    
                }
                
                s_ROSBridgeMessage teste;
                strcpy(teste.topicName,"GoTo");
                memmove(teste.buffer,(char*)&vCmdvel,sizeof(vCmdvel));
                this->monitor->addROSBridgeMessage(teste);
            }
            
            t0 = t0 + this->tick;
        }
            break;
        case enum_AtomicTaskStatus::completed:
            this->monitor->print("Arrived at the destination!");
            break;
    }
}

void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

float GoTo::adjustAngle(float angle)
{
    angle = fmod(angle,2*M_PI);
    if (angle > M_PI)
        angle -= 2*M_PI;
    return angle;
}
