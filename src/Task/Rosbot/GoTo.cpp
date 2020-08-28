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
            std::cout << "Going to the location."<< std::endl;
            this->status = enum_AtomicTaskStatus::running;
            break;
        }
            
        case enum_AtomicTaskStatus::running:
        {
            if(this->status != enum_AtomicTaskStatus::completed){
                s_pose p;
                this->monitor->getPosition(p);
                s_pose deltaError;
                
                deltaError.x = this->endPosition.x - p.x;
                deltaError.y = this->endPosition.y - p.y;
                deltaError.yaw = this->endPosition.yaw - p.yaw;
                s_cmdvel vCmdvel;
                
                if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2)) <= 0.1)
                {
                    vCmdvel.x = 0;
                    vCmdvel.theta = 0;
                    this->status = enum_AtomicTaskStatus::completed;
                } else
                {
                    float ro = sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2));
                    alpha_t_old = alpha_t;
                    alpha_t = atan2(deltaError.y, deltaError.x) - p.yaw;
                    if(alpha_t > M_PI)
                        alpha_t = alpha_t - 2*M_PI ;
                    if (alpha_t < -M_PI)
                        alpha_t = alpha_t + 2*M_PI ;
                    
                    v = fmin(ro, 0.5);
                    sum_Alpha_t += alpha_t;
                    omega = kp * alpha_t + ki * sum_Alpha_t + kd * (alpha_t - alpha_t_old);
                    
                    // Limitando a atuação
                    if (omega > M_PI)
                        omega = M_PI;
                    else if (omega < - M_PI)
                        omega = - M_PI;
                    
                    vCmdvel.x = v;
                    vCmdvel.theta = omega;
                    
                }
                
                s_ROSBridgeMessage teste;
                strcpy(teste.topicName,"GoTo");
                memmove(teste.buffer,(char*)&vCmdvel,sizeof(vCmdvel));
                this->monitor->addROSBridgeMessage(teste);
                usleep(100000); //Vamos Precisar de um buffer
            }
        }
            break;
        case enum_AtomicTaskStatus::completed:
            std::cout << "Arrived at the destination!"<< std::endl;
            break;
    }
}

void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

