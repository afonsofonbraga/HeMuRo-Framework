//
//  GoTo.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "GoTo.hpp"

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
                deltaError.z = this->endPosition.z - p.z;
                geometry_msgs::Twist msg;
                
                if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2) + pow(deltaError.z, 2)) <= 0.1)
                {
                    this->status = enum_AtomicTaskStatus::completed;
                } else
                {
                    s_ROSBridgeMessage teste;
                    s_pose caminho;
                    
                    caminho.x = this->endPosition.x;
                    caminho.y = this->endPosition.y;
                    caminho.z = this->endPosition.z;
                    strcpy(teste.topicName,"GoTo");
                    memmove(teste.buffer,(char*)&caminho,sizeof(caminho));
                    this->monitor->addROSBridgeMessage(teste);
                    bool a=true;
                    while (a)
                    {
                        usleep(10000000); //Vamos Precisar de um buffer

                        this->monitor->getPosition(p);

                        deltaError.x = this->endPosition.x - p.x;
                        deltaError.y = this->endPosition.y - p.y;
                        deltaError.z = this->endPosition.z - p.z;
                        std::cout << "Goal! X: " << endPosition.x << " Y: "<< endPosition.y <<" Z: " << endPosition.z << std::endl;
                        std::cout << "Posicao Robo! X: " << p.x << " Y: "<< p.y <<" Z: " << p.z << std::endl;
                        std::cout << "Erro Robo! X: " << deltaError.x << " Y: "<< deltaError.y <<" Z: " << deltaError.z << std::endl;
                        std::cout << sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2) + pow(deltaError.z, 2)) << std::endl;
                        if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2) + pow(deltaError.z, 2)) <= 0.2)
                        {this->status = enum_AtomicTaskStatus::completed;
                        a = false;}

                    }

                }
                
            }
            break;
        }
        case enum_AtomicTaskStatus::completed:
        {
            std::cout << "Arrived at the destination!"<< std::endl;
            break;
        }
        default:
            break;
    }
}
void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2)) * this->costMeter;
}


