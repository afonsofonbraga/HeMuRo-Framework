//
//  TaskManager.cpp
//  MRSMac
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TaskManager.hpp"

TaskManager::TaskManager(BlackBoard* monitor): Module(monitor)
{
    
}

TaskManager::~TaskManager()
{
    
}

void TaskManager::run()
{
    if (performingTask== false && this->isRunning == true)
    {
        vTask = new Task();
        this->monitor->getTask(*vTask);
        
        if (vTask != nullptr && this->isRunning == true)
        {
            std::cout<< "Removed: "<<vTask->taskName <<std::endl;
            this -> performingTask = true;
            taskSwitch(*vTask);
        }
    }
    else if (this->isRunning == true)
        taskSwitch(*vTask);
        else
            std::cout << "Vazio" << std::endl;
}

void TaskManager::taskSwitch(Task& menu)
{
    switch(menu.taskName)
    {
        case TaskDescription::null:
            break;
            
        case TaskDescription::chargeBattery :
            charging();
        break;
            
        case TaskDescription::turnOn :
            turnOn();
        break;
            
        case TaskDescription::goTo :
            goToPosition();
            break;
    }
}


void TaskManager::charging()
{
    if (monitor->getBatteryLevel() < 100)
    {
            monitor->chargeBattery(1.0);
            std::cout << "Carregando... " << this->monitor->getBatteryLevel() << "% " << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
        std::cout << "Bateria Carregada!" << std::endl;
        this->performingTask = false;
    }
}

void TaskManager::turnOn()
{
    if (monitor->getBatteryLevel() > 50)
    {
        monitor->consumeBattery(1.0);
        std::cout << "Ligado... " << this->monitor->getBatteryLevel() << "%"<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
        this->performingTask = false;
    }
}

void TaskManager::goToPosition()
{
    if(this->goal == nullptr)
    {
        this->goal = new s_pose;
        memcpy(goal, vTask->atributes, sizeof(s_pose));
    }
    
    s_pose* actualPosition = new s_pose;
    monitor->getPosition(*actualPosition);
    float e = 0.1;
    if(abs(goal->x - actualPosition->x) >= e || abs(goal->y - actualPosition->y) >= e)
    {
        actualPosition->x += (goal->x - actualPosition->x)* 0.1;
        actualPosition->y += (goal->y - actualPosition->y)* 0.1;
        actualPosition->theta += (goal->theta - actualPosition->theta)* 0.1;
        monitor->setPosition(*actualPosition);
        monitor->consumeBattery(1.0);
        std::cout << "Stil moving! At (" << actualPosition->x << " , " << actualPosition->y << " , " << actualPosition->theta << "). Battery Level: " << this->monitor->getBatteryLevel() << "%"<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else
    {
        std::cout << "Arrived at destination! (" << actualPosition->x << " , " << actualPosition->y << " , " << actualPosition->theta << ")." << std::endl;
        
        
        //Dar uma olhada pq não esta deletando os ponteiros
        this->goal = nullptr;
        actualPosition = nullptr;
        delete this->goal;
        delete actualPosition;
        
        
        this->performingTask = false;
    }
    
}
