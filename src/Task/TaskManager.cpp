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
    this->monitor->conditional_missionTask.notify_one();
}

void TaskManager::run()
{
    if (performingTask== false && this->isRunning == true)
    {
        vTask = this->monitor->getTaskFromMission();
        if (vTask != nullptr && this->isRunning == true)
        {
            if(vTask->getStatus() == enum_AtomicTaskStatus::waiting)
            {
                this -> performingTask = true;
                vTask->run();
            }
        }
    }
    else if (this->isRunning == true)
    {
        if (vTask->getStatus() == enum_AtomicTaskStatus::completed)
        {
            this-> performingTask = false;
            // SIMULATION HERE WHEN A TASK IS FINISHED ITS COST IS REDUCED FROM THE BATTERY, THIS LINE SHOULD BE REMOVED LATER
            this->monitor->consumeBattery(vTask->getCost());
            std::cout << "Battery level: " << this->monitor->getBatteryLevel() << std::endl;
        }
        else
            vTask->run();
    }
    else
            std::cout << "Vazio" << std::endl;
}
