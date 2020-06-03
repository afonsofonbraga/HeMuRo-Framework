//
//  DecomposableTask.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "DecomposableTask.hpp"

DecomposableTask::DecomposableTask(BlackBoard* vMonitor,  enum_DecomposableTask vtaskToBeDecomposed)
{
    std::vector<enum_AtomicTask> teste;
    teste.push_back(enum_AtomicTask::turnOn);
    teste.push_back(enum_AtomicTask::chargeBattery);
    enum_DecomposableTask lala = enum_DecomposableTask::checkPosition;
    
    avaliableTasks.insert_or_assign(lala , &teste);
    
    
    
    this->monitor = vMonitor;
    this->taskToBeDecomposed = vtaskToBeDecomposed;
    auto search = avaliableTasks.find(this->taskToBeDecomposed);

    if (search != avaliableTasks.end()) {
        std::cout << "Found " << '\n';
        this->vTasks = new std::vector<AtomicTask>;
        s_pose currentPosition;
        s_pose goalPosition;
        this->monitor->getPosition(currentPosition);
        
        AtomicTask* vAtomicTask = nullptr;
        
        for (auto n : *search->second){
            switch(n){
                case enum_AtomicTask::null :
                    break;
                case enum_AtomicTask::goTo :
                    vAtomicTask = new GoTo(currentPosition,goalPosition);
                    currentPosition = goalPosition;
                    break;
                case enum_AtomicTask::turnOn :
                    vAtomicTask = new TurnOn(currentPosition,goalPosition);
                    std::cout << "LIGA CARAI"<< std::endl;
                    break;
                case enum_AtomicTask::chargeBattery :
                    vAtomicTask = new ChargeBattery(currentPosition,goalPosition);
                    std::cout << "CARREGA DEMOIN"<< std::endl;
                    break;
            }
        }
        if (vAtomicTask != nullptr)
        {
            this->vTasks->push_back(*vAtomicTask);
            delete vAtomicTask;
        }
    } else {
        std::cout << "Not found\n";
    }
}

DecomposableTask::DecomposableTask(BlackBoard* vMonitor,  enum_DecomposableTask vtaskToBeDecomposed, std::vector<AtomicTask>& taskVector)
{
    std::vector<enum_AtomicTask> teste;
    teste.push_back(enum_AtomicTask::turnOn);
    teste.push_back(enum_AtomicTask::chargeBattery);
    enum_DecomposableTask lala = enum_DecomposableTask::checkPosition;
    
    avaliableTasks.insert_or_assign(lala , &teste);
    
    
    
    this->monitor = vMonitor;
    this->taskToBeDecomposed = vtaskToBeDecomposed;
    auto search = avaliableTasks.find(this->taskToBeDecomposed);

    if (search != avaliableTasks.end()) {
        std::cout << "Found " << '\n';
        this->vTasks = new std::vector<AtomicTask>;
        s_pose currentPosition;
        s_pose goalPosition;
        this->monitor->getPosition(currentPosition);
        
        AtomicTask* vAtomicTask = nullptr;
        
        for (auto n : *search->second){
            switch(n){
                case enum_AtomicTask::null :
                    break;
                case enum_AtomicTask::goTo :
                    vAtomicTask = new GoTo(currentPosition,goalPosition);
                    currentPosition = goalPosition;
                    break;
                case enum_AtomicTask::turnOn :
                    vAtomicTask = new TurnOn(currentPosition,goalPosition);
                    std::cout << "LIGA CARAI"<< std::endl;
                    break;
                case enum_AtomicTask::chargeBattery :
                    vAtomicTask = new ChargeBattery(currentPosition,goalPosition);
                    std::cout << "CARREGA DEMOIN"<< std::endl;
                    break;
            }
            if (vAtomicTask != nullptr)
            {
                taskVector.push_back(*vAtomicTask);
                delete vAtomicTask;
            } else {
                std::cout << "Not found\n";
            }
        }
    }
}








DecomposableTask::~DecomposableTask(){ }

void DecomposableTask::setAllCost()
{
    
}

float DecomposableTask::getAllCost()
{
    return cost;
}

bool DecomposableTask::isDecomposable()
{
    return decomposable;
}

