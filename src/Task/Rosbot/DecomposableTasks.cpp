//
//  DecomposableTasks.cpp
//  MRSMac
//
//  Created by Afonso Braga on 31/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "DecomposableTasks.hpp"

void decomposableTaskList(BlackBoard* monitor)
{
    
    std::vector<enum_AtomicTask> atomicTaskVector;
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    enum_DecomposableTask dTask = enum_DecomposableTask::checkPosition;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    dTask = enum_DecomposableTask::deliverPicture;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    dTask = enum_DecomposableTask::takePicture;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::chargeBattery);
    dTask = enum_DecomposableTask::lowBattery;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
}

void addAtomicTask2(BlackBoard* monitor, MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskList.clear();
    std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    monitor->getPosition(currentPosition);
    //std::queue<s_pose> goal = vMissionDecomposable.goal; // When working with the UDPReceiverSim this is necessary
    
    for (auto n : vMissionDecomposable.vAtomicTaskVector){
        switch(n){
            case enum_AtomicTask::null :
                break;
            case enum_AtomicTask::goTo :
                vAtomicTaskitem = std::make_shared<GoTo>(monitor, currentPosition,vMissionDecomposable.goal);
                currentPosition = vMissionDecomposable.goal;
                //goal.pop();
                break;
            case enum_AtomicTask::moveBaseGoal:
                vAtomicTaskitem = std::make_shared<moveBaseGoal>(monitor, currentPosition,vMissionDecomposable.goal);
                currentPosition = vMissionDecomposable.goal;
                //goal.pop();
                break;
            case enum_AtomicTask::turnOn :
                vAtomicTaskitem = std::make_shared<TurnOn>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = std::make_shared<ChargeBattery>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::takePicture :
                vAtomicTaskitem = std::make_shared<TakePicture>(monitor, currentPosition,currentPosition);
                break;
                
        }
        if (vAtomicTaskitem != nullptr)
        {
            vMissionDecomposable.atomicTaskList.push_back(std::move(vAtomicTaskitem));
            //delete vAtomicTaskitem;
        } else {
            std::cout << "Not found\n";
        }
    }
}

bool addAtomicTask(BlackBoard* monitor, MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskList.clear();
    std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    monitor->getPosition(currentPosition);
    
    char* temp = vMissionDecomposable.attributesBuffer;
    int vAttribuites = 0;
    int partialSize = 0;
    int totalSize = ((int*) temp)[0];
    partialSize += 4;
    temp += 4;
    
    for (auto n : vMissionDecomposable.vAtomicTaskVector){
        switch(n){
            case enum_AtomicTask::null :
                break;
            case enum_AtomicTask::goTo :
            {
                int vSize = ((int*) temp)[0];
                 partialSize += 4;
                 temp += 4;
                 vAttribuites++;
                 s_pose goal = ((s_pose*) temp)[0];
                 vAtomicTaskitem = std::make_shared<GoTo>(monitor, currentPosition, goal);
                 currentPosition = goal;
                 partialSize += vSize;
                 temp += vSize;
                 break;
            }
 
            case enum_AtomicTask::moveBaseGoal:
            {
                int vSize = ((int*) temp)[0];
                partialSize += 4;
                temp += 4;
                vAttribuites++;
                s_pose goal = ((s_pose*) temp)[0];
                vAtomicTaskitem = std::make_shared<moveBaseGoal>(monitor, currentPosition, goal);
                currentPosition = goal;
                partialSize += vSize;
                temp += vSize;
                break;
            }
            case enum_AtomicTask::turnOn :
                vAtomicTaskitem = std::make_shared<TurnOn>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = std::make_shared<ChargeBattery>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::takePicture :
                vAtomicTaskitem = std::make_shared<TakePicture>(monitor, currentPosition,currentPosition);
                break;
                
        }
        if (vAtomicTaskitem != nullptr)
        {
            vMissionDecomposable.atomicTaskList.push_back(std::move(vAtomicTaskitem));
            //delete vAtomicTaskitem;
        } else {
            std::cout << "Not found\n";
        }
    }
    return (totalSize == partialSize && vAttribuites == vMissionDecomposable.numberOfAttributes) ? true : false;
}
