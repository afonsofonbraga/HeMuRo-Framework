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
    
    std::vector<enum_AtomicTask> teste;
    teste.push_back(enum_AtomicTask::goTo);
    enum_DecomposableTask lala = enum_DecomposableTask::checkPosition;
    monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::goTo);
    teste.push_back(enum_AtomicTask::takePicture);
    teste.push_back(enum_AtomicTask::goTo);
    lala = enum_DecomposableTask::deliverPicture;
    monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::goTo);
    teste.push_back(enum_AtomicTask::takePicture);
    lala = enum_DecomposableTask::takePicture;
    monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::goTo);
    teste.push_back(enum_AtomicTask::chargeBattery);
    lala = enum_DecomposableTask::lowBattery;
    
    monitor->addDecomposableTaskList(lala, teste);
}

/*
void addAtomicTask2(BlackBoard* monitor, MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskSequence.clear();
    std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    monitor->getPosition(currentPosition);
    //std::queue<s_pose> goal = vMissionDecomposable.goal; // When working with the UDPReceiverSim this is necessary
    for (auto n : vMissionDecomposable.atomicTaskEnumerator){
        switch(n){
            case enum_AtomicTask::null :
                break;
            case enum_AtomicTask::goTo :
                vAtomicTaskitem = std::make_shared<GoToSim>(monitor, currentPosition,vMissionDecomposable.goal);
                currentPosition = vMissionDecomposable.goal;
                break;
            case enum_AtomicTask::turnOn :
                vAtomicTaskitem = std::make_shared<TurnOnSim>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = std::make_shared<ChargeBatterySim>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::takePicture :
                vAtomicTaskitem = std::make_shared<TakePictureSim>(monitor, currentPosition,currentPosition);
                break;
            default:
                break;
        }
        if (vAtomicTaskitem != nullptr)
        {
            vMissionDecomposable.atomicTaskSequence.push_back(std::move(vAtomicTaskitem));
            //delete vAtomicTaskitem;
        } else {
            std::cout << "Not found\n";
        }
    }
} */

bool addAtomicTask(BlackBoard* monitor, MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskSequence.clear();
    std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    monitor->getPosition(currentPosition);
    //std::queue<s_pose> goal = vMissionDecomposable.goal; // When working with the UDPReceiverSim this is necessary
    
    char* temp = vMissionDecomposable.attributesBuffer;
    int vAttribuites = 0;
    int partialSize = 0;
    int totalSize = ((int*) temp)[0];
    partialSize += 4;
    temp += 4;
    
    for (auto n : vMissionDecomposable.atomicTaskEnumerator){
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
                vAtomicTaskitem = std::make_shared<GoToSim>(monitor, currentPosition, goal);
                currentPosition = goal;
                partialSize += vSize;
                temp += vSize;
                break;
            }

            case enum_AtomicTask::turnOn :
                vAtomicTaskitem = std::make_shared<TurnOnSim>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = std::make_shared<ChargeBatterySim>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::takePicture :
                vAtomicTaskitem = std::make_shared<TakePictureSim>(monitor, currentPosition,currentPosition);
                break;
            default:
                break;
        }
        if (vAtomicTaskitem != nullptr)
        {
            vMissionDecomposable.atomicTaskSequence.push_back(std::move(vAtomicTaskitem));
            //delete vAtomicTaskitem;
        } else {
            std::cout << "Not found\n";
        }
    }
    return (totalSize == partialSize && vAttribuites == vMissionDecomposable.numberOfAttributes) ? true : false;
}
