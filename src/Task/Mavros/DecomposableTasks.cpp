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
    lala = enum_DecomposableTask::takePicture;
    monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::chargeBattery);
    lala = enum_DecomposableTask::lowBattery;
    
    monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::arm);
    teste.push_back(enum_AtomicTask::takeOff);
    teste.push_back(enum_AtomicTask::goTo);
    teste.push_back(enum_AtomicTask::land);
    lala = enum_DecomposableTask::flightTest;
    monitor->addDecomposableTaskList(lala, teste);
}

void addAtomicTask2(BlackBoard* monitor, MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskSequence.clear();
        std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
        s_pose currentPosition;
        monitor->getPosition(currentPosition);
        
        for (auto n : vMissionDecomposable.atomicTaskEnumerator){
            switch(n){
                case enum_AtomicTask::null :
                    break;
                case enum_AtomicTask::goTo :
                    vAtomicTaskitem = std::make_shared<GoTo>(monitor, currentPosition,vMissionDecomposable.goal);
                    currentPosition = vMissionDecomposable.goal;
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
                    
                    
                case enum_AtomicTask::arm :
                {
                    vAtomicTaskitem = std::make_shared<ArmMavROS>(monitor, currentPosition,currentPosition);
                }
                    
                    break;
                    
                case enum_AtomicTask::takeOff :
                {
                    s_pose vPose;
                    vPose = currentPosition;
                    vPose.z = vMissionDecomposable.goal.z;
                    vAtomicTaskitem = std::make_shared<TakeOffMavROS>(monitor, currentPosition,vPose);
                    currentPosition = vPose;
                }
                    
                    break;
                    
                case enum_AtomicTask::land :
                {
                    vAtomicTaskitem = std::make_shared<LandMavROS>(monitor, currentPosition,currentPosition);
                }
                    
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
}
