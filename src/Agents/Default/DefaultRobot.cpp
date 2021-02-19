//
//  DefaultRobot.cpp
//  MRSMac
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "DefaultRobot.hpp"

DefaultRobot::DefaultRobot(Blackboard* monitor, bool decentralized): Agent(monitor)
{
    strcpy(mode,"Robot");
    this->decentralized = decentralized;
    this->monitor->setBatteryLevel(32);
    monitor->setRobotCategory(enum_RobotCategory::ugv);
    decomposableTaskList();
    
    broadcast = new UDPBroadcast(monitor);
    sender = new UDPSender(monitor);
    //missionManager = new MissionManager(monitor, this);
    
    auction = new Auction(monitor, this);
    taskModule = new TaskModule(monitor, this);
    
    batteryManager = new BatteryManager(monitor,mode);
    
    //usleep(10000);
    if (this->decentralized == true)
    {
        receiver = new UDPReceiver(monitor);
        logger = new Logger(monitor);
        
        logger->Module::start();
        receiver->start();
    }

    sender->start();
    broadcast->start();
    batteryManager->start();
    //missionManager->start();
    auction->start();
    taskModule->start();
}

DefaultRobot::~DefaultRobot()
{
    delete this->auction;
    delete this->taskModule;
    delete this->broadcast;
    delete this->sender;
    delete this->batteryManager;
    
    if (this->decentralized == true)
    {
        delete this->receiver;
        delete this->logger;
    }
}

bool DefaultRobot::addAtomicTask(MissionExecution& vMissionDecomposable)
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


void DefaultRobot::decomposableTaskList()
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
