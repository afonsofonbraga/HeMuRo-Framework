//
//  P3DXRobot.cpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "P3DXRobot.hpp"

P3DXRobot::P3DXRobot(Blackboard* monitor, ros::NodeHandle& vNode, bool decentralized): Agent(monitor)
{
    strcpy(mode,"Robot");
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::ugv);
    decomposableTaskList();
    
    broadcast = new UDPBroadcast(monitor);
    sender = new UDPSender(monitor);
    
    auction = new Auction(monitor, this);
    taskModule = new TaskModule(monitor, this);
    
    batteryManager = new BatteryManager(monitor,mode);
    
    if (this->decentralized == true)
    {
        receiver = new UDPReceiver(monitor);
        logger = new Logger(monitor);
        
        logger->Module::start();
        receiver->start();
    }
    
    rosModule = new ROSModuleP3DX(monitor, vNode);
    sender->start();
    broadcast->start();
    rosModule->start();
    batteryManager->start();
    
    auction->start();
    taskModule->start();
}

P3DXRobot::~P3DXRobot()
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
    delete this->rosModule;
}

void P3DXRobot::decomposableTaskList()
{
    std::vector<enum_AtomicTask> atomicTaskVector;
    enum_DecomposableTask dTask = enum_DecomposableTask::null;

    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::chargeBattery);
    dTask = enum_DecomposableTask::lowBattery;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);

    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::pickUpSample);
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::dropOffSample);
    dTask = enum_DecomposableTask::deliverSmallSample;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);

    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::pickUpSample);
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::dropOffSample);
    dTask = enum_DecomposableTask::deliverBigSample;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);

    atomicTaskVector.clear();
    atomicTaskVector.push_back(enum_AtomicTask::moveBaseGoal);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    dTask = enum_DecomposableTask::inspectPlace;
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
}

bool P3DXRobot::addAtomicTask(MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskSequence.clear();
    std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    monitor->getPosition(currentPosition);
    
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
 
            case enum_AtomicTask::moveBaseGoal:
            {
                int vSize = ((int*) temp)[0];
                partialSize += 4;
                temp += 4;
                vAttribuites++;
                s_pose goal = ((s_pose*) temp)[0];
                vAtomicTaskitem = std::make_shared<MoveBaseGoal>(monitor, currentPosition, goal);
                
                float battery_discharge = 50000; // Motors discharge [mAh]
                float robots_max_speed = 0.2; // Robot's maximum speed [m/s]
                float battery_capacity = 7000; // Battery's capacity [mAh]
                int factor = 10; // The robot does not go straight to the goal LACOXAMBRE

                vAtomicTaskitem->setCostFactor(factor * (battery_discharge /(robots_max_speed*3600))/battery_capacity);
                vAtomicTaskitem->setTimeFactor(robots_max_speed);

                currentPosition = goal;
                partialSize += vSize;
                temp += vSize;
                break;
            }
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = std::make_shared<ChargeBatteryROS>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::takePicture :
                vAtomicTaskitem = std::make_shared<TakePictureSim>(monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::pickUpSample:
                vAtomicTaskitem = std::make_shared<PickUpSim>(monitor, currentPosition, currentPosition);
                break;
            case enum_AtomicTask::dropOffSample:
                vAtomicTaskitem = std::make_shared<DropOffSim>(monitor,currentPosition,currentPosition);
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
