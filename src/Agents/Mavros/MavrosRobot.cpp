//
//  MavrosRobot.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "MavrosRobot.hpp"

MavrosRobot::MavrosRobot(Blackboard* monitor, ros::NodeHandle& vNode, bool decentralized): Agent(monitor)
{
    strcpy(mode,"Robot");
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::uav);
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
    
    rosModule = new ROSModuleMavros(monitor, vNode);
    sender->start();
    broadcast->start();
    rosModule->start();
    batteryManager->start();
    
    auction->start();
    taskModule->start();
}

MavrosRobot::~MavrosRobot()
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

void MavrosRobot::decomposableTaskList()
{
    std::vector<enum_AtomicTask> atomicTaskVector;
    enum_DecomposableTask dTask = enum_DecomposableTask::null;

    atomicTaskVector.clear();
    dTask = enum_DecomposableTask::checkPosition;
    atomicTaskVector.push_back(enum_AtomicTask::arm);
    atomicTaskVector.push_back(enum_AtomicTask::takeOff);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::goToBasis);
    atomicTaskVector.push_back(enum_AtomicTask::land);
    //atomicTaskVector.push_back(enum_AtomicTask::disarm);
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    dTask = enum_DecomposableTask::inspectArea;
    atomicTaskVector.push_back(enum_AtomicTask::arm);
    atomicTaskVector.push_back(enum_AtomicTask::takeOff);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::goToBasis);
    atomicTaskVector.push_back(enum_AtomicTask::land);
    //atomicTaskVector.push_back(enum_AtomicTask::disarm);
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    dTask = enum_DecomposableTask::takePicture;
    atomicTaskVector.push_back(enum_AtomicTask::arm);
    atomicTaskVector.push_back(enum_AtomicTask::takeOff);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::takePicture);
    atomicTaskVector.push_back(enum_AtomicTask::goToBasis);
    atomicTaskVector.push_back(enum_AtomicTask::land);
    //atomicTaskVector.push_back(enum_AtomicTask::disarm);
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    dTask = enum_DecomposableTask::lowBattery;
    atomicTaskVector.push_back(enum_AtomicTask::arm);
    atomicTaskVector.push_back(enum_AtomicTask::takeOff);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::land);
    atomicTaskVector.push_back(enum_AtomicTask::disarm);
    atomicTaskVector.push_back(enum_AtomicTask::chargeBattery);
    atomicTaskVector.push_back(enum_AtomicTask::arm);
    atomicTaskVector.push_back(enum_AtomicTask::takeOff);
    atomicTaskVector.push_back(enum_AtomicTask::goToBasis);
    //atomicTaskVector.push_back(enum_AtomicTask::land);
    atomicTaskVector.push_back(enum_AtomicTask::disarm);
    
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
    
    atomicTaskVector.clear();
    dTask = enum_DecomposableTask::emergencyLanding;
    atomicTaskVector.push_back(enum_AtomicTask::land);
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);

    atomicTaskVector.clear();
    dTask = enum_DecomposableTask::flightTest;
    atomicTaskVector.push_back(enum_AtomicTask::arm);
    atomicTaskVector.push_back(enum_AtomicTask::takeOff);
    atomicTaskVector.push_back(enum_AtomicTask::goTo);
    atomicTaskVector.push_back(enum_AtomicTask::land);
    monitor->addDecomposableTaskList(dTask, atomicTaskVector);
}

bool MavrosRobot::addAtomicTask(MissionExecution& vMissionDecomposable)
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
                case enum_AtomicTask::goTo :
                {
                    int vSize = ((int*) temp)[0];
                    partialSize += 4;
                    temp += 4;
                    vAttribuites++;
                    s_pose goal = ((s_pose*) temp)[0];
                    vAtomicTaskitem = std::make_shared<NavigateMavROS>(monitor, currentPosition, goal);
                    //std::cout << "go to x: " <<goal.x << " y " << goal.y<< " z " << goal.z << std::endl;
                    currentPosition = goal;
                    partialSize += vSize;
                    temp += vSize;
                    break;
                }
                case enum_AtomicTask::goToBasis :
                {
                    s_pose p;
                    this->monitor->getBasisPosition(p);
                    vAtomicTaskitem = std::make_shared<NavigateMavROS>(monitor,currentPosition,p);
                    currentPosition = p;
                    break;
                }
                    
                case enum_AtomicTask::turnOn :
                {
                    vAtomicTaskitem = std::make_shared<TurnOnSim>(monitor, currentPosition,currentPosition);
                    break;
                }
                    
                case enum_AtomicTask::chargeBattery :
                {
                    vAtomicTaskitem = std::make_shared<ChargeBatteryROS>(monitor, currentPosition,currentPosition);
                    break;
                }
                    
                case enum_AtomicTask::takePicture :
                {
                    vAtomicTaskitem = std::make_shared<TakePictureSim>(monitor, currentPosition,currentPosition);
                    break;
                }
                    
                case enum_AtomicTask::arm :
                {
                    vAtomicTaskitem = std::make_shared<ArmMavROS>(monitor, currentPosition,currentPosition);
                    break;
                }

                case enum_AtomicTask::disarm :
                {
                    vAtomicTaskitem = std::make_shared<DisarmMavROS>(monitor, currentPosition,currentPosition);
                    break;
                }
    
                case enum_AtomicTask::takeOff :
                {
                    temp += 4;
                    s_pose goal = ((s_pose*) temp)[0];
                    temp -= 4;
                    s_pose vPose;
                    vPose = currentPosition;
                    vPose.z = goal.z;
                    vAtomicTaskitem = std::make_shared<TakeOffMavROS>(monitor, currentPosition,vPose);
                    //std::cout << "Take off x: " <<vPose.x << " y " << vPose.y<< " z " << vPose.z << std::endl;
                    currentPosition = vPose;
                    break;
                }
                    
                case enum_AtomicTask::land :
                {
                    vAtomicTaskitem = std::make_shared<LandMavROS>(monitor, currentPosition,currentPosition);
                    break;
                }
                    
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
