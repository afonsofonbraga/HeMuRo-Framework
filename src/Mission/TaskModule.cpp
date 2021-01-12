//
//  TaskModule.cpp
//  MRSMac
//
//  Created by Afonso Braga on 30/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TaskModule.hpp"

TaskModule::TaskModule(BlackBoard* monitor) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->monitor->getRobotsName(*this->robotName);
    decomposableTaskList(monitor);
    
}

TaskModule::~TaskModule()
{
    
}

void TaskModule::mainThread()
{
    executeMission =  new std::thread(&TaskModule::startMissionToExecute, this);
    while(this->isRunning)
    {
        this->run();
    }
}

void TaskModule::run()
{
    if(this->isRunning == true)
    {
        auto vTaskMessage = std::make_unique<s_TaskMessage>();
        this->monitor->getTaskMessage(*vTaskMessage);
        
        if (vTaskMessage != nullptr && this->isRunning == true)
        {
            switch(vTaskMessage->operation)
            {
                case enum_TaskMessage::null:
                    break;
                
                case enum_TaskMessage::requestCost:
                    
                    break;
                    
                case enum_TaskMessage::addTask:
                    addTaskReceived(std::move(vTaskMessage));
                    break;
                    
                case enum_TaskMessage::executeTask:
                    startCommand(std::move(vTaskMessage));
                    break;
                    
                case enum_TaskMessage::addAndExecute:
                    //missionAccepted(std::move(vMissionMessage));
                    break;
                    
                case enum_TaskMessage::redirect:
                    //missionAborted(std::move(vMissionMessage));
                    break;
                case enum_TaskMessage::addEmergency:
                    emergencyCall(std::move(vTaskMessage));
                    break;
                default:
                    break;
            }
        }
    }
}

void TaskModule::startMissionToExecute()
{
    //STILL NEEDS TO HOLD THE MISSION IN CASE OF NEDDED
    while (this->isRunning)
    {
        if (getEmergencyStatus() == false)
        {
            std::unique_lock<std::mutex> lk(mutex_mission);
            switch(this->missionToExecute.enum_execution)
            {
                case enum_MissionExecution::null:
                    this->conditional_executeMission.wait(lk);
                    break;
                    
                case enum_MissionExecution::waitingAuction:
                    this->conditional_executeMission.wait(lk);
                    break;
                    
                case enum_MissionExecution::waitingStart:
                    this->conditional_executeMission.wait(lk);
                    break;
                    
                case enum_MissionExecution::executing:
                {
                    auto currentTime = std::chrono::system_clock::now();
                    if ( currentTime - this->missionToExecute.startTime <= std::chrono::seconds(this->missionToExecute.executionTime))
                        this->missionToExecute.run();
                    else
                    {
                        this->monitor->print("TIMEOUT!!! Redirecting Misssion " + std::string(this->missionToExecute.missionCode) + "!");
                        this->missionToExecute.enum_execution = enum_MissionExecution::null;
                        this->missionToExecute.stop();
                        redirectMission(this->missionToExecute);
                        this->monitor->clearCostToExecute();
                        this->monitor->unlockRobot();
                    }
                    break;
                }
                    
                case enum_MissionExecution::missionComplete:
                {
                    this->monitor->print("Mission Complete!");
                    
                    s_MissionMessage missionMessage;
                    strcpy(missionMessage.missionCode, missionToExecute.missionCode);
                    strcpy(missionMessage.senderAddress, missionToExecute.senderAddress);
                    strcpy(missionMessage.senderName, missionToExecute.senderName);
                    missionMessage.operation = enum_MissionOperation::notifyMissionComplete;
                    this->monitor->addMissionMessage(missionMessage);
                    
                    this->monitor->unlockRobot();
                    this->conditional_executeMission.wait(lk);
                    break;
                }
                    
                default:
                    break;
            }
        } else
        {
            // EMERGENCY NOW
            std::unique_lock<std::mutex> lk(mutex_emergency);
            switch(this->missionEmergency.enum_execution)
            {
                case enum_MissionExecution::null:
                    this->missionEmergency.enum_execution = enum_MissionExecution::executing;
                    break;
                    
                case enum_MissionExecution::waitingAuction:
                    this->missionEmergency.enum_execution = enum_MissionExecution::executing;
                    break;
                    
                case enum_MissionExecution::waitingStart:
                    this->missionEmergency.enum_execution = enum_MissionExecution::executing;
                    break;
                    
                case enum_MissionExecution::executing:
                    this->missionEmergency.run();
                    break;
                    
                case enum_MissionExecution::missionComplete:
                    this->monitor->print("Emergency finished!");
                    lk.unlock();
                    cleanEmergecy();
                    this->monitor->unlockRobot();
                    //this->conditional_executeMission.wait(lk);
                    break;
                
                default:
                    break;
            }
        }
    }
}

void TaskModule::addTaskReceived(std::unique_ptr<s_TaskMessage> vTaskMessage)
{
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vTaskMessage->taskToBeDecomposed, vMission->atomicTaskEnumerator) == true && vTaskMessage->robotCat == this->monitor->getRobotsCategory() && this->monitor->lockRobot(enum_RobotStatus::executing) == true)
    {
        missionToExecute.clear();
        std::unique_lock<std::mutex> lk(mutex_mission);
        strcpy(this->missionToExecute.missionCode,vTaskMessage->missionCode);
        strcpy(this->missionToExecute.senderAddress,vTaskMessage->senderAddress);
        strcpy(this->missionToExecute.senderName,vTaskMessage->senderName);
        this->missionToExecute.enum_execution = enum_MissionExecution::waitingStart;
        this->missionToExecute.mission = vTaskMessage->taskToBeDecomposed;
        
        this->missionToExecute.atomicTaskEnumerator = std::move(vMission->atomicTaskEnumerator);
        this->missionToExecute.goal = vTaskMessage->goal;
        this->missionToExecute.numberOfAttributes = vTaskMessage->numberOfAttributes;
        int totalSize = ((int*) vTaskMessage->attributesBuffer)[0];
        memcpy(this->missionToExecute.attributesBuffer,&vTaskMessage->attributesBuffer, totalSize);
        
        this->missionToExecute.robotCategory = vTaskMessage->robotCat;
        this->missionToExecute.executionTime = vTaskMessage->executionTime;
        
        
        bool status = addAtomicTask(monitor, this->missionToExecute);
        calculateMissionCost(this->missionToExecute);
        this->monitor->setCostToExecute(this->missionToExecute.missionCost);
        
        //this->monitor->print("Mission " + std::string(vTaskMessage->missionCode) + " costs: " + std::to_string(this->missionToExecute.missionCost) + " BL: " + std::to_string(this->monitor->getBatteryLevel()));
        
        if(this->missionToExecute.missionCost <= this->monitor->getBatteryLevel() && status == true)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::waitingStart;
            s_MissionMessage missionMessage;
            strcpy(missionMessage.missionCode, missionToExecute.missionCode);
            strcpy(missionMessage.senderName,missionToExecute.senderName);
            strcpy(missionMessage.senderAddress,missionToExecute.senderAddress);
            missionMessage.operation = enum_MissionOperation::lockingComplete;
            this->monitor->addMissionMessage(missionMessage);
        }else
        {
            // If the robot doesn't have enough battery to execute the mission it will be freed to bid again.
            this->monitor->unlockRobot();
        }

        // Aqui temos que testar ainda o que o STATUS VAI FAZER
        lk.unlock();
    }

}

void TaskModule::calculateMissionCost(MissionExecution& mission)
{
    mission.missionCost = 0;
    for (auto n: mission.atomicTaskSequence)
    {
        mission.missionCost += n->getCost();
    }
}

void TaskModule::startCommand(std::unique_ptr<s_TaskMessage> vTaskMessage)
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    if(strcmp(this->missionToExecute.missionCode, vTaskMessage->missionCode) == 0 && this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart)
    {
        this->monitor->print("Adding " + std::string(this->missionToExecute.missionCode) + " to execute!");
        missionToExecute.enum_execution = enum_MissionExecution::executing;
        lk.unlock();
        this->missionToExecute.startTime = std::chrono::system_clock::now();
        this->conditional_executeMission.notify_one();
    }
}

// Emergency related

void TaskModule::emergencyCall(std::unique_ptr<s_TaskMessage> vTaskMessage)
{
    //Configurar Missão, checar se está acontecendo algo, travar robo, cancelar e terceirizar missão, executar missão de emergência.
    this->monitor->print("EMERGENCY ALLERT!!!!");
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vTaskMessage->taskToBeDecomposed, vMission->atomicTaskEnumerator))
    {
        // Checar se robô está executando alguma missão, se sim, redirecionar. Se não, não é necessario.
        std::unique_lock<std::mutex> lk(mutex_mission);
        if(this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart || this->missionToExecute.enum_execution == enum_MissionExecution::executing)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::null;
            redirectMission(this->missionToExecute);
        }
        lk.unlock();
        
        strcpy(vMission->missionCode,vTaskMessage->missionCode);
        strcpy(vMission->senderAddress,vTaskMessage->senderAddress);
        strcpy(vMission->senderName,vTaskMessage->senderName);
        vMission->mission = vTaskMessage->taskToBeDecomposed;
        
        vMission->goal = vTaskMessage->goal;
        vMission->numberOfAttributes = vTaskMessage->numberOfAttributes;
        int totalSize = ((int*) vTaskMessage->attributesBuffer)[0];
        memcpy(vMission->attributesBuffer,&vTaskMessage->attributesBuffer, totalSize);

        bool status = addAtomicTask(monitor, *vMission);
        if (status == true)
        {
            calculateMissionCost(*vMission);
            
            this->monitor->lockRobot(enum_RobotStatus::emergency);
            addMissionEmergency(*vMission);
        }
        
    }
}

bool TaskModule::getEmergencyStatus()
{
    std::unique_lock<std::mutex> lk(mutex_emergency);
    bool status = this->emergency;
    lk.unlock();
    return status;
}

bool TaskModule::setEmergency()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_emergency);
    if (this->emergency == false)
    {
        this->emergency = true;
        status = true;
        this->conditional_executeMission.notify_one();
    }
    lk.unlock();
    return status;
}

bool TaskModule::cleanEmergecy()
{
    bool status = false;
    std::unique_lock<std::mutex> lk(mutex_emergency);
    if (this->emergency == true)
    {
        this->emergency = false;
        status = true;
    }
    lk.unlock();
    return status;
}

void TaskModule::addMissionEmergency(MissionExecution& vMissionExecute)
{
    if(getEmergencyStatus() == false)
    {
        std::unique_lock<std::mutex> lk(mutex_emergency);
        missionEmergency = vMissionExecute;
        missionEmergency.enum_execution = enum_MissionExecution::waitingStart;
        lk.unlock();
        setEmergency();
    }
}

void TaskModule::redirectMission(MissionExecution& vMissionExecute)
{
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, vMissionExecute.missionCode);
    strcpy(missionMessage.senderName, vMissionExecute.senderName);
    strcpy(missionMessage.senderAddress, vMissionExecute.senderAddress);
    missionMessage.operation = enum_MissionOperation::redirectRequest;
    
    this->monitor->addMissionMessage(missionMessage);
}

