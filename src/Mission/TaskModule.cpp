//
//  TaskModule.cpp
//  MRSMac
//
//  Created by Afonso Braga on 30/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TaskModule.hpp"

TaskModule::TaskModule(Blackboard* monitor) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->monitor->getRobotsName(*this->robotName);
    
}
TaskModule::TaskModule(Blackboard* monitor, Agent* a) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->monitor->getRobotsName(*this->robotName);
    this->agent = a;
}

TaskModule::~TaskModule()
{
    
}

void TaskModule::mainThread()
{
    executeMission =  new std::thread(&TaskModule::startMissionToExecute, this);
    while(this->isRunning == true)
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
                    addMissionToExecute(std::move(vTaskMessage));
                    break;
                    
                case enum_TaskMessage::redirect:
                    //missionAborted(std::move(vMissionMessage));
                    break;
                case enum_TaskMessage::addEmergency:
                    emergencyCall(std::move(vTaskMessage));
                    break;
                case enum_TaskMessage::failure:
                    failure(std::move(vTaskMessage));
                    break;
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
        if (this->monitor->getRobotStatus() == enum_RobotStatus::failure)
            {
                this->stop();
                this->missionToExecute.stop();
                this->missionEmergency.stop();
                break;
            }
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
                    if ( currentTime - this->missionToExecute.startTime <= this->missionToExecute.relativeDeadline)
                        this->missionToExecute.run();
                    else
                    {
                        this->monitor->print("TIMEOUT!!! Redirecting Misssion " + std::string(this->missionToExecute.missionCode) + "!");
                        this->missionToExecute.enum_execution = enum_MissionExecution::null;
                        this->missionToExecute.stop();
                        
                        s_MissionStatus missionStatus;
                        strcpy(missionStatus.missionCode, this->missionToExecute.missionCode);
                        strcpy(missionStatus.missionOwner, this->missionToExecute.senderName);
                        strcpy(missionStatus.missionExecutioner, robotName);
                        missionStatus.status = enum_MissionStatus::timeout;
                        this->monitor->printMissionStatus(missionStatus);
                        
                        redirectMission(this->missionToExecute);
                        this->monitor->clearCostToExecute();
                        this->monitor->unlockRobot();
                    }
                    break;
                }
                    
                case enum_MissionExecution::missionComplete:
                {
                    auto currentTime = std::chrono::system_clock::now();
                    
                    s_MissionStatus missionStatus;
                    strcpy(missionStatus.missionCode, this->missionToExecute.missionCode);
                    strcpy(missionStatus.missionOwner, this->missionToExecute.senderName);
                    missionStatus.executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - this->missionToExecute.startTime);
                    
                    this->monitor->printMissionStatus(missionStatus);
                    
                    
                    
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
            lk.unlock();
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
                    this->missionToExecute.enum_execution = enum_MissionExecution::null;
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
        //this->missionToExecute.goal = vTaskMessage->goal;
        this->missionToExecute.numberOfAttributes = vTaskMessage->numberOfAttributes;
        int totalSize = ((int*) vTaskMessage->attributesBuffer)[0];
        memcpy(this->missionToExecute.attributesBuffer,&vTaskMessage->attributesBuffer, totalSize);
        
        this->missionToExecute.robotCategory = vTaskMessage->robotCat;
        this->missionToExecute.relativeDeadline = vTaskMessage->relativeDeadline;
        
        
        bool status = agent->addAtomicTask(this->missionToExecute);
        
        calculateMissionExecutionTime(this->missionToExecute);
        calculateMissionCost(this->missionToExecute);

        this->monitor->setCostToExecute(this->missionToExecute.missionCost);
                
        s_MissionStatus missionStatus;
        strcpy(missionStatus.missionCode, this->missionToExecute.missionCode);
        strcpy(missionStatus.missionOwner, this->missionToExecute.senderName);
        missionStatus.estimatedExecutionTime = this->missionToExecute.timeToExecute;
        
        this->monitor->printMissionStatus(missionStatus);
        
        
        if(this->missionToExecute.missionCost <= this->monitor->getBatteryLevel() && status == true && this->missionToExecute.timeToExecute <= this->missionToExecute.relativeDeadline)
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

void TaskModule::calculateMissionExecutionTime(MissionExecution& mission)
{
    mission.timeToExecute = std::chrono::milliseconds(0);
    for (auto n:mission.atomicTaskSequence)
    {
        mission.timeToExecute += n->getTime();
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

void TaskModule::addMissionToExecute(std::unique_ptr<s_TaskMessage> vTaskMessage)
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
        //this->missionToExecute.goal = vTaskMessage->goal;
        this->missionToExecute.numberOfAttributes = vTaskMessage->numberOfAttributes;
        int totalSize = ((int*) vTaskMessage->attributesBuffer)[0];
        memcpy(this->missionToExecute.attributesBuffer,&vTaskMessage->attributesBuffer, totalSize);
        
        this->missionToExecute.robotCategory = vTaskMessage->robotCat;
        this->missionToExecute.relativeDeadline = vTaskMessage->relativeDeadline;
        
        
        bool status = agent->addAtomicTask( this->missionToExecute);
        
        calculateMissionExecutionTime(this->missionToExecute);
        calculateMissionCost(this->missionToExecute);
        
        this->monitor->setCostToExecute(this->missionToExecute.missionCost);
        
        if(this->missionToExecute.missionCost <= this->monitor->getBatteryLevel() && status == true && this->missionToExecute.timeToExecute <= this->missionToExecute.relativeDeadline)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::executing;
            s_MissionMessage missionMessage;
            strcpy(missionMessage.missionCode, missionToExecute.missionCode);
            strcpy(missionMessage.senderName,missionToExecute.senderName);
            strcpy(missionMessage.senderAddress,missionToExecute.senderAddress);
            missionMessage.operation = enum_MissionOperation::lockingComplete;
            this->monitor->addMissionMessage(missionMessage);
            lk.unlock();
            this->monitor->print("Adding " + std::string(this->missionToExecute.missionCode) + " to execute!");
            
            this->missionToExecute.startTime = std::chrono::system_clock::now();
            this->conditional_executeMission.notify_one();
        }else
        {
            // If the robot doesn't have enough battery to execute the mission it will be freed to bid again.
            this->monitor->unlockRobot();
            lk.unlock();
        }

        // Aqui temos que testar ainda o que o STATUS VAI FAZER
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
        
        
        strcpy(vMission->missionCode,vTaskMessage->missionCode);
        strcpy(vMission->senderAddress,vTaskMessage->senderAddress);
        strcpy(vMission->senderName,vTaskMessage->senderName);
        vMission->mission = vTaskMessage->taskToBeDecomposed;
        
        vMission->numberOfAttributes = vTaskMessage->numberOfAttributes;
        int totalSize = ((int*) vTaskMessage->attributesBuffer)[0];
        memcpy(vMission->attributesBuffer,&vTaskMessage->attributesBuffer, totalSize);

        bool status = agent->addAtomicTask( *vMission);
        if (status == true)
        {
            calculateMissionCost(*vMission);
            
            this->monitor->lockRobot(enum_RobotStatus::emergency);
            addMissionEmergency(*vMission);
        }
        
        std::unique_lock<std::mutex> lk(mutex_mission);
        if(this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart || this->missionToExecute.enum_execution == enum_MissionExecution::executing)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::null;
            this->missionToExecute.stop();
            
            s_MissionStatus missionStatus;
            strcpy(missionStatus.missionCode, this->missionToExecute.missionCode);
            strcpy(missionStatus.missionOwner, this->missionToExecute.senderName);
            strcpy(missionStatus.missionExecutioner, robotName);
            missionStatus.status = enum_MissionStatus::lowBattery;
            this->monitor->printMissionStatus(missionStatus);
            
            redirectMission(this->missionToExecute);
        }
        lk.unlock();
    }
}

void TaskModule::failure(std::unique_ptr<s_TaskMessage> vTaskMessage)
{
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vTaskMessage->taskToBeDecomposed, vMission->atomicTaskEnumerator))
    {
        this->stop();
        std::unique_lock<std::mutex> lk(mutex_mission);
        if(this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart || this->missionToExecute.enum_execution == enum_MissionExecution::executing)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::null;
            this->missionToExecute.stop();
            
            s_MissionStatus missionStatus;
            strcpy(missionStatus.missionCode, this->missionToExecute.missionCode);
            strcpy(missionStatus.missionOwner, this->missionToExecute.senderName);
            strcpy(missionStatus.missionExecutioner, robotName);
            missionStatus.status = enum_MissionStatus::failure;
            this->monitor->printMissionStatus(missionStatus);
            
            redirectMission(this->missionToExecute);
        }
        lk.unlock();
        
        std::unique_lock<std::mutex> lk2(mutex_emergency);
        if(this->missionEmergency.enum_execution == enum_MissionExecution::executing)
        {
            this->missionEmergency.stop();
            this->missionEmergency.enum_execution = enum_MissionExecution::null;
            
            s_MissionStatus missionStatus;
            strcpy(missionStatus.missionCode, this->missionEmergency.missionCode);
            strcpy(missionStatus.missionOwner, this->missionEmergency.senderName);
            strcpy(missionStatus.missionExecutioner, robotName);
            missionStatus.status = enum_MissionStatus::chargingCancelled;
            this->monitor->printMissionStatus(missionStatus);
        }
        lk2.unlock();
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

