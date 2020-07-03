//
//  MissionManager.cpp
//  MRSMac
//
//  Created by Afonso Braga on 08/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "MissionManager.hpp"

MissionManager::MissionManager(BlackBoard* monitor) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    // In the future it won't be here
    std::vector<enum_AtomicTask> teste;
    teste.push_back(enum_AtomicTask::turnOn);
    teste.push_back(enum_AtomicTask::chargeBattery);
    enum_DecomposableTask lala = enum_DecomposableTask::checkPosition;
    
    this->monitor->addDecomposableTaskList(lala, teste);
}

MissionManager::~MissionManager()
{
    
}

void MissionManager::run()
{
    if(this->isRunning == true){
        vMissionMessage = new s_MissionMessage;
        
        this->monitor->getMissionMessage(*vMissionMessage);
        if (vMissionMessage != nullptr && this->isRunning == true)
        {
            switch(vMissionMessage->operation)
            {
                case enum_MissionOperation::null:
                    break;
                    
                case enum_MissionOperation::createMission:
                {
                    strcpy(this->missionOwnerList[vMissionMessage->missionCode].missionCode, vMissionMessage->missionCode);
                    this->monitor->getRobotsIP(*this->missionOwnerList[vMissionMessage->missionCode].senderAddress);
                    this->missionOwnerList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
                    this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
                    this->missionOwnerList[vMissionMessage->missionCode].t5 =  new std::thread(&MissionManager::timer, this, std::ref(this->missionOwnerList[vMissionMessage->missionCode].missionCode));
                    break;
                }
                
                case enum_MissionOperation::addMission:
                {
                    bool status = this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vAtomicTaskVector);
                    if(status == true)
                    {
                        std::cout << "[bidder] Received a mission." <<std::endl;
                        vMission = new MissionExecution;
                        strcpy(vMission->missionCode,vMissionMessage->missionCode);
                        strcpy(vMission->senderAddress,vMissionMessage->senderAddress);
                        vMission->enum_execution = enum_MissionExecution::waitingAuction;
                        //vMission->status = TaskStatus::waiting;
                        addAtomicTask();
                        calculateMissionCost(*vMission);
                        this->MissionList.insert_or_assign(vMission->missionCode, *vMission);
                        delete vMission;
                        sendMissionCost(this->MissionList[vMissionMessage->missionCode]);
                    }
                    break;
                }
                
                case enum_MissionOperation::addAndRequestCost:
                {
                    bool status = this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vAtomicTaskVector);
                    if(status == true){
                        std::cout << "[bidder] Received a mission." <<std::endl;
                        vMission = new MissionExecution;
                        strcpy(vMission->missionCode,vMissionMessage->missionCode);
                        strcpy(vMission->senderAddress,vMissionMessage->senderAddress);
                        vMission->enum_execution = enum_MissionExecution::waitingAuction;
                        //vMission->status = TaskStatus::waiting;
                        addAtomicTask();
                        calculateMissionCost(*vMission);
                        this->MissionList.insert_or_assign(vMission->missionCode, *vMission);
                        delete vMission;
                        break;
                    }
                }
                    
                case enum_MissionOperation::Bid:
                {
                    std::cout << "[owner] Received a bid!" <<std::endl;
                    Bid bid;
                    bid.price = vMissionMessage->Cost;
                    strcpy(bid.bidderIP, vMissionMessage->senderAddress);
                    
                    this->missionOwnerList[vMissionMessage->missionCode].vectorBids.push_back(bid);
                    break;
                }
                case enum_MissionOperation::removeMission:
                    // asasdasd
                    break;
                    
                case enum_MissionOperation::winningBid:
                {
                    std::cout << "[bidder] I win!"<<std::endl;
                    if (this->monitor->isRobotAvailable() == true)
                    {
                        std::cout << "[bidder] I'm available to execute this  mission!"<<std::endl;
                        // Still need to block the robot for not adding any other mission to be executed.
                        auto mission = this->MissionList.find(vMissionMessage->missionCode);
                        this->monitor->addMissionToExecute(mission->second);
                        
                        s_MissionMessage missionMessage;
                        
                        strcpy(missionMessage.missionCode, vMissionMessage->missionCode);
                        this->monitor->getRobotsIP(*missionMessage.senderAddress);
                        missionMessage.operation = enum_MissionOperation::acceptMission;
                        
                        sendUDPMessage(missionMessage, *vMissionMessage->senderAddress);
                    }
                    else
                        std::cout << "[bidder] I'm unavailable to execute this mission." << std::endl;
                    break;
                }
                
                case enum_MissionOperation::acceptMission:
                {
                    this->missionOwnerList[vMissionMessage->missionCode].missionAccepted = true;
                    this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
                    break;
                }

        
                case enum_MissionOperation::startMission:
                {
                    std::cout << "[bidder] Adding mission to execute!"<<std::endl;
                    //auto mission = this->MissionList.find(vMissionMessage->missionCode);
                    //this->monitor->addMissionToExecute(mission->second);
                    this->monitor->startMissionExecution();
                    this->MissionList.clear();
                    break;
                }
                    
            }
        }
        delete vMissionMessage;
    }
}


void MissionManager::addAtomicTask()
{
    
    AtomicTask* vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    s_pose goalPosition;
    
    for (auto n : vAtomicTaskVector){
        switch(n){
            case enum_AtomicTask::null :
                break;
            case enum_AtomicTask::goTo :
                vAtomicTaskitem = new GoTo(currentPosition,goalPosition);
                currentPosition = goalPosition;
                break;
            case enum_AtomicTask::turnOn :
                vAtomicTaskitem = new TurnOn(currentPosition,goalPosition);
                break;
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = new ChargeBattery(currentPosition,goalPosition);
                break;
        }
        if (vAtomicTaskitem != nullptr)
        {
            this->vMission->atomicTaskList.push_back(vAtomicTaskitem);
            //delete vAtomicTaskitem;
        } else {
            std::cout << "Not found\n";
        }
    }
}

void MissionManager::calculateMissionCost(MissionExecution& mission){
    mission.missionCost = 0;
    for (auto n: mission.atomicTaskList){
        mission.missionCost += n->getCost();
    }
}

void MissionManager::sendMissionCost(MissionExecution& mission)
{
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, mission.missionCode);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    missionMessage.operation = enum_MissionOperation::Bid;
    missionMessage.Cost = mission.missionCost;
    
    sendUDPMessage(missionMessage, *mission.senderAddress);
}

void MissionManager::sendUDPMessage(s_MissionMessage &vMissionMessage, char &address)
{
    s_UDPMessage message;
    
    strcpy(message.address, &address);
    Operation operation = Operation::missionMessage;
    *((Operation*)message.buffer) = operation;
    *((int*)(message.buffer + 4)) = sizeof(vMissionMessage);
    memmove(message.buffer+8,(const unsigned char*)&vMissionMessage,sizeof(vMissionMessage));
    message.messageSize = sizeof(message.buffer);
    this->monitor->addUDPMessage(message);
    //std::cout << "pronto"<< std::endl;
}


void MissionManager::timer(char *missionID)
{
    
    this->missionOwnerList[missionID].cv = new std::condition_variable;
    this->missionOwnerList[missionID].cv_m = new std::mutex;
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    
    while(this->missionOwnerList[missionID].enum_request != enum_MissionRequest::missionComplete)
    {
        switch(this->missionOwnerList[missionID].enum_request)
        {
            case enum_MissionRequest::null:
                break;
                
            case enum_MissionRequest::waitingBids:
            {
                auto now = std::chrono::system_clock::now();
                std::cout << "[owner] Waiting for Bids" << std::endl;
                
                s_MissionMessage missionMessage;
                
                strcpy(missionMessage.missionCode, missionID);
                this->monitor->getRobotsIP(*missionMessage.senderAddress);
                missionMessage.operation = enum_MissionOperation::addMission;
                missionMessage.taskToBeDecomposed = this->missionOwnerList[missionID].mission;
                
                sendUDPMessage(missionMessage, *this->broadcastIP);
                this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].biddingTime));
                
                if(this->missionOwnerList[missionID].vectorBids.empty())
                {
                    std::cout << "[owner] No Bids received. Trying again." << std::endl;
                } else
                {
                    this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
                }
                break;
            }
                
            case enum_MissionRequest::notifyingWinner:
            {
                if(this->missionOwnerList[missionID].vectorBids.empty() || this->missionOwnerList[missionID].vectorBids.size() == 0)
                {
                    std::cout << "[owner] No one accepted the mission, offering again..." << std::endl;
                    this->missionOwnerList[missionID].enum_request = enum_MissionRequest::waitingBids;
                } else
                {
                    int winner = 0;
                    for(int i=0; i< this->missionOwnerList[missionID].vectorBids.size(); i++)
                    {
                        if (this->missionOwnerList[missionID].vectorBids.at(winner).price > this->missionOwnerList[missionID].vectorBids.at(i).price)
                            winner = i;
                    }
                    strcpy(this->missionOwnerList[missionID].winnerAddress, this->missionOwnerList[missionID].vectorBids.at(winner).bidderIP);
                    this->missionOwnerList[missionID].vectorBids.erase(this->missionOwnerList[missionID].vectorBids.begin() + winner);
                    
                    std::cout << "[owner] Notifying Winner" << std::endl;
                    
                    s_MissionMessage missionMessage;
                    
                    strcpy(missionMessage.missionCode, missionID);
                    this->monitor->getRobotsIP(*missionMessage.senderAddress);
                    missionMessage.operation = enum_MissionOperation::winningBid;
                    
                    sendUDPMessage(missionMessage, *this->missionOwnerList[missionID].winnerAddress);
                    
                    auto now = std::chrono::system_clock::now();
                    this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].communicationTime));
                    this->missionOwnerList[missionID].enum_request = enum_MissionRequest::executingMission;
                }
                break;
            }
            
            case enum_MissionRequest::executingMission:
            {
                std::cout << "[owner] Sending the execution command!" << std::endl;
                
                auto now = std::chrono::system_clock::now();
                this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].communicationTime));
                
                if(this->missionOwnerList[missionID].missionAccepted == false)
                {
                    std::cout << "[owner] Not accepted. Notifying the next bid." << std::endl;
                    this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
                }else
                {
                    s_MissionMessage missionMessage;
                    
                    strcpy(missionMessage.missionCode, missionID);
                    this->monitor->getRobotsIP(*missionMessage.senderAddress);
                    missionMessage.operation = enum_MissionOperation::startMission;
                    
                    sendUDPMessage(missionMessage, *this->missionOwnerList[missionID].winnerAddress);
                    this->missionOwnerList[missionID].enum_request = enum_MissionRequest::missionComplete;
                }
                break;
            }
                
            case enum_MissionRequest::missionComplete:
                std::cout << "[owner] Mission Complete!" << std::endl;
                break;
        }
    }
    delete this->missionOwnerList[missionID].cv;
    delete this->missionOwnerList[missionID].cv_m;
}
