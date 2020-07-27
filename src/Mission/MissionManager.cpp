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
    teste.push_back(enum_AtomicTask::goTo);
    enum_DecomposableTask lala = enum_DecomposableTask::checkPosition;
    this->monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::goTo);
    teste.push_back(enum_AtomicTask::takePicture);
    lala = enum_DecomposableTask::takePicture;
    this->monitor->addDecomposableTaskList(lala, teste);
    
    teste.clear();
    teste.push_back(enum_AtomicTask::chargeBattery);
    lala = enum_DecomposableTask::lowBattery;
    
    this->monitor->addDecomposableTaskList(lala, teste);
}

MissionManager::~MissionManager()
{
    
}

void MissionManager::mainThread()
{
    executeMission =  new std::thread(&MissionManager::startMissionToExecute, this);
    while(this->isRunning)
    {
        this->run();
    }
}

void MissionManager::run()
{
    if(this->isRunning == true)
    {
        s_MissionMessage* vMissionMessage = new s_MissionMessage;
        this->monitor->getMissionMessage(*vMissionMessage);
        
        if (vMissionMessage != nullptr && this->isRunning == true)
        {
            switch(vMissionMessage->operation)
            {
                case enum_MissionOperation::null:
                    break;
                    
                    // All related to Auctioneer
                    
                case enum_MissionOperation::createMission:
                    createMission(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::Bid:
                    addBidReceived(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::acceptMission:
                    missionAccepted(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::abortMission:
                    missionAborted(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::missionComplete:
                    missionComplete(vMissionMessage);
                    break;
                    
                    // All Related to Bidders
                    
                case enum_MissionOperation::addMission:
                    addMissionReceived(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::addAndRequestCost:
                    addMissionCalculateCost(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::winningBid:
                    winningBid(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::startMission:
                    startCommand(vMissionMessage);
                    break;
                    
                case enum_MissionOperation::emergency:
                    emergencyCall(vMissionMessage);
                    break;
            }
        }
        delete vMissionMessage;
    }
}

// This is a timer for each new MissionRequest
void MissionManager::missionRequestController(char* missionID)
{
    this->missionOwnerList[missionID].cv = new std::condition_variable;
    this->missionOwnerList[missionID].cv_m = new std::mutex;
    
    while(this->missionOwnerList[missionID].endMission == false)
    {
        switch(this->missionOwnerList[missionID].enum_request)
        {
            case enum_MissionRequest::null:
                break;
                
            case enum_MissionRequest::waitingBids:
                waitingForBids(missionID);
                break;
                
            case enum_MissionRequest::notifyingWinner:
                notifyingWinner(missionID);
                break;
            
            case enum_MissionRequest::executingMission:
                notifyingToExecute(missionID);
                break;
                
            case enum_MissionRequest::missionComplete:
                std::cout << "[owner] Mission Complete!" << std::endl;
                this->missionOwnerList[missionID].endMission = true;
                break;
        }
    }
    delete this->missionOwnerList[missionID].cv;
    delete this->missionOwnerList[missionID].cv_m;
}

void MissionManager::startMissionToExecute()
{
    //STILL NEEDS TO HOLD THE MISSION IN CASE OF NEDDED
    std::unique_lock<std::mutex> lk(mutex_mission);
    while (this->isRunning)
    {
        if (getEmergencyStatus() == false)
        {
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
                    this->missionToExecute.run();
                    break;
                    
                case enum_MissionExecution::missionComplete:
                    notifyingMissionComplete();
                    this->monitor->unlockRobot();
                    this->conditional_executeMission.wait(lk);
                    break;
            }
        } else
        {
            // EMERGENCY NOW
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
                    cleanEmergecy();
                    this->monitor->unlockRobot();
                    break;
            }
        }
    }
}

//****************************************************
//*        Functions Related to The Bidder           *
//****************************************************

void MissionManager::addMissionReceived(s_MissionMessage* vMissionMessage)
{
    std::cout << "[bidder] Received a mission." <<std::endl;
    MissionExecution* vMission = new MissionExecution;
    
    bool status = this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector); // Checking if it is decomposable
    if(status == true)
    {
        std::cout << "[bidder] Mission received is decomposable." <<std::endl;

        // Adding the new Mission into database, including all AtomicTasks and calculating total cost
        //Protect if the robot is executing a mission it doesnt need to bid
        strcpy(vMission->missionCode,vMissionMessage->missionCode);
        strcpy(vMission->senderAddress,vMissionMessage->senderAddress);
        vMission->enum_execution = enum_MissionExecution::waitingAuction;
        vMission->mission = vMissionMessage->taskToBeDecomposed;
        vMission->goal = vMissionMessage->goal;
        
        addAtomicTask(*vMission);
        calculateMissionCost(*vMission);
        //CHECK IF THERE IS ENOUGH BATTERY OR IF THE PATH IS FEASABLE
        this->MissionList.insert_or_assign(vMission->missionCode, *vMission);
        
        // Send back the proposal
        sendMissionCost(this->MissionList[vMissionMessage->missionCode]);
    }
    delete vMission;
}

void MissionManager::addMissionCalculateCost(s_MissionMessage* vMissionMessage)
{
    std::cout << "[bidder] Received a mission." <<std::endl;
    MissionExecution* vMission = new MissionExecution;
    
    bool status = this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector); // Checking if it is decomposable
    if(status == true)
    {
        std::cout << "[bidder] Mission received is decomposable." <<std::endl;
        
        // Adding the new Mission into database, including all AtomicTasks and calculating total cost

        strcpy(vMission->missionCode,vMissionMessage->missionCode);
        strcpy(vMission->senderAddress,vMissionMessage->senderAddress);
        vMission->enum_execution = enum_MissionExecution::waitingAuction;
        vMission->mission = vMissionMessage->taskToBeDecomposed;
        vMission->goal = vMissionMessage->goal;
        
        addAtomicTask(*vMission);
        calculateMissionCost(*vMission);
        this->MissionList.insert_or_assign(vMission->missionCode, *vMission);
        // This one doesn't send back
    }
    delete vMission;
}


void MissionManager::winningBid(s_MissionMessage* vMissionMessage)
{
    // If this mission was already waitinf for the auctions result and won, execute:
    
    if(this->MissionList[vMissionMessage->missionCode].enum_execution == enum_MissionExecution::waitingAuction)
    {
        std::cout << "[bidder] I win!"<<std::endl;
        
        // First check if the robot is still available to execute the mission, if so, lock it up!
        if (this->monitor->lockRobot() == true)
        {
            std::cout << "[bidder] I'm available to execute this mission!"<<std::endl;
            
            // Add this mission into the execution Module.
            addMissionToExecute(this->MissionList[vMissionMessage->missionCode]);
            s_MissionMessage missionMessage;
            
            // Send to the auctioneer that the mission was accepted
            strcpy(missionMessage.missionCode, vMissionMessage->missionCode);
            this->monitor->getRobotsIP(*missionMessage.senderAddress);
            missionMessage.operation = enum_MissionOperation::acceptMission;
                
            sendUDPMessage(missionMessage, *vMissionMessage->senderAddress);
        }
        else
            std::cout << "[bidder] I'm unavailable to execute this mission." << std::endl;
    }
}


void MissionManager::addMissionToExecute(MissionExecution& vMissionExecute)
{
    // The pointers from atomictasklist are not created they just point to the variables
    // Solved using shared_ptr
    std::unique_lock<std::mutex> lk(mutex_mission);
    missionToExecute = vMissionExecute;
    missionToExecute.enum_execution = enum_MissionExecution::waitingStart;
    lk.unlock();
}

void MissionManager::startCommand(s_MissionMessage *vMissionMessage)
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    if(strcmp(this->missionToExecute.missionCode, vMissionMessage->missionCode) == 0 && this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart)
    {
        std::cout << "[bidder] Adding mission to execute!"<<std::endl;
        missionToExecute.enum_execution = enum_MissionExecution::executing;
        lk.unlock();
        this->conditional_executeMission.notify_one();
        this->MissionList.clear();
    }
}

void MissionManager::emergencyCall(s_MissionMessage *vMissionMessage)
{
    //Configurar Missão, checar se está acontecendo algo, travar robo, cancelar e terceirizar missão, executar missão de emergência.
    std::cout << "[bidder] EMERGENCY ALLERT!!!!" <<std::endl;
    MissionExecution* vMission = new MissionExecution;
    
    if(this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector))
    {
        // Checar se robô está executando alguma missão, se sim, redirecionar. Se não, não é necessario.
        if(this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart || this->missionToExecute.enum_execution == enum_MissionExecution::executing)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::null;
            std::cout << "[bidder] Redirecting Misssion!" <<std::endl;
            redirectMission(this->missionToExecute);
        }
        
        
        strcpy(vMission->missionCode,vMissionMessage->missionCode);
        strcpy(vMission->senderAddress,vMissionMessage->senderAddress);
        vMission->mission = vMissionMessage->taskToBeDecomposed;
        
        addAtomicTask(*vMission);
        calculateMissionCost(*vMission);
        
        this->monitor->lockRobot();
        addMissionEmergency(*vMission);
    }
    delete vMission;
}

void MissionManager::notifyingMissionComplete()
{
    s_MissionMessage missionMessage;
    
    // Send to the auctioneer that the mission was accepted
    strcpy(missionMessage.missionCode, this->missionToExecute.missionCode);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    missionMessage.operation = enum_MissionOperation::missionComplete;
        
    sendUDPMessage(missionMessage, *this->missionToExecute.senderAddress);
}

//****************************************************
//*       Functions Related to The auctioneer         *
//****************************************************

void MissionManager::createMission(s_MissionMessage* vMissionMessage)
{
    strcpy(this->missionOwnerList[vMissionMessage->missionCode].missionCode, vMissionMessage->missionCode);
    this->monitor->getRobotsIP(*this->missionOwnerList[vMissionMessage->missionCode].senderAddress);
    this->missionOwnerList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
    this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
    this->missionOwnerList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
    this->missionOwnerList[vMissionMessage->missionCode].t5 =  new std::thread(&MissionManager::missionRequestController, this, std::ref(this->missionOwnerList[vMissionMessage->missionCode].missionCode));
}


void MissionManager::waitingForBids(char* missionID)
{
    // Lock mutex and start timer
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    auto now = std::chrono::system_clock::now();
    
    std::cout << "[owner] Waiting for Bids" << std::endl;
    
    // Send the mission to all robots
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, missionID);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    missionMessage.operation = enum_MissionOperation::addMission;
    missionMessage.taskToBeDecomposed = this->missionOwnerList[missionID].mission;
    missionMessage.goal = this->missionOwnerList[missionID].goal;
    sendUDPMessage(missionMessage, *this->broadcastIP);
    
    // Lock and wait intil the time is passed by
    this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].biddingTime));
    
    if(this->missionOwnerList[missionID].vectorBids.empty())
    {
        // If there is no available robots, start again.
        std::cout << "[owner] No Bids received. Trying again." << std::endl;
    } else
    {
        // If there is at least a bid, proceed.
        this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
    }
}

void MissionManager::addBidReceived(s_MissionMessage* vMissionMessage)
{
    // Check if the mission is still receiving bids.
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (this->missionOwnerList[vMissionMessage->missionCode].enum_request == enum_MissionRequest::waitingBids)
    {
        std::cout << "[owner] Received a bid!" <<std::endl;
        Bid bid;
        bid.price = vMissionMessage->Cost;
        strcpy(bid.bidderIP, vMissionMessage->senderAddress);
        
        this->missionOwnerList[vMissionMessage->missionCode].vectorBids.push_back(bid);
    }
    lk.unlock();
}

void MissionManager::notifyingWinner(char* missionID)
{
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
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
        
        if(this->missionOwnerList[missionID].missionAccepted == false)
        {
            std::cout << "[owner] Not accepted. Notifying the next bid." << std::endl;
            this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
        }else
            this->missionOwnerList[missionID].enum_request = enum_MissionRequest::executingMission;
    }
    lk.unlock();
}

void MissionManager::notifyingToExecute(char* missionID)
{
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    
    std::cout << "[owner] Sending the execution command!" << std::endl;
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, missionID);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    missionMessage.operation = enum_MissionOperation::startMission;
    
    sendUDPMessage(missionMessage, *this->missionOwnerList[missionID].winnerAddress);
    this->missionOwnerList[missionID].cv->wait(lk);
}

void MissionManager::missionAccepted(s_MissionMessage* vMissionMessage)
{
    // it is necessary to check if the winner still is the one selected to perform the mission.
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0)
    {
        this->missionOwnerList[vMissionMessage->missionCode].missionAccepted = true;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
    }
}

void MissionManager::missionAborted(s_MissionMessage *vMissionMessage)
{
    // Check if the received message comes from the winning bid, clear bidding vector, restart auctionProcess.
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0)
    {
        std::cout << "[owner] Received an Abort Mission Command!" <<std::endl;
        this->missionOwnerList[vMissionMessage->missionCode].vectorBids.clear();
        this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
        lk.unlock();
    }
}

void MissionManager::missionComplete(s_MissionMessage *vMissionMessage)
{
    // Check if the received message comes from the winning bid and finish the mission
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0)
    {
        this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::missionComplete;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
        lk.unlock();
    }
}

//****************************************************
//*                    THE END                       *
//****************************************************





void MissionManager::addAtomicTask(MissionExecution& vMissionDecomposable)
{
    vMissionDecomposable.atomicTaskList.clear();
    std::shared_ptr<AtomicTask> vAtomicTaskitem = nullptr;
    s_pose currentPosition;
    this->monitor->getPosition(currentPosition);
    
    for (auto n : vMissionDecomposable.vAtomicTaskVector){
        switch(n){
            case enum_AtomicTask::null :
                break;
            case enum_AtomicTask::goTo :
                vAtomicTaskitem = std::make_shared<GoTo>(this->monitor, currentPosition,vMissionDecomposable.goal);
                currentPosition = vMissionDecomposable.goal;
                break;
            case enum_AtomicTask::turnOn :
                vAtomicTaskitem = std::make_shared<TurnOn>(this->monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::chargeBattery :
                vAtomicTaskitem = std::make_shared<ChargeBattery>(this->monitor, currentPosition,currentPosition);
                break;
            case enum_AtomicTask::takePicture :
            vAtomicTaskitem = std::make_shared<TakePicture>(this->monitor, currentPosition,currentPosition);
            break;
        }
        if (vAtomicTaskitem != nullptr)
        {
            vMissionDecomposable.atomicTaskList.push_back(vAtomicTaskitem);
            //delete vAtomicTaskitem;
        } else {
            std::cout << "Not found\n";
        }
    }
}


void MissionManager::calculateMissionCost(MissionExecution& mission)
{
    mission.missionCost = 0;
    for (auto n: mission.atomicTaskList)
    {
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
}


// Emergency related

bool MissionManager::getEmergencyStatus()
{
    std::unique_lock<std::mutex> lk(mutex_emergency);
    bool status = this->emergency;
    lk.unlock();
    return status;
}

bool MissionManager::setEmergency()
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

bool MissionManager::cleanEmergecy()
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

void MissionManager::addMissionEmergency(MissionExecution& vMissionExecute)
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

void MissionManager::redirectMission(MissionExecution& vMissionExecute)
{
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, vMissionExecute.missionCode);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    missionMessage.operation = enum_MissionOperation::abortMission;
    
    sendUDPMessage(missionMessage, *vMissionExecute.senderAddress);
}
