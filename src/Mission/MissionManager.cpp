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
    this->monitor->getRobotsName(*this->robotName);
    decomposableTaskList(monitor);
    
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
        //s_MissionMessage* vMissionMessage = new s_MissionMessage;
        auto vMissionMessage = std::make_unique<s_MissionMessage>();
        this->monitor->getMissionMessage(*vMissionMessage);
        
        
        if (vMissionMessage != nullptr && this->isRunning == true)
        {
            switch(vMissionMessage->operation)
            {
                case enum_MissionOperation::null:
                    break;
                    
                    // All related to Auctioneer
                    
                case enum_MissionOperation::createMission:
                    createMission(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::Bid:
                    addBidReceived(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::acceptMission:
                    missionAccepted(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::abortMission:
                    missionAborted(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::missionComplete:
                    missionComplete(std::move(vMissionMessage));
                    break;
                    
                    // All Related to Bidders
                    
                case enum_MissionOperation::addMission:
                    addMissionReceived(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::addAndRequestCost:
                    addMissionCalculateCost(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::winningBid:
                    winningBid(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::startMission:
                    startCommand(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::emergency:
                    emergencyCall(std::move(vMissionMessage));
                    break;
            }
        }
    }
}

// This is a timer for each new MissionRequest
void MissionManager::missionRequestController(char* missionID)
{
    this->missionOwnerList[missionID].cv = std::make_unique<std::condition_variable>() ;
    this->missionOwnerList[missionID].cv_m = std::make_unique<std::mutex>();
    
    
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
                std::cout << "[" << this->robotName << "] Mission " << missionID << " Complete!" << std::endl;
                this->missionOwnerList[missionID].endMission = true;
                break;
        }
    }
    //std::cout << "Deleting Mutexes" <<std::endl;
    this->missionOwnerList[missionID].cv.release();
    this->missionOwnerList[missionID].cv_m.release();
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
                {
                    auto currentTime = std::chrono::system_clock::now();
                    if ( currentTime - this->missionToExecute.startTime <= std::chrono::seconds(this->missionToExecute.executionTime))
                        this->missionToExecute.run();
                    else
                    {
                        std::cout << "[" << this->robotName << "] TIMEOUT!!! Redirecting Misssion "<< this->missionToExecute.missionCode <<"!"<<std::endl;
                        this->missionToExecute.enum_execution = enum_MissionExecution::null;
                        redirectMission(this->missionToExecute);
                        this->monitor->unlockRobot();
                    }
                    break;
                }
                    
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
                    this->conditional_executeMission.wait(lk);
                    break;
            }
        }
    }
}

//****************************************************
//*        Functions Related to The Bidder           *
//****************************************************

void MissionManager::addMissionReceived(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    std::cout << "[" << this->robotName << "] Received "<< vMissionMessage->missionCode <<"!" <<std::endl;
    //MissionExecution* vMission = new MissionExecution;
    auto vMission = std::make_unique<MissionExecution>();
    
    //bool status = this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector); // Checking if it is decomposable
    
    if(this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector) == true && vMissionMessage->robotCat == this->monitor->getRobotsCategory())
    {
        std::cout << "[" << this->robotName << "] "<< vMissionMessage->missionCode <<" is decomposable." <<std::endl;
        
        strcpy(this->MissionList[vMissionMessage->missionCode].missionCode,vMissionMessage->missionCode);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderAddress,vMissionMessage->senderAddress);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderName,vMissionMessage->senderName);
        this->MissionList[vMissionMessage->missionCode].enum_execution = enum_MissionExecution::waitingAuction;
        this->MissionList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
        this->MissionList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
        this->MissionList[vMissionMessage->missionCode].vAtomicTaskVector = std::move(vMission->vAtomicTaskVector);
        this->MissionList[vMissionMessage->missionCode].robotCategory = vMissionMessage->robotCat;
        this->MissionList[vMissionMessage->missionCode].executionTime = vMissionMessage->executionTime;
        
        //addAtomicTask(this->MissionList[vMissionMessage->missionCode]);
        addAtomicTask2(monitor, this->MissionList[vMissionMessage->missionCode]);
        calculateMissionCost(this->MissionList[vMissionMessage->missionCode]);
        //CHECK IF THERE IS ENOUGH BATTERY OR IF THE PATH IS FEASABLE
        //this->MissionList.insert_or_assign(vMission->missionCode, *vMission);
        std::cout << "Mission "<< vMissionMessage->missionCode << " costs: " << this->MissionList[vMissionMessage->missionCode].missionCost << " BL: " << this->monitor->getBatteryLevel() << std::endl;
        if(this->MissionList[vMissionMessage->missionCode].missionCost <= this->monitor->getBatteryLevel())
            sendMissionCost(this->MissionList[vMissionMessage->missionCode]); // Send back the proposal
    }
    //std::cout << "Deleting vMission" <<std::endl;
    //delete vMission;
}

void MissionManager::addMissionCalculateCost(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    std::cout << "[" << this->robotName << "] Received "<< vMissionMessage->missionCode <<"!" <<std::endl;
    //MissionExecution* vMission = new MissionExecution;
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector) == true && vMissionMessage->robotCat == this->monitor->getRobotsCategory())
    {
        std::cout << "[" << this->robotName << "] "<< vMissionMessage->missionCode <<" is decomposable." <<std::endl;
        
        // Adding the new Mission into database, including all AtomicTasks and calculating total cost
        
        strcpy(this->MissionList[vMissionMessage->missionCode].missionCode,vMissionMessage->missionCode);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderAddress,vMissionMessage->senderAddress);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderName,vMissionMessage->senderName);
        this->MissionList[vMissionMessage->missionCode].enum_execution = enum_MissionExecution::waitingAuction;
        this->MissionList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
        this->MissionList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
        this->MissionList[vMissionMessage->missionCode].vAtomicTaskVector = std::move(vMission->vAtomicTaskVector);
        this->MissionList[vMissionMessage->missionCode].robotCategory = vMissionMessage->robotCat;
        this->MissionList[vMissionMessage->missionCode].executionTime = vMissionMessage->executionTime;
        
        //addAtomicTask(this->MissionList[vMissionMessage->missionCode]);
        addAtomicTask2(monitor, this->MissionList[vMissionMessage->missionCode]);
        calculateMissionCost(this->MissionList[vMissionMessage->missionCode]);
        // This one doesn't send back
    }
    //std::cout << "Deleting vMission" <<std::endl;
    //delete vMission;
}


void MissionManager::winningBid(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // If this mission was already waitinf for the auctions result and won, execute:
    
    if(this->MissionList[vMissionMessage->missionCode].enum_execution == enum_MissionExecution::waitingAuction)
    {
        std::cout << "[" << this->robotName << "] I win!"<<std::endl;
        
        // First check if the robot is still available to execute the mission, if so, lock it up!
        if (this->monitor->lockRobot() == true)
        {
            std::cout << "[" << this->robotName << "] I'm available to execute "<< vMissionMessage->missionCode <<"!"<<std::endl;
            
            // Add this mission into the execution Module.
            addMissionToExecute(this->MissionList[vMissionMessage->missionCode]);
            s_MissionMessage missionMessage;
            
            // Send to the auctioneer that the mission was accepted
            strcpy(missionMessage.missionCode, vMissionMessage->missionCode);
            
            this->monitor->getRobotsIP(*missionMessage.senderAddress);
            this->monitor->getRobotsName(*missionMessage.senderName);
            
            missionMessage.operation = enum_MissionOperation::acceptMission;
            
            sendUDPMessage(missionMessage, *vMissionMessage->senderAddress, *vMissionMessage->senderName);
        }
        else
            std::cout << "[" << this->robotName << "] I'm unavailable to execute " << vMissionMessage->missionCode <<"!"<<std::endl;
    }
}


void MissionManager::addMissionToExecute(MissionExecution& vMissionExecute)
{
    // The pointers from atomictasklist are not created they just point to the variables
    // Solved using shared_ptr
    std::unique_lock<std::mutex> lk(mutex_mission);
    //missionToExecute = vMissionExecute;
    strcpy(missionToExecute.missionCode,vMissionExecute.missionCode);
    strcpy(missionToExecute.senderAddress,vMissionExecute.senderAddress);
    strcpy(missionToExecute.senderName,vMissionExecute.senderName);
    missionToExecute.mission = vMissionExecute.mission;
    missionToExecute.goal = vMissionExecute.goal;
    missionToExecute.vAtomicTaskVector = std::move(vMissionExecute.vAtomicTaskVector);
    missionToExecute.atomicTaskList = std::move(vMissionExecute.atomicTaskList);
    missionToExecute.atomicTaskIndex = vMissionExecute.atomicTaskIndex;
    missionToExecute.missionCost = vMissionExecute.missionCost;
    
    missionToExecute.robotCategory = vMissionExecute.robotCategory;
    missionToExecute.executionTime = vMissionExecute.executionTime;
    
    missionToExecute.enum_execution = enum_MissionExecution::waitingStart;
    lk.unlock();
}

void MissionManager::startCommand(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    std::unique_lock<std::mutex> lk(mutex_mission);
    if(strcmp(this->missionToExecute.missionCode, vMissionMessage->missionCode) == 0 && this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart)
    {
        std::cout << "[" << this->robotName << "] Adding "<< this->missionToExecute.missionCode << " to execute!"<<std::endl;
        missionToExecute.enum_execution = enum_MissionExecution::executing;
        lk.unlock();
        this->missionToExecute.startTime = std::chrono::system_clock::now();
        this->conditional_executeMission.notify_one();
        this->MissionList.clear();
    }
}

void MissionManager::emergencyCall(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    //Configurar Missão, checar se está acontecendo algo, travar robo, cancelar e terceirizar missão, executar missão de emergência.
    std::cout << "[" << this->robotName << "] EMERGENCY ALLERT!!!!" <<std::endl;
    //MissionExecution* vMission = new MissionExecution;
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->vAtomicTaskVector))
    {
        // Checar se robô está executando alguma missão, se sim, redirecionar. Se não, não é necessario.
        if(this->missionToExecute.enum_execution == enum_MissionExecution::waitingStart || this->missionToExecute.enum_execution == enum_MissionExecution::executing)
        {
            this->missionToExecute.enum_execution = enum_MissionExecution::null;
            std::cout << "[" << this->robotName << "] Redirecting " << this->missionToExecute.missionCode << "!" <<std::endl;
            redirectMission(this->missionToExecute);
        }
        
        
        strcpy(vMission->missionCode,vMissionMessage->missionCode);
        strcpy(vMission->senderAddress,vMissionMessage->senderAddress);
        strcpy(vMission->senderName,vMissionMessage->senderName);
        vMission->mission = vMissionMessage->taskToBeDecomposed;
        vMission->goal = vMissionMessage->goal;
        //addAtomicTask(*vMission);
        
        addAtomicTask2(monitor, *vMission);
        calculateMissionCost(*vMission);
        
        this->monitor->lockRobot();
        addMissionEmergency(*vMission);
    }
    //std::cout << "Deleting vMission Emergency" <<std::endl;
    //delete vMission;
}

void MissionManager::notifyingMissionComplete()
{
    s_MissionMessage missionMessage;
    
    // Send to the auctioneer that the mission was accepted
    strcpy(missionMessage.missionCode, this->missionToExecute.missionCode);
    
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    
    missionMessage.operation = enum_MissionOperation::missionComplete;
    
    sendUDPMessage(missionMessage, *this->missionToExecute.senderAddress, *this->missionToExecute.senderName);
}

//****************************************************
//*       Functions Related to The auctioneer         *
//****************************************************

void MissionManager::createMission(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    strcpy(this->missionOwnerList[vMissionMessage->missionCode].missionCode, vMissionMessage->missionCode);
    this->monitor->getRobotsIP(*this->missionOwnerList[vMissionMessage->missionCode].senderAddress);
    
    this->monitor->getRobotsName(*this->missionOwnerList[vMissionMessage->missionCode].senderName);
    
    this->missionOwnerList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
    this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
    this->missionOwnerList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
    this->missionOwnerList[vMissionMessage->missionCode].robotCategory = vMissionMessage->robotCat;
    this->missionOwnerList[vMissionMessage->missionCode].executionTime = vMissionMessage->executionTime;
    
    this->missionOwnerList[vMissionMessage->missionCode].t5 =  new std::thread(&MissionManager::missionRequestController, this, std::ref(this->missionOwnerList[vMissionMessage->missionCode].missionCode));
}


void MissionManager::waitingForBids(char* missionID)
{
    // Lock mutex and start timer
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    std::chrono::time_point now = std::chrono::system_clock::now();
    
    std::cout << "[" << this->robotName << "] Waiting for Bids" << std::endl;
    
    // Send the mission to all robots
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, missionID);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    missionMessage.operation = enum_MissionOperation::addMission;
    missionMessage.taskToBeDecomposed = this->missionOwnerList[missionID].mission;
    missionMessage.goal = this->missionOwnerList[missionID].goal;
    missionMessage.robotCat = this->missionOwnerList[missionID].robotCategory;
    missionMessage.executionTime = this->missionOwnerList[missionID].executionTime;
    char broadcast[10] = "Broadcast";
    sendUDPMessage(missionMessage, *this->broadcastIP, *broadcast);
    
    // Lock and wait intil the time is passed by
    this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].biddingTime));

    if(this->missionOwnerList[missionID].vectorBids.empty())
    {
        // If there is no available robots, start again.
        std::cout << "[" << this->robotName << "] No Bids received. Trying again." << std::endl;
    } else
    {
        // If there is at least a bid, proceed.
        this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
    }
}

void MissionManager::addBidReceived(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // Check if the mission is still receiving bids.
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (this->missionOwnerList[vMissionMessage->missionCode].enum_request == enum_MissionRequest::waitingBids)
    {
        std::cout << "[" << this->robotName << "] " << vMissionMessage->missionCode << " received a bid from "<< vMissionMessage->senderName << "!" <<std::endl;
        Bid bid;
        bid.price = vMissionMessage->Cost;
        strcpy(bid.bidderIP, vMissionMessage->senderAddress);
        strcpy(bid.bidderName, vMissionMessage->senderName);
        
        this->missionOwnerList[vMissionMessage->missionCode].vectorBids.push_back(bid);
    }
    lk.unlock();
}

void MissionManager::notifyingWinner(char* missionID)
{
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    if(this->missionOwnerList[missionID].vectorBids.empty() || this->missionOwnerList[missionID].vectorBids.size() == 0)
    {
        std::cout << "[" << this->robotName << "] No one accepted "<< missionID << ". Offering again..." << std::endl;
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
        strcpy(this->missionOwnerList[missionID].winnerName, this->missionOwnerList[missionID].vectorBids.at(winner).bidderName);
        
        this->missionOwnerList[missionID].vectorBids.erase(this->missionOwnerList[missionID].vectorBids.begin() + winner);
        
        std::cout << "[" << this->robotName << "] "<< this->missionOwnerList[missionID].winnerName << " won " << missionID <<"! Notifying!" << std::endl;
        
        s_MissionMessage missionMessage;
        
        strcpy(missionMessage.missionCode, missionID);
        
        this->monitor->getRobotsIP(*missionMessage.senderAddress);
        this->monitor->getRobotsName(*missionMessage.senderName);
        
        missionMessage.operation = enum_MissionOperation::winningBid;
        
        sendUDPMessage(missionMessage, *this->missionOwnerList[missionID].winnerAddress, *this->missionOwnerList[missionID].winnerName);
        
        auto now = std::chrono::system_clock::now();
        this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].communicationTime));
        
        if(this->missionOwnerList[missionID].missionAccepted == false)
        {
            std::cout << "[" << this->robotName << "] Not accepted. Notifying the next bid." << std::endl;
            this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
        }else
            this->missionOwnerList[missionID].enum_request = enum_MissionRequest::executingMission;
    }
    lk.unlock();
}

void MissionManager::notifyingToExecute(char* missionID)
{
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    
    std::cout << "[" << this->robotName << "] Sending the execution command!" << std::endl;
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, missionID);
    
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    
    missionMessage.operation = enum_MissionOperation::startMission;
    
    sendUDPMessage(missionMessage, *this->missionOwnerList[missionID].winnerAddress, *this->missionOwnerList[missionID].winnerName);
    this->missionOwnerList[missionID].cv->wait(lk);
}

void MissionManager::missionAccepted(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // it is necessary to check if the winner still is the one selected to perform the mission.
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0 && strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerName, vMissionMessage->senderName)==0)
    {
        this->missionOwnerList[vMissionMessage->missionCode].missionAccepted = true;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
    }
}

void MissionManager::missionAborted(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // Check if the received message comes from the winning bid, clear bidding vector, restart auctionProcess.
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0)
    {
        std::cout << "[" << this->robotName << "] Received an Abort Mission Command!" <<std::endl;
        this->missionOwnerList[vMissionMessage->missionCode].vectorBids.clear();
        this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
        lk.unlock();
    }
}

void MissionManager::missionComplete(std::unique_ptr<s_MissionMessage> vMissionMessage)
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
    this->monitor->getRobotsName(*missionMessage.senderName);
    missionMessage.operation = enum_MissionOperation::Bid;
    missionMessage.Cost = mission.missionCost;
    
    sendUDPMessage(missionMessage, *mission.senderAddress, *mission.senderName);
}

void MissionManager::sendUDPMessage(s_MissionMessage &vMissionMessage, char &targetAddress, char& targetName)
{
    s_UDPMessage message;
    
    strcpy(message.address, &targetAddress);
    strcpy(message.name, &targetName);
    memcpy(message.buffer, &targetName,10);
    Operation operation = Operation::missionMessage;
    *((Operation*)(message.buffer + 10)) = operation;
    *((int*)(message.buffer + 14)) = sizeof(vMissionMessage);
    memmove(message.buffer+18,(const unsigned char*)&vMissionMessage,sizeof(vMissionMessage));
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
    
    sendUDPMessage(missionMessage, *vMissionExecute.senderAddress, *vMissionExecute.senderName);
}

