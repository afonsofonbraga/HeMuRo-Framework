//
//  Auction.cpp
//  MRSMac
//
//  Created by Afonso Braga on 30/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Auction.hpp"


Auction::Auction(Blackboard* monitor) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->monitor->getRobotsName(*this->robotName);
    
}
Auction::Auction(Blackboard* monitor, Agent* a) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->monitor->getRobotsName(*this->robotName);
    this->agent = a;
    
}

Auction::~Auction()
{
    
}

void Auction::run()
{
    if(this->isRunning == true)
    {
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
                    
                case enum_MissionOperation::lockingComplete:
                    lockingComplete(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::startMission:
                    startCommand(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::redirectRequest:
                    abortMission(std::move(vMissionMessage));
                    break;
                    
                case enum_MissionOperation::notifyMissionComplete:
                    notifyingMissionComplete(std::move(vMissionMessage));
                    break;
                default:
                    break;
            }
        }
    }
}

// This is a timer for each new MissionRequest
void Auction::missionRequestController(char* missionID)
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
                this->monitor->print("Mission " + std::string(missionID) + " Complete!");
                this->missionOwnerList[missionID].endMission = true;
                break;
            default:
                break;
        }
    }
    //std::cout << "Deleting Mutexes" <<std::endl;
    this->missionOwnerList[missionID].cv.release();
    this->missionOwnerList[missionID].cv_m.release();
}


//****************************************************
//*        Functions Related to The Bidder           *
//****************************************************

void Auction::addMissionReceived(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->atomicTaskEnumerator) == true && vMissionMessage->robotCat == this->monitor->getRobotsCategory() && this->monitor->isRobotAvailable()==true)
    {
        this->monitor->print("Received " + std::string(vMissionMessage->missionCode) + "!");
        this->monitor->print(std::string(vMissionMessage->missionCode) + " is decomposable.");
        
        strcpy(this->MissionList[vMissionMessage->missionCode].missionCode,vMissionMessage->missionCode);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderAddress,vMissionMessage->senderAddress);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderName,vMissionMessage->senderName);
        this->MissionList[vMissionMessage->missionCode].enum_execution = enum_MissionExecution::waitingAuction;
        this->MissionList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
        
        //this->MissionList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
        this->MissionList[vMissionMessage->missionCode].numberOfAttributes = vMissionMessage->numberOfAttributes;
        int totalSize = ((int*) vMissionMessage->attributesBuffer)[0];
        memcpy(this->MissionList[vMissionMessage->missionCode].attributesBuffer,&vMissionMessage->attributesBuffer, totalSize);
        
        this->MissionList[vMissionMessage->missionCode].atomicTaskEnumerator = std::move(vMission->atomicTaskEnumerator);
        this->MissionList[vMissionMessage->missionCode].robotCategory = vMissionMessage->robotCat;
        this->MissionList[vMissionMessage->missionCode].relativeDeadline = vMissionMessage->relativeDeadline;
        
        bool status = agent->addAtomicTask(this->MissionList[vMissionMessage->missionCode]);
        calculateMissionCost(this->MissionList[vMissionMessage->missionCode]);
        
        //CHECK IF THERE IS ENOUGH BATTERY OR IF THE PATH IS FEASABLE
        this->monitor->print("Mission " + std::string(vMissionMessage->missionCode) + " costs: " + std::to_string(this->MissionList[vMissionMessage->missionCode].missionCost) + " BL: " + std::to_string(this->monitor->getBatteryLevel()));
        
        if(this->MissionList[vMissionMessage->missionCode].missionCost <= this->monitor->getBatteryLevel() && status == true)
            sendMissionCost(this->MissionList[vMissionMessage->missionCode]); // Send back the proposal
    }
}

void Auction::addMissionCalculateCost(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    this->monitor->print("Received " + std::string(vMissionMessage->missionCode) + "!");
    
    auto vMission = std::make_unique<MissionExecution>();
    
    if(this->monitor->getDecomposableTask(vMissionMessage->taskToBeDecomposed, vMission->atomicTaskEnumerator) == true && vMissionMessage->robotCat == this->monitor->getRobotsCategory())
    {
        
        this->monitor->print(std::string(vMissionMessage->missionCode) + " is decomposable.");
        // Adding the new Mission into database, including all AtomicTasks and calculating total cost
        
        strcpy(this->MissionList[vMissionMessage->missionCode].missionCode,vMissionMessage->missionCode);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderAddress,vMissionMessage->senderAddress);
        strcpy(this->MissionList[vMissionMessage->missionCode].senderName,vMissionMessage->senderName);
        this->MissionList[vMissionMessage->missionCode].enum_execution = enum_MissionExecution::waitingAuction;
        this->MissionList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
        
        //this->MissionList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
        this->MissionList[vMissionMessage->missionCode].numberOfAttributes = vMissionMessage->numberOfAttributes;
        int totalSize = ((int*) vMissionMessage->attributesBuffer)[0];
        memcpy(this->MissionList[vMissionMessage->missionCode].attributesBuffer,&vMissionMessage->attributesBuffer, totalSize);
        
        
        this->MissionList[vMissionMessage->missionCode].atomicTaskEnumerator = std::move(vMission->atomicTaskEnumerator);
        this->MissionList[vMissionMessage->missionCode].robotCategory = vMissionMessage->robotCat;
        this->MissionList[vMissionMessage->missionCode].relativeDeadline = vMissionMessage->relativeDeadline;
        
        bool status = agent->addAtomicTask( this->MissionList[vMissionMessage->missionCode]);
        calculateMissionCost(this->MissionList[vMissionMessage->missionCode]);
        // This one doesn't send back
    }
}


void Auction::winningBid(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // If this mission was already waitinf for the auctions result and won, execute:
    
    if(this->MissionList[vMissionMessage->missionCode].enum_execution == enum_MissionExecution::waitingAuction)
    {
        this->monitor->print("I win!");
        
        // First check if the robot is still available to execute the mission, if so, lock it up!
        if (this->monitor->getRobotStatus() == enum_RobotStatus::available)
        {
            // Add this mission into the execution Module.
            addMissionToExecute(this->MissionList[vMissionMessage->missionCode]);
        }
        else
            this->monitor->print("I'm unavailable to execute " + std::string(vMissionMessage->missionCode) + "!");
    }
}


void Auction::addMissionToExecute(MissionExecution& vMissionExecute)
{
    s_TaskMessage vTaskMessage;
    strcpy(vTaskMessage.missionCode,vMissionExecute.missionCode);
    strcpy(vTaskMessage.senderAddress,vMissionExecute.senderAddress);
    strcpy(vTaskMessage.senderName,vMissionExecute.senderName);
    
    vTaskMessage.taskToBeDecomposed = vMissionExecute.mission;
    
    //vTaskMessage.goal = vMissionExecute.goal;
    vTaskMessage.numberOfAttributes = vMissionExecute.numberOfAttributes;
    strcpy(vTaskMessage.attributesBuffer,vMissionExecute.attributesBuffer);
    int totalSize = ((int*) vMissionExecute.attributesBuffer)[0];
    memcpy(vTaskMessage.attributesBuffer,vMissionExecute.attributesBuffer, totalSize);
    
    vTaskMessage.robotCat = vMissionExecute.robotCategory;
    vTaskMessage.relativeDeadline = vMissionExecute.relativeDeadline;
    vTaskMessage.operation = enum_TaskMessage::addTask;
    this->monitor->addTaskMessage(vTaskMessage);
    
}

void Auction::lockingComplete(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    this->monitor->print("I'm available to execute " + std::string(vMissionMessage->missionCode) + "!");
    
    s_MissionMessage missionMessage;
    
    // Send to the auctioneer that the mission was accepted
    strcpy(missionMessage.missionCode, vMissionMessage->missionCode);
    
    
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    
    missionMessage.operation = enum_MissionOperation::acceptMission;
    
    sendUDPMessage(missionMessage, *vMissionMessage->senderAddress, *vMissionMessage->senderName);
}

void Auction::startCommand(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    s_TaskMessage vTaskMessage;
    
    strcpy(vTaskMessage.missionCode,vMissionMessage->missionCode);
    vTaskMessage.operation = enum_TaskMessage::executeTask;
    this->monitor->addTaskMessage(vTaskMessage);
    
    this->MissionList.clear();
}


void Auction::abortMission(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    this->monitor->print("Redirecting " + std::string(vMissionMessage->missionCode) + "!");
    
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, vMissionMessage->missionCode);
    this->monitor->getRobotsIP(*vMissionMessage->senderAddress);
    missionMessage.operation = enum_MissionOperation::abortMission;
    
    sendUDPMessage(missionMessage, *vMissionMessage->senderAddress, *vMissionMessage->senderName);
}


void Auction::notifyingMissionComplete(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    s_MissionMessage missionMessage;
    
    // Send to the auctioneer that the mission was accepted
    strcpy(missionMessage.missionCode, vMissionMessage->missionCode);
    
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    
    missionMessage.operation = enum_MissionOperation::missionComplete;
    
    sendUDPMessage(missionMessage, *vMissionMessage->senderAddress, *vMissionMessage->senderName);
}

//****************************************************
//*       Functions Related to The auctioneer         *
//****************************************************

void Auction::createMission(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    strcpy(this->missionOwnerList[vMissionMessage->missionCode].missionCode, vMissionMessage->missionCode);
    this->monitor->getRobotsIP(*this->missionOwnerList[vMissionMessage->missionCode].senderAddress);
    
    this->monitor->getRobotsName(*this->missionOwnerList[vMissionMessage->missionCode].senderName);
    
    this->missionOwnerList[vMissionMessage->missionCode].mission = vMissionMessage->taskToBeDecomposed;
    this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
    
    //this->missionOwnerList[vMissionMessage->missionCode].goal = vMissionMessage->goal;
    this->missionOwnerList[vMissionMessage->missionCode].numberOfAttributes = vMissionMessage->numberOfAttributes;
    int totalSize = ((int*) vMissionMessage->attributesBuffer)[0];
    memcpy(this->missionOwnerList[vMissionMessage->missionCode].attributesBuffer,vMissionMessage->attributesBuffer, totalSize);
    
    
    this->missionOwnerList[vMissionMessage->missionCode].robotCategory = vMissionMessage->robotCat;
    this->missionOwnerList[vMissionMessage->missionCode].relativeDeadline = vMissionMessage->relativeDeadline;
    
    s_MissionStatus missionStatus;
    strcpy(missionStatus.missionCode, vMissionMessage->missionCode);
    strcpy(missionStatus.missionOwner, vMissionMessage->senderName);
    missionStatus.status = enum_MissionStatus::null;
    this->monitor->printMissionStatus(missionStatus);
    
    this->missionOwnerList[vMissionMessage->missionCode].t5 =  new std::thread(&Auction::missionRequestController, this, std::ref(this->missionOwnerList[vMissionMessage->missionCode].missionCode));
}


void Auction::waitingForBids(char* missionID)
{
    // Lock mutex and start timer
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    std::chrono::time_point now = std::chrono::system_clock::now();

    this->monitor->print("Waiting for Bids");
    
    // Send the mission to all robots
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, missionID);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    missionMessage.operation = enum_MissionOperation::addMission;
    missionMessage.taskToBeDecomposed = this->missionOwnerList[missionID].mission;
    
    
    //missionMessage.goal = this->missionOwnerList[missionID].goal;
    missionMessage.numberOfAttributes = this->missionOwnerList[missionID].numberOfAttributes;
    int totalSize = ((int*) this->missionOwnerList[missionID].attributesBuffer)[0];
    memcpy(missionMessage.attributesBuffer,this->missionOwnerList[missionID].attributesBuffer, totalSize);
    
    
    missionMessage.robotCat = this->missionOwnerList[missionID].robotCategory;
    missionMessage.relativeDeadline = this->missionOwnerList[missionID].relativeDeadline;
    char broadcast[MAX_ROBOT_ID] = "Broadcast";
    sendUDPMessage(missionMessage, *this->broadcastIP, *broadcast);
    
    s_MissionStatus missionStatus;
    strcpy(missionStatus.missionCode, missionID);
    strcpy(missionStatus.missionOwner,missionMessage.senderName);
    missionStatus.status = enum_MissionStatus::allocating;
    this->monitor->printMissionStatus(missionStatus);
    
    // Lock and wait intil the time is passed by
    this->missionOwnerList[missionID].cv->wait_until(lk, now + std::chrono::seconds(this->missionOwnerList[missionID].biddingTime));
    
    if(this->missionOwnerList[missionID].vectorBids.empty())
    {
        // If there is no available robots, start again.
        this->monitor->print("No Bids received for "+ std::string(missionID) +". Trying again.");
    } else
    {
        // If there is at least a bid, proceed.
        this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
    }
}

void Auction::addBidReceived(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // Check if the mission is still receiving bids.
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (this->missionOwnerList[vMissionMessage->missionCode].enum_request == enum_MissionRequest::waitingBids)
    {
        this->monitor->print(std::string(vMissionMessage->missionCode) + " received a bid from " + std::string(vMissionMessage->senderName) + "!");
        Bid bid;
        bid.price = vMissionMessage->Cost;
        strcpy(bid.bidderIP, vMissionMessage->senderAddress);
        strcpy(bid.bidderName, vMissionMessage->senderName);
        
        this->missionOwnerList[vMissionMessage->missionCode].vectorBids.push_back(bid);
    }
    lk.unlock();
}

void Auction::notifyingWinner(char* missionID)
{
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);
    if(this->missionOwnerList[missionID].vectorBids.empty() || this->missionOwnerList[missionID].vectorBids.size() == 0)
    {
        this->monitor->print("No one accepted " + std::string(missionID) + ". Offering again...");
        
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
        
        this->monitor->print(std::string(this->missionOwnerList[missionID].winnerName) + " won " + std::string(missionID) + "! Notifying!");
        
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
            this->monitor->print("Not accepted. Notifying the next bid.");
            this->missionOwnerList[missionID].enum_request = enum_MissionRequest::notifyingWinner;
        }else
        {
            this->missionOwnerList[missionID].enum_request = enum_MissionRequest::executingMission;
        }
            
    }
    lk.unlock();
}

void Auction::notifyingToExecute(char* missionID)
{
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[missionID].cv_m);

    this->monitor->print("Sending the execution command!");
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, missionID);
    
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    
    missionMessage.operation = enum_MissionOperation::startMission;
    
    sendUDPMessage(missionMessage, *this->missionOwnerList[missionID].winnerAddress, *this->missionOwnerList[missionID].winnerName);
    
    s_MissionStatus missionStatus;
    strcpy(missionStatus.missionCode, missionID);
    strcpy(missionStatus.missionOwner,missionMessage.senderName);
    strcpy(missionStatus.missionExecutioner, this->missionOwnerList[missionID].winnerName);
    missionStatus.status = enum_MissionStatus::executing;
    this->monitor->printMissionStatus(missionStatus);
    
    this->missionOwnerList[missionID].cv->wait(lk);
}

void Auction::missionAccepted(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // it is necessary to check if the winner still is the one selected to perform the mission.
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0 && strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerName, vMissionMessage->senderName)==0)
    {
        this->missionOwnerList[vMissionMessage->missionCode].missionAccepted = true;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
    }
}

void Auction::missionAborted(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // Check if the received message comes from the winning bid, clear bidding vector, restart auctionProcess.
    std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
    if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0)
    {
        this->monitor->print("Received an Abort Mission Command!");
        this->missionOwnerList[vMissionMessage->missionCode].vectorBids.clear();
        this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::waitingBids;
        this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
        
        s_MissionStatus missionStatus;
        strcpy(missionStatus.missionCode, vMissionMessage->missionCode);
        strcpy(missionStatus.missionOwner,robotName);
        strcpy(missionStatus.missionExecutioner, vMissionMessage->senderName);
        missionStatus.status = enum_MissionStatus::aborted;
        this->monitor->printMissionStatus(missionStatus);
        
        lk.unlock();
    }
}

void Auction::missionComplete(std::unique_ptr<s_MissionMessage> vMissionMessage)
{
    // Check if the received message comes from the winning bid and finish the mission
    try
    {
        std::unique_lock<std::mutex> lk(*this->missionOwnerList[vMissionMessage->missionCode].cv_m);
        if (strcmp(this->missionOwnerList[vMissionMessage->missionCode].winnerAddress, vMissionMessage->senderAddress) == 0)
        {
            this->missionOwnerList[vMissionMessage->missionCode].enum_request = enum_MissionRequest::missionComplete;
            this->missionOwnerList[vMissionMessage->missionCode].cv->notify_one();
            
            s_MissionStatus missionStatus;
            strcpy(missionStatus.missionCode, vMissionMessage->missionCode);
            strcpy(missionStatus.missionOwner,robotName);
            strcpy(missionStatus.missionExecutioner, vMissionMessage->senderName);
            missionStatus.status = enum_MissionStatus::complete;
            this->monitor->printMissionStatus(missionStatus);
            
            lk.unlock();
        }
    }
    catch(...)
    {

    }
}

//****************************************************
//*                    THE END                       *
//****************************************************


void Auction::calculateMissionCost(MissionExecution& mission)
{
    mission.missionCost = 0;
    for (auto n: mission.atomicTaskSequence)
    {
        mission.missionCost += n->getCost();
    }
}

void Auction::sendMissionCost(MissionExecution& mission)
{
    s_MissionMessage missionMessage;
    
    strcpy(missionMessage.missionCode, mission.missionCode);
    this->monitor->getRobotsIP(*missionMessage.senderAddress);
    this->monitor->getRobotsName(*missionMessage.senderName);
    missionMessage.operation = enum_MissionOperation::Bid;
    missionMessage.Cost = mission.missionCost;
    
    sendUDPMessage(missionMessage, *mission.senderAddress, *mission.senderName);
}

void Auction::sendUDPMessage(s_MissionMessage &vMissionMessage, char &targetAddress, char& targetName)
{
    s_UDPMessage message;
    
    strcpy(message.address, &targetAddress);
    strcpy(message.name, &targetName);
    memcpy(message.buffer, &targetName,MAX_ROBOT_ID);
    Operation operation = Operation::missionMessage;
    *((Operation*)(message.buffer + MAX_ROBOT_ID)) = operation;
    *((int*)(message.buffer + MAX_ROBOT_ID + 4)) = sizeof(vMissionMessage);
    memmove(message.buffer + MAX_ROBOT_ID + 8,(const unsigned char*)&vMissionMessage,sizeof(vMissionMessage));
    message.messageSize = sizeof(message.buffer);
    this->monitor->addUDPMessage(message);
}
