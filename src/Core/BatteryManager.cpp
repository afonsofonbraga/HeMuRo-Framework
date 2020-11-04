//
//  BatteryManager.cpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "BatteryManager.hpp"

BatteryManager::BatteryManager(BlackBoard* monitor, char vMode[]) : Module(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->monitor->getRobotsName(*this->agentName);
    strcpy(this->mode,vMode);
    if (strcmp(mode,"ChargingStation") == 0)
    {
        s_pose position;
        this->monitor->getPosition(position);
        ChargingSpot spot;
        position.x = position.x;
        position.y = position.y;
        spot.setSpotPosition(position);
        spot.setChargerCompatibility(enum_RobotCategory::ugv);
        this->chargingSpotList["Spot1"] = spot;
        
        position.x = position.x + 1;
        position.y = position.y;
        spot.setSpotPosition(position);
        spot.setChargerCompatibility(enum_RobotCategory::ugv);
        this->chargingSpotList["Spot2"] = spot;
        
        position.x = position.x - 2;
        position.y = position.y;
        spot.setSpotPosition(position);
        spot.setChargerCompatibility(enum_RobotCategory::ugv);
        this->chargingSpotList["Spot3"] = spot;
        
    }
}

BatteryManager::~BatteryManager()
{
    conditional_batteryCheck.notify_all();
}

void BatteryManager::mainThread()
{
    if(strcmp(this->mode,"Robot") == 0)
        batteryCheck =  new std::thread(&BatteryManager::batteryCheckLoop, this);
    
    while(this->isRunning)
    {
        this->run();
    }
}

void BatteryManager::run()
{
    if(this->isRunning == true)
    {
        auto vBatteryMessage = std::make_unique<s_BatteryMessage>();
        this->monitor->getBatteryMessage(*vBatteryMessage);
        
        if (vBatteryMessage != nullptr && this->isRunning == true)
        {
            switch(vBatteryMessage->operation)
            {
                case enum_ChargingOperation::null:
                    break;
                case enum_ChargingOperation::chargingRequest:
                    if (strcmp(mode,"ChargingStation") == 0)
                        chargingRequest(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::bid:
                    bid(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::winningBid:
                    winningBid(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::acceptRequest:
                    acceptRequest(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::arrivedAtStation:
                    arrivedAtStation(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::startCharging:
                    startCharging(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::chargingComplete:
                    chargingComplete(std::move(vBatteryMessage));
                    break;
                case enum_ChargingOperation::atomicTaskInterrupt:
                {
                    conditional_batteryCheck.notify_one();
                    break;
                }
                    
            }
        }
    }
}

void BatteryManager::batteryCheckLoop()
{
    while(this->isRunning)
    {
        // HERE CAN BE USED AN ALGORITHM TO ESTIMATE LIFETIME
        std::unique_lock<std::mutex> lock1(mutex_batteryCheck);
        
        float batteryLevel = this->monitor->getBatteryLevel();
        
        switch(this->batteryStatus)
        {
            case enum_ChargingRequest::null:
            {
                this->batteryStatus = enum_ChargingRequest::ok;
                lock1.unlock();
                break;
            }
            case enum_ChargingRequest::ok:
            {
                //std::cout << "BatteryLevel: " << batteryLevel << std::endl;
                //this->monitor->print("BatteryLevel: " + std::to_string(batteryLevel));
                
                if(batteryLifeTime() == false)
                {
                    this->monitor->getRobotStatus() == enum_RobotStatus::lowBattery ? this->monitor->unlockRobot() : false;
                    auto t0 = std::chrono::high_resolution_clock::now();
                    conditional_batteryCheck.wait_until(lock1, t0 + std::chrono::seconds(5));
                } else
                    this->batteryStatus = enum_ChargingRequest::chargingRequest;
                lock1.unlock();
                break;
            }
            case enum_ChargingRequest::chargingRequest:
            {
                if(batteryLifeTime()) //Remember to modify this line in case of modyfing the conditions for the charging request
                {
                    this->monitor->lockRobot(enum_RobotStatus::lowBattery);
                    
                    s_BatteryMessage message;
                    this->monitor->getRobotsName(*message.senderName);
                    std::string id = message.senderName + std::to_string(this->countID); // The countID will be increased when the charging is fully completed
                    strcpy(this->requestID,id.c_str());
                    strcpy(message.requestID,this->requestID);
                    this->monitor->getRobotsIP(*message.senderAddress);
                    message.robotCat = this->monitor->getRobotsCategory();
                    message.operation = enum_ChargingOperation::chargingRequest;
                    
                    char broadcast[MAX_ROBOT_ID] = "Broadcast";
                    
                    sendUDPMessage(message, *this->broadcastIP, *broadcast);
                    
                    auto t0 = std::chrono::high_resolution_clock::now();
                    conditional_batteryCheck.wait_until(lock1, t0 + std::chrono::seconds(5));
                    this->batteryStatus = enum_ChargingRequest::notfyingWinner;
                } else
                    this->batteryStatus = enum_ChargingRequest::ok;
                lock1.unlock();
                break;
            }
            case enum_ChargingRequest::notfyingWinner:
            {
                if(this->vectorBids.empty() || this->vectorBids.size() == 0)
                {
                    //std::cout << "[robot] Charging request ignored by all. Trying again." << std::endl;
                    this->monitor->print("Charging request ignored by all. Trying again.");
                    
                    this->batteryStatus = enum_ChargingRequest::ok;
                } else
                {
                    int winner = 0;
                    for(int i = 0; i < this->vectorBids.size(); i++)
                    {
                        if (this->vectorBids.at(winner).price > this->vectorBids.at(i).price)
                            winner = i;
                    }
                    
                    strcpy(chargingStationWinner.chargingIP,this->vectorBids.at(winner).bidderIP);
                    strcpy(chargingStationWinner.chargingID,this->vectorBids.at(winner).bidderName);
                    chargingStationWinner.price = this->vectorBids.at(winner).price;
                    this->vectorBids.erase(this->vectorBids.begin() + winner);
                    
                    s_BatteryMessage message;
                    strcpy(message.requestID,this->requestID);
                    this->monitor->getRobotsName(*message.senderName);
                    this->monitor->getRobotsIP(*message.senderAddress);
                    message.robotCat = this->monitor->getRobotsCategory();
                    message.operation = enum_ChargingOperation::winningBid;
                    
                    sendUDPMessage(message, *chargingStationWinner.chargingIP, *chargingStationWinner.chargingID);
                    
                    auto t0 = std::chrono::high_resolution_clock::now();
                    if(conditional_batteryCheck.wait_until(lock1, t0 + std::chrono::seconds(2)) == std::cv_status::no_timeout)
                        this->batteryStatus = enum_ChargingRequest::goingToLocation;
                }
                lock1.unlock();
                break;
            }
            case enum_ChargingRequest::goingToLocation:
            {
                s_MissionMessage missionMessage;
                
                strcpy(missionMessage.missionCode, chargingStationWinner.requestID);
                missionMessage.operation = enum_MissionOperation::emergency;
                missionMessage.taskToBeDecomposed = enum_DecomposableTask::lowBattery;
                missionMessage.goal = chargingStationWinner.spotPosition;
                this->monitor->addMissionMessage(missionMessage);
                
                //auto t0 = std::chrono::high_resolution_clock::now();
                //if(conditional_batteryCheck.wait_until(lock1, t0 + std::chrono::seconds(15)) == std::cv_status::no_timeout)
                
                conditional_batteryCheck.wait(lock1);
                {
                    s_BatteryMessage message;
                    strcpy(message.requestID,this->requestID);
                    strcpy(message.spotID,chargingStationWinner.spotID);
                    this->monitor->getRobotsName(*message.senderName);
                    this->monitor->getRobotsIP(*message.senderAddress);
                    message.operation = enum_ChargingOperation::arrivedAtStation;
                    
                    sendUDPMessage(message, *chargingStationWinner.chargingIP, *chargingStationWinner.chargingID);
                    
                    this->batteryStatus = enum_ChargingRequest::charging;
                    
                }
                
                lock1.unlock();
                break;
            }
            case enum_ChargingRequest::charging:
            {
                if(batteryLevel == 100)
                {
                    s_BatteryMessage message;
                    strcpy(message.requestID,this->requestID);
                    this->monitor->getRobotsName(*message.senderName);
                    this->monitor->getRobotsIP(*message.senderAddress);
                    message.operation = enum_ChargingOperation::chargingComplete;
                    strcpy(message.spotID,this->chargingStationWinner.spotID);
                    
                    sendUDPMessage(message, *chargingStationWinner.chargingIP, *chargingStationWinner.chargingID);
                    
                    this->batteryStatus = enum_ChargingRequest::chargingComplete;
                    
                } else
                {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    conditional_batteryCheck.wait_until(lock1, t0 + std::chrono::seconds(5));
                }
                lock1.unlock();
                break;
            }
            case enum_ChargingRequest::chargingComplete:
            {
                this->batteryStatus = enum_ChargingRequest::ok;
                this->countID ++;
                lock1.unlock();
                break;
            }
        }
    }
}

//****************************************************
//*           Functions Related to The CS            *
//****************************************************
void BatteryManager::chargingRequest(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    //std::cout << "[CS " << this->agentName << "] Received a charging request from " << vBatteryMessage->senderName << "!"<< std::endl;
    this->monitor->print("Received a charging request from " + std::string(vBatteryMessage->senderName));
    
    bool freeSpot = false;
    for (auto n : this->chargingSpotList)
    {
        if (n.second.isAvailable() == true && vBatteryMessage->robotCat == n.second.getChargerCompatibility())
        {
            freeSpot = true;
            break;
        }
    }
    
    if (freeSpot == true)
    {
        //Adding Requesto to List
        memmove(this->chargingRequestList[vBatteryMessage->requestID].requestID, vBatteryMessage->requestID, MAX_ID);
        this->chargingRequestList[vBatteryMessage->requestID].operation = vBatteryMessage->operation;
        this->chargingRequestList[vBatteryMessage->requestID].robotCat = vBatteryMessage->robotCat;
        this->chargingRequestList[vBatteryMessage->requestID].robotsPosition = vBatteryMessage->position;
        memmove(this->chargingRequestList[vBatteryMessage->requestID].robotsAddress, vBatteryMessage->senderAddress, MAX_IP);
        memmove(this->chargingRequestList[vBatteryMessage->requestID].robotsName, vBatteryMessage->senderName, MAX_ROBOT_ID);
        
        //Notifying Robot
        s_BatteryMessage message;
        
        strcpy(message.requestID,vBatteryMessage->requestID);
        this->monitor->getRobotsName(*message.senderName);
        this->monitor->getRobotsIP(*message.senderAddress);
        message.operation = enum_ChargingOperation::bid;
        this->monitor->getPosition(message.position);
        message.Cost = 10; //COST NEEDS TO BE FULLFILLED
        
        sendUDPMessage(message, *this->chargingRequestList[vBatteryMessage->requestID].robotsAddress,*this->chargingRequestList[vBatteryMessage->requestID].robotsName);
        
        //std::cout << "[CS " << this->agentName << "] Available charging spot! Informing... " << std::endl;
        this->monitor->print("Available charging spot! Informing...");
    }
}

void BatteryManager::winningBid(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    // This ChargingStation is the Auction's winner!!
    // First let's check if the Request is stored on our database
    auto search = chargingRequestList.find(vBatteryMessage->requestID);
    if (search != chargingRequestList.end()) {
        
        for (auto n : this->chargingSpotList)
        {
            if (this->chargingSpotList[n.first].isAvailable() == true && vBatteryMessage->robotCat == n.second.getChargerCompatibility())
            {
                if (this->chargingSpotList[n.first].assignChargingRequest(chargingRequestList[vBatteryMessage->requestID]) == true)
                {
                    //std::cout << "[CS " << this->agentName << "] Charging spot reserved for " << vBatteryMessage->senderName << "!"<< std::endl;
                    this->monitor->print("Charging spot reserved for " + std::string(vBatteryMessage->senderName) + "!");
                    
                    //Notifying Robot
                    s_BatteryMessage message;
                    
                    strcpy(message.requestID,vBatteryMessage->requestID);
                    this->monitor->getRobotsName(*message.senderName);
                    this->monitor->getRobotsIP(*message.senderAddress);
                    message.operation = enum_ChargingOperation::acceptRequest;
                    strcpy(message.spotID,n.first.c_str());
                    this->chargingSpotList[n.first].getSpotPosition(message.position); // NOW SENDIG THE SPOT LOCATION
                    
                    // Now the informations regarding the Charging Request belongs to the charging spot. Maybe this will be difficult to access
                    sendUDPMessage(message, *vBatteryMessage->senderAddress, *vBatteryMessage->senderName);
                    break;
                }
            }
        }
    }
}
void BatteryManager::arrivedAtStation(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    //IF ARRIVED AT THE STATION START CHARGING
    //HERE DONT NEED TO DO ANYTHING JUST SEND BACK A COMMAND
    
    //std::cout << "[CS " << this->agentName << "] " << vBatteryMessage->senderName << " arrived at CS " << this->agentName << "!"<< std::endl;
    this->monitor->print(std::string(vBatteryMessage->senderName) + " arrived at CS " + std::string(this->agentName) + "!");
    
    auto search = chargingSpotList.find(vBatteryMessage->spotID);
    if (search != chargingSpotList.end()) {
        if( strcmp(chargingSpotList[vBatteryMessage->spotID].chargingRequest.requestID,vBatteryMessage->requestID) == 0)
        {
            s_BatteryMessage message;
            
            strcpy(message.requestID,vBatteryMessage->requestID);
            this->monitor->getRobotsName(*message.senderName);
            this->monitor->getRobotsIP(*message.senderAddress);
            message.operation = enum_ChargingOperation::startCharging;
            
            sendUDPMessage(message, *vBatteryMessage->senderAddress, *vBatteryMessage->senderName);
        }
    }
}

void BatteryManager::chargingComplete(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    //std::cout << "[CS " << this->agentName << "] " << vBatteryMessage->senderName << " finished charging!"<< std::endl;
    this->monitor->print(std::string(vBatteryMessage->senderName) + " finished charging!");
    
    auto search = chargingSpotList.find(vBatteryMessage->spotID);
    if (search != chargingSpotList.end()) {
        if( strcmp(chargingSpotList[vBatteryMessage->spotID].chargingRequest.requestID,vBatteryMessage->requestID) == 0)
        {
            chargingSpotList[vBatteryMessage->spotID].clearChargingRequest();
        }
    }
}

//****************************************************
//*           Functions Related to The Robot         *
//****************************************************

void BatteryManager::bid(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    //std::cout << "[Robot] I received a bid from " <<vBatteryMessage->senderName << " !"<< std::endl;
    this->monitor->print("I received a bid from " + std::string(vBatteryMessage->senderName) + " !");
    
    ChargingBid bid;
    s_pose robotsPosition;
    
    this->monitor->getPosition(robotsPosition);
    float distance = sqrtf(pow(vBatteryMessage->position.x - robotsPosition.x, 2) + pow(vBatteryMessage->position.y - robotsPosition.y, 2));
    
    //Prices bid is the charging cost and euclidian distance sum.
    bid.price = vBatteryMessage->Cost + distance;
    strcpy(bid.bidderIP, vBatteryMessage->senderAddress);
    strcpy(bid.bidderName, vBatteryMessage->senderName);
    
    this->vectorBids.push_back(bid);
}

void BatteryManager::acceptRequest(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    //std::cout << "[Robot] Charging request approved by " <<vBatteryMessage->senderName << "! Going to location!"<< std::endl;
    this->monitor->print("Charging request approved by " + std::string(vBatteryMessage->senderName) + "! Going to location!");
    
    
    strcpy(this->chargingStationWinner.spotID, vBatteryMessage->spotID);
    this->chargingStationWinner.spotPosition = vBatteryMessage->position;
    this->batteryStatus = enum_ChargingRequest::goingToLocation;
    
    //std::cout << "Spot ID: " << vBatteryMessage->spotID << " Global Position -> X: " << vBatteryMessage->position.x << " y: " << vBatteryMessage->position.y << std::endl;
    this->monitor->print("Spot ID: " + std::string(vBatteryMessage->spotID) + " Global Position -> X: " + std::to_string(vBatteryMessage->position.x) + " y: " + std::to_string(vBatteryMessage->position.y));
    conditional_batteryCheck.notify_one();
    
}

void BatteryManager::startCharging(std::unique_ptr<s_BatteryMessage> vBatteryMessage)
{
    //std::cout << "[Robot] Start charging!"<< std::endl;
    this->monitor->print("Start charging!");
    
    conditional_batteryCheck.notify_one();
    // THIS IS A BIT COMPLICATED; THIS COMMAND NEEDS TO BE ATTACHED TO THE ATOMICTASK.
}


float BatteryManager::minimumDistance()
{
    std::unordered_map<std::string, s_BroadcastMessage> list;
    s_pose robotsPosition;
    float dist = 0;
    
    this->monitor->getAllRobotsPosition(list);
    this->monitor->getPosition(robotsPosition);
    for (auto n : list)
    {
        if (n.second.robotCategory == enum_RobotCategory::chargingStation && n.second.robotStatus == enum_RobotStatus::available)
        {
            float vDist = sqrtf(pow(n.second.robotsPosition.x - robotsPosition.x, 2) + pow(n.second.robotsPosition.y - robotsPosition.y, 2));
            if (dist == 0 || dist > vDist)
            {
                dist = vDist;
            }
        }
    }
    return dist;
}

bool BatteryManager::batteryLifeTime()
{
    bool status = false;
    float batteryLevel = this->monitor->getBatteryLevel();
    
    batteryLevel <= 30 ? status = true : false;
    (minimumDistance() * 1 > batteryLevel) ? status = true : false; //inventei

    return status;
}

//**************************************
//*          Common Functions          *
//**************************************

void BatteryManager::sendUDPMessage(s_BatteryMessage &vBatteryMessage, char &targetAddress, char& targetName)
{
    s_UDPMessage message;
    
    strcpy(message.address, &targetAddress);
    strcpy(message.name, &targetName);
    memcpy(message.buffer, &targetName,MAX_ROBOT_ID);
    Operation operation = Operation::batteryMessage;
    *((Operation*)(message.buffer + MAX_ROBOT_ID)) = operation;
    *((int*)(message.buffer + MAX_ROBOT_ID + 4)) = sizeof(vBatteryMessage);
    memmove(message.buffer + MAX_ROBOT_ID + 8,(const unsigned char*)&vBatteryMessage,sizeof(vBatteryMessage));
    message.messageSize = sizeof(message.buffer);
    this->monitor->addUDPMessage(message);
}
