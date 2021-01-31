//
//  ChargingStation.cpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ChargingStation.hpp"

ChargingStation::ChargingStation(Blackboard* monitor, bool decentralized): Agent(monitor)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::chargingStation);

    sender = new UDPSender(monitor);
    broadcast = new UDPBroadcast(monitor);
    char mode[] = "ChargingStation";
    batteryManager = new BatteryManager(monitor,mode);
    //missionManager = new MissionManager(monitor);
    
    auction = new Auction(monitor, this);
    taskModule = new TaskModule(monitor, this);
    
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
    taskModule->start();
    auction->start();
}

ChargingStation::~ChargingStation()
{
    delete this->broadcast;
    if (this->decentralized == true)
    {
        delete this->receiver;
        delete this->logger;
    }
    
    delete this->sender;
    //delete this->missionManager;
    delete this->batteryManager;
    
}
