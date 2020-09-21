//
//  ChargingStation.cpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ChargingStation.hpp"

ChargingStation::ChargingStation(BlackBoard* monitor, bool decentralized)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::null);
    broadcast = new UDPBroadcast(monitor);
    if (this->decentralized == true)
        receiver = new UDPReceiver(monitor);
    sender = new UDPSender(monitor);
    missionManager = new MissionManager(monitor);
    char mode[] = "ChargingStation";
    batteryManager = new BatteryManager(monitor,mode);
}

ChargingStation::~ChargingStation()
{
    delete this->broadcast;
    if (this->decentralized == true)
        delete this->receiver;
    delete this->sender;
    delete this->missionManager;
    delete this->batteryManager;
}
