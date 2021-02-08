//
//  ChargingSpot.cpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ChargingSpot.hpp"

ChargingSpot::ChargingSpot()
{
    this->available = true;
}

ChargingSpot::~ChargingSpot()
{
    
}

bool ChargingSpot::isAvailable()
{
    return this->available;
}

bool ChargingSpot::lockSpot()
{
    if (this->available == true)
    {
        this->available = false;
        return true;
    }
    return false;
}

bool ChargingSpot::unlockSpot()
{
    if (this->available == false)
    {
        this->available = true;
        return true;
    }
    return false;
}

float ChargingSpot::getPower()
{
    return this->power;
}

enum_RobotCategory ChargingSpot::getChargerCompatibility()
{
    return chargerCompatibility;
}

void ChargingSpot::setChargerCompatibility(enum_RobotCategory cat)
{
    this->chargerCompatibility = cat;
}

void ChargingSpot::getSpotPosition(s_pose& vPose)
{
    vPose = this->chargerPosition;
}

void ChargingSpot::setSpotPosition(s_pose& vPose)
{
    this->chargerPosition = vPose;
}

bool ChargingSpot::assignChargingRequest(s_ChargingRequest& vChargingRequest)
{
    if (this->lockSpot() == true)
    {
        this->chargingRequest = vChargingRequest;
        return true;
    }
    return false;
}

bool ChargingSpot::clearChargingRequest()
{
    if (this->unlockSpot() == true)
    {
        return true;
    }
    return false;
}
