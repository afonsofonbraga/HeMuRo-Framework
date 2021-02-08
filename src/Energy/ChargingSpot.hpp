//
//  ChargingSpot.hpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ChargingSpot_hpp
#define ChargingSpot_hpp

#include <stdio.h>
#include "dataTypes.hpp"
//#include "ChargingRequest.hpp"

class ChargingSpot
{
protected:
    s_pose chargerPosition;
    bool available;
    float power;
    enum_RobotCategory chargerCompatibility;
    
    
public:
    s_ChargingRequest chargingRequest;
    ChargingSpot();
    ~ChargingSpot();
    
    bool isAvailable();
    bool lockSpot();
    bool unlockSpot();
    
    float getPower();
    enum_RobotCategory getChargerCompatibility();
    void setChargerCompatibility(enum_RobotCategory cat);
    void getSpotPosition(s_pose& vPose);
    void setSpotPosition(s_pose& vPose);
    
    bool assignChargingRequest(s_ChargingRequest& vChargingRequest);
    bool clearChargingRequest();
};
#endif /* ChargingSpot_hpp */
