//
//  BatteryManager.hpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef BatteryManager_hpp
#define BatteryManager_hpp


#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"


#include "ChargingSpot.hpp"
#include "ChargingRequest.hpp"


struct ChargingBid
{
    char bidderIP[15];
    char bidderName[10];
    float price;
};

struct ChargingStationWinner
{
    char requestID[10] = "null";
    char chargingIP[15] = "null";
    char chargingID[10] = "null";
    char spotID[10] = "null";
    float price;
    s_pose spotPosition;
};

class BatteryManager: public Module
{
private:
    char broadcastIP[16]="null";
    char agentName[10] = "null";
    char mode[16];
    std::vector<ChargingBid> vectorBids;
    ChargingStationWinner chargingStationWinner;
    
    std::thread* batteryCheck;
    std::condition_variable conditional_batteryCheck;
    std::mutex mutex_batteryCheck;
    
    enum_ChargingRequest batteryStatus = enum_ChargingRequest::null;
    
protected:
    
    virtual void run() override;
    virtual void mainThread() override;
    
    std::map<std::string, ChargingSpot> chargingSpotList;
    std::unordered_map <std::string, s_ChargingRequest> chargingRequestList;
    
    void chargingRequest(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    void bid(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    void winningBid(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    void acceptRequest(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    void arrivedAtStation(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    void startCharging(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    void chargingComplete(std::unique_ptr<s_BatteryMessage> vBatteryMessage);
    
    void batteryCheckLoop();
    
    void sendUDPMessage(s_BatteryMessage &vBatteryMessage, char &targetAddress, char& targetName);
        
public:
    BatteryManager(BlackBoard* monitor, char vMode[]);
    ~BatteryManager();
};

#endif /* BatteryManager_hpp */