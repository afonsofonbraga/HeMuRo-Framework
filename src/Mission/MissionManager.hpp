//
//  MissionManager.hpp
//  MRSMac
//
//  Created by Afonso Braga on 08/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef MissionManager_hpp
#define MissionManager_hpp

#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"
#include "AtomicTask.hpp"
#include "Mission.hpp"

#include "GoTo.hpp"
#include "ChargeBattery.hpp"
#include "TurnOn.hpp"

class MissionManager: public Module
{
protected:
    // Here is the deal, we will create an ordened map, with the first argument the mission code that will be based on the name of the robot+3NUMBERS (ex. Thor001). This code will be also inside the Mission object. The second argument will be the Mission Object. It will be easier to locate the misison, I dont know if it is the most efficient way to programm it. Don`t care at the moment. Later will be a good thing to accept multiple missions to be executed.
    
    char broadcastIP[16]="null";
    
    std::map<std::string, Mission> MissionList;
    Mission* vMission;
    s_MissionMessage* vMissionMessage;
    std::vector<enum_AtomicTask> vAtomicTaskVector;
    
    
    std::unordered_map<std::string, Mission> missionOwnerList;
    //std::unordered_map<std::string, int> timerList;
    virtual void run() override;
    
public:
    MissionManager(BlackBoard* monitor);
    ~MissionManager();
    void addAtomicTask();
    void calculateMissionCost(Mission& mission);
    void sendMissionCost(Mission& mission);
    void sendUDPMessage(s_MissionMessage& vMissionMessage, char& address);
    void timer(char* missionID);
};

#endif /* MissionManager_hpp */
