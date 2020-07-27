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

//#include "Mission.hpp"
#include "MissionRequest.hpp"
#include "MissionExecution.hpp"

#include "GoTo.hpp"
#include "ChargeBattery.hpp"
#include "TurnOn.hpp"
#include "TakePicture.hpp"

class MissionManager: public Module
{
private:
    char broadcastIP[16]="null";
    std::map<std::string, MissionExecution> MissionList;
    std::unordered_map<std::string, MissionRequest> missionOwnerList;
    
    std::thread* executeMission;
    MissionExecution missionToExecute;
    std::condition_variable conditional_executeMission;
    std::mutex mutex_mission;
    bool emergency = false;
    std::mutex mutex_emergency;
    MissionExecution missionEmergency;
    
    
protected:
    // Here is the deal, we will create an ordened map, with the first argument the mission code that will be based on the name of the robot+3NUMBERS (ex. Thor001). This code will be also inside the Mission object. The second argument will be the Mission Object. It will be easier to locate the misison, I dont know if it is the most efficient way to programm it. Don`t care at the moment. Later will be a good thing to accept multiple missions to be executed.
    
    virtual void run() override;
    virtual void mainThread() override;
    
    
    // Starting to incorporate TaskManagerModule
    void createMission(s_MissionMessage* vMissionMessage);
    void addMissionReceived(s_MissionMessage* vMissionMessage);
    void addMissionCalculateCost(s_MissionMessage* vMissionMessage);
    void addBidReceived(s_MissionMessage* vMissionMessage);
    void winningBid(s_MissionMessage* vMissionMessage);
    void missionAccepted(s_MissionMessage* vMissionMessage);
    void startCommand(s_MissionMessage* vMissionMessage);
    void missionComplete(s_MissionMessage* vMissionMessage);
    void waitingForBids(char* missionID);
    void notifyingWinner(char* missionID);
    void notifyingToExecute(char* missionID);
    void notifyingMissionComplete();

    void addMissionToExecute(MissionExecution& vMissionExecute);
    void startMissionToExecute();
    
    // Variables and functions related to mission emergency

    void addMissionEmergency(MissionExecution& vMissionExecute);
    bool getEmergencyStatus();
    bool setEmergency();
    bool cleanEmergecy();
    void redirectMission(MissionExecution& vMissionExecute);
    void emergencyCall(s_MissionMessage* vMissionMessage);
    void missionAborted(s_MissionMessage* vMissionMessage);
    void addAtomicTask(MissionExecution& mission);
    void calculateMissionCost(MissionExecution& mission);
    void sendMissionCost(MissionExecution& mission);
    void sendUDPMessage(s_MissionMessage& vMissionMessage, char& address);
    void missionRequestController(char* missionID);
    
public:
    MissionManager(BlackBoard* monitor);
    ~MissionManager();
};

#endif /* MissionManager_hpp */
