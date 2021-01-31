//
//  Auction.hpp
//  MRSMac
//
//  Created by Afonso Braga on 30/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Auction_hpp
#define Auction_hpp

#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "Blackboard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"
#include "AtomicTask.hpp"

//#include "Mission.hpp"
#include "MissionRequest.hpp"
#include "MissionExecution.hpp"

//#include "DecomposableTasks.hpp"
#include "Agent.hpp"

class Auction: public Module
{
private:
    char broadcastIP[MAX_IP]="null";
    char robotName[MAX_ROBOT_ID] = "null";
    std::map<std::string, MissionExecution> MissionList;
    std::unordered_map<std::string, MissionRequest> missionOwnerList;
    Agent* agent;
    //MissionExecution missionToExecute;
    
protected:
    // Here is the deal, we will create an ordened map, with the first argument the mission code that will be based on the name of the robot+3NUMBERS (ex. Thor001). This code will be also inside the Mission object. The second argument will be the Mission Object. It will be easier to locate the misison, I dont know if it is the most efficient way to programm it. Don`t care at the moment. Later will be a good thing to accept multiple missions to be executed.
    
    virtual void run() override;
    
    
    // Starting to incorporate TaskManagerModule
    void createMission(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void addMissionReceived(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void addMissionCalculateCost(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void addBidReceived(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void winningBid(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void lockingComplete(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void missionAccepted(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void startCommand(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void missionComplete(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void waitingForBids(char* missionID);
    void notifyingWinner(char* missionID);
    void notifyingToExecute(char* missionID);
    void notifyingMissionComplete(std::unique_ptr<s_MissionMessage> vMissionMessage);

    void addMissionToExecute(MissionExecution& vMissionExecute);
    void startMissionToExecute();
    
    // Variables and functions related to mission emergency

    void abortMission(std::unique_ptr<s_MissionMessage> vMissionMessage);
    void missionAborted(std::unique_ptr<s_MissionMessage> vMissionMessage);
    //void addAtomicTask(MissionExecution& mission);
    void calculateMissionCost(MissionExecution& mission);
    void sendMissionCost(MissionExecution& mission);
    void sendUDPMessage(s_MissionMessage& vMissionMessage, char& targetAddress, char& targetName);
    
    void missionRequestController(char* missionID);
    
public:
    Auction(Blackboard* monitor);
    Auction(Blackboard* monitor, Agent* a);
    ~Auction();
};
#endif /* Auction_hpp */
