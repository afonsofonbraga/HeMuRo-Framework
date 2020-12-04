//
//  TaskModule.hpp
//  MRSMac
//
//  Created by Afonso Braga on 30/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TaskModule_hpp
#define TaskModule_hpp

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

#include "DecomposableTasks.hpp"

class TaskModule: public Module
{
private:
    char broadcastIP[MAX_IP]="null";
    char robotName[MAX_ROBOT_ID] = "null";
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
    
    void addTaskReceived(std::unique_ptr<s_TaskMessage> vTaskMessage);
    void startCommand(std::unique_ptr<s_TaskMessage> vTaskMessage);
    void addMissionToExecute(MissionExecution& vMissionExecute);
    void startMissionToExecute();
    
    // Variables and functions related to mission emergency
    
    void addMissionEmergency(MissionExecution& vMissionExecute);
    bool getEmergencyStatus();
    bool setEmergency();
    bool cleanEmergecy();
    void redirectMission(MissionExecution& vMissionExecute);
    void emergencyCall(std::unique_ptr<s_TaskMessage> vTaskMessage);

    void calculateMissionCost(MissionExecution& mission);
    void missionRequestController(char* missionID);
    
public:
    TaskModule(BlackBoard* monitor);
    ~TaskModule();
};

#endif /* TaskModule_hpp */
