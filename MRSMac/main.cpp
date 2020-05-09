//
//  main.cpp
//  MRSMac
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include "BlackBoard.hpp"
#include "Module.hpp"
#include "ModulePeriodic.hpp"
#include <unordered_map>
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "TaskManager.hpp"
#include "dataTypes.hpp"
#include "Task.hpp"

int main(){
    
    std::vector<BlackBoard *> v_BlackBoard; // = new std::vector<BlackBoard>;
    std::vector<UDPBroadcast*> v_Broadcast;// = new std::vector<UDPBroadcast>;
    std::vector<UDPReceiver*> v_Receiver;
    std::vector<TaskManager*> v_TaskManager;
    
    unsigned char tes[]= "chato";
    unsigned char* value = new unsigned char;
    s_pose goal{10,10,0};
    memcpy(value, &goal, sizeof(s_pose));
    
    
    Task discharge(TaskDescription::turnOn, tes, sizeof(tes));
    Task charge(TaskDescription::chargeBattery, tes,sizeof(tes));
    Task walk(TaskDescription::goTo, value, sizeof(s_pose));
    
    std::vector<std::string> nomes{"Thor","Zeus","Chronos","Athena","Gaia","Artimedes","Dionisio","Pegasus","Percius","Poseidon"};
    int i = 0;
    //for (auto n: nomes)
    auto n = nomes.at(1);
    {
        //std::string nome = "Robo" + std::to_string(i);
        BlackBoard* memory = new BlackBoard(n);
        v_BlackBoard.push_back(memory);
        UDPBroadcast* broadcast = new UDPBroadcast(v_BlackBoard.at(i));
        UDPReceiver* receiver = new UDPReceiver(v_BlackBoard.at(i));
        TaskManager* taskManager = new TaskManager(v_BlackBoard.at(i));
        v_Broadcast.push_back(broadcast);
        v_Receiver.push_back(receiver);
        v_TaskManager.push_back(taskManager);
        i++;
    }
    /*
    for(int j=0; j< nomes.size(); j++)
    {
        v_BlackBoard.at(j)->addTask(discharge);
        v_BlackBoard.at(j)->addTask(walk);
        v_BlackBoard.at(j)->addTask(charge);
    }*/
    std::this_thread::sleep_for(std::chrono::seconds(100));
    
    return 0;
}
