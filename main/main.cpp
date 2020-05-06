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

int main( int argc, char *argv[ ] ){
    
    if (argc != 2)
    {
        std::cerr << "Favor informar apenas o nome do robô como argumento.";
        return 0;
    }
    
    std::string nome{argv[1]};
    
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
    
    BlackBoard* memory = new BlackBoard(nome);
    UDPBroadcast* broadcast = new UDPBroadcast(memory);
    UDPReceiver* receiver = new UDPReceiver(memory);
    TaskManager* taskManager = new TaskManager(memory);
    
    memory->addTask(discharge);
    memory->addTask(walk);
    memory->addTask(charge);
    
    std::this_thread::sleep_for(std::chrono::seconds(100));
    delete memory;
    delete broadcast;
    delete receiver;
    delete taskManager;
    return 0;
}
