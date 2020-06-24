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
#include "UDPSender.hpp"
#include "TaskManager.hpp"
#include "dataTypes.hpp"
#include "Task.hpp"
#include "DecomposableTask.hpp"

int main(){
    
    std::vector<BlackBoard *> v_BlackBoard; // = new std::vector<BlackBoard>;
    std::vector<UDPBroadcast*> v_Broadcast;// = new std::vector<UDPBroadcast>;
    std::vector<UDPReceiver*> v_Receiver;
    std::vector<UDPSender*> v_Sender;
    std::vector<TaskManager*> v_TaskManager;
    
    
    
    std::vector<AtomicTask>* vector = new std::vector<AtomicTask>;
    
    unsigned char tes[]= "chato";
    unsigned char* value = new unsigned char;
    s_pose goal{10,10,0};
    memcpy(value, &goal, sizeof(s_pose));
    
    
    //Task discharge(TaskDescription::turnOn, tes, sizeof(tes));
    //Task charge(TaskDescription::chargeBattery, tes,sizeof(tes));
    //Task walk(TaskDescription::goTo, value, sizeof(s_pose));
    
    std::vector<std::string> nomes{"Thor","Zeus","Chronos","Athena","Gaia","Artimedes","Dionisio","Pegasus","Percius","Poseidon"};
    int i = 0;
    for (auto n: nomes)
    //auto n = nomes.at(1);
    {
        //std::string nome = "Robo" + std::to_string(i);
        BlackBoard* memory = new BlackBoard(n);
        v_BlackBoard.push_back(memory);
        UDPBroadcast* broadcast = new UDPBroadcast(v_BlackBoard.at(i));
        UDPReceiver* receiver = new UDPReceiver(v_BlackBoard.at(i));
        UDPSender* sender = new UDPSender(v_BlackBoard.at(i));
        TaskManager* taskManager = new TaskManager(v_BlackBoard.at(i));
        v_Broadcast.push_back(broadcast);
        v_Receiver.push_back(receiver);
        v_TaskManager.push_back(taskManager);
        i++;
        DecomposableTask teste(memory,enum_DecomposableTask::checkPosition, *vector);
        
    }
    s_UDPMessage message;
    strcpy(message.address , "10.0.0.109");
    //strcpy(message.buffer , "1111111111");
    
    Operation operation = Operation::missionAssignment;
    *((Operation*)message.buffer) = operation;
    
    enum_DecomposableTask tarefa;
    tarefa = enum_DecomposableTask::checkPosition;
    *((int*)(message.buffer + 4)) = sizeof(tarefa);
    
    memmove(message.buffer+8,(const unsigned char*)&tarefa,sizeof(tarefa));

    message.messageSize = sizeof(message.buffer);

    v_BlackBoard.at(0)->addUDPMessage(message);
    
    //for(int j=0; j< 1; j++)
    //{
     //   v_BlackBoard.at(j)->addTask(discharge);
      //  v_BlackBoard.at(j)->addTask(walk);
       // v_BlackBoard.at(j)->addTask(charge);
    //}
    for (int j=0; j<100;j++){
        v_BlackBoard.at(1)->addUDPMessage(message);
    }
    std::this_thread::sleep_for(std::chrono::seconds(100));
    
    return 0;
}
