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

int main(){
    
    std::vector<BlackBoard *> vetorrr; // = new std::vector<BlackBoard>;
    std::vector<UDPBroadcast*> broadcasting;// = new std::vector<UDPBroadcast>;
    std::vector<UDPReceiver*> receiving;
    
    std::vector<std::string> nomes{"Thor","Zeus","Chronos","Athena","Gaia","Artimedes","Dionisio","Pegasus","Percius","Poseidon"};
    int i = 0;
    for (auto n: nomes)
    {
        std::string nome = "Robo" + std::to_string(i);
        BlackBoard* fulano = new BlackBoard(nome);
        vetorrr.push_back(fulano);
        UDPBroadcast* brodzin = new UDPBroadcast(vetorrr.at(i));
        UDPReceiver* recebedorzin = new UDPReceiver(vetorrr.at(i));
        broadcasting.push_back(brodzin);
        receiving.push_back(recebedorzin);
        i++;
    }
    std::this_thread::sleep_for(std::chrono::seconds(100));
    return 0;
}
