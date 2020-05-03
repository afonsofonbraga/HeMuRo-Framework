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
    std::string lala = "Thor";
    BlackBoard teste = lala;
    UDPReceiver* server = new UDPReceiver(&teste);
    UDPBroadcast* envia = new UDPBroadcast(&teste);
    
    lala = "Zeus";
    BlackBoard teste1 = lala;
    UDPReceiver* server1 = new UDPReceiver(&teste1);
    UDPBroadcast* envia1 = new UDPBroadcast(&teste1);
    
    lala = "Chronos";
    BlackBoard teste2 = lala;
    UDPReceiver* server2 = new UDPReceiver(&teste2);
    UDPBroadcast* envia2 = new UDPBroadcast(&teste2);
    
    lala = "Athena";
    BlackBoard teste3 = lala;
    UDPReceiver* server3 = new UDPReceiver(&teste3);
    UDPBroadcast* envia3 = new UDPBroadcast(&teste3);

    lala = "Gaia";
    BlackBoard teste4 = lala;
    UDPReceiver* server4 = new UDPReceiver(&teste4);
    UDPBroadcast* envia4 = new UDPBroadcast(&teste4);
    
    lala = "Artimedes";
    BlackBoard teste5 = lala;
    UDPReceiver* server5 = new UDPReceiver(&teste5);
    UDPBroadcast* envia5 = new UDPBroadcast(&teste5);
    
    lala = "Dionisio";
    BlackBoard teste6 = lala;
    UDPReceiver* server6 = new UDPReceiver(&teste6);
    UDPBroadcast* envia6 = new UDPBroadcast(&teste6);
    
    lala = "Pegasus";
    BlackBoard teste7 = lala;
    UDPReceiver* server7 = new UDPReceiver(&teste7);
    UDPBroadcast* envia7 = new UDPBroadcast(&teste7);
    
    lala = "Percius";
    BlackBoard teste8 = lala;
    UDPReceiver* server8 = new UDPReceiver(&teste8);
    UDPBroadcast* envia8 = new UDPBroadcast(&teste8);
    
    lala = "Poseidon";
    BlackBoard teste9 = lala;
    UDPReceiver* server9 = new UDPReceiver(&teste9);
    UDPBroadcast* envia9 = new UDPBroadcast(&teste9);
    
    std::this_thread::sleep_for(std::chrono::seconds(100));
    return 0;
}
