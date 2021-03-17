//
//  ModulePeriodic.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ModulePeriodic.hpp"


ModulePeriodic::ModulePeriodic(Blackboard *monitor): Module(monitor){
    
}

ModulePeriodic::ModulePeriodic(Blackboard* monitor, int vTick): Module(monitor){
    this->tick = std::chrono::milliseconds(vTick);
}

ModulePeriodic::~ModulePeriodic()
{
    this->Module::~Module();
    //if (this->t_main.joinable())
    //    this->t_main.join();
}

void ModulePeriodic::mainThread()
{
    auto t0 = std::chrono::high_resolution_clock::now();
    while(this->isRunning == true)
    {
        std::this_thread::sleep_until(t0 + this->tick);
        this->run();
        t0 = t0 + this->tick;
    }
}

void ModulePeriodic::run()
{
    // Here comes the module code.
    // MUST NOT IMPLEMENT A LOOP
    std::cout << "Empty Periodic runnig thread!!!" <<std::endl;
}
