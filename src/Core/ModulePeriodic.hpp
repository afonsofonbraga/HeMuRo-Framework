//
//  ModulePeriodic.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ModulePeriodic_hpp
#define ModulePeriodic_hpp

#include <iostream>
#include <thread>
#include <chrono>
#include "BlackBoard.hpp"
#include "Module.hpp"


class ModulePeriodic: public Module {
protected:
    //BlackBoard* monitor;
    void mainThread() override;                                          // Thread object
    virtual void run() override ;                                                 // Thread Implementation
    std::chrono::milliseconds tick = std::chrono::milliseconds(1000);   // Threads period
public:
    ModulePeriodic(BlackBoard* monitor);                                // Constructor with a Default Period
    ModulePeriodic(BlackBoard* monitor, int vTick);                     // Ordnary Constructor
    ~ModulePeriodic();
};

#endif /* ModulePeriodic_hpp */
