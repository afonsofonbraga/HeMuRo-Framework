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
    virtual void mainThread();
    virtual void run();
    std::chrono::milliseconds tick = std::chrono::milliseconds(1000);
public:
    ModulePeriodic(BlackBoard* monitor);
    ModulePeriodic(BlackBoard* monitor, int vTick);
    ~ModulePeriodic();
};

#endif /* ModulePeriodic_hpp */
