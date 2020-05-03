//
//  Module.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
// This class creates a thread module.
// In this case, there is no time constraints,
// this means that if you don't add a sleeping function, for example,
// it will be running indefinitely.
// This class is recommendended for non periodic tasks using semaphore
// or mutexes to control its execution.
// If you want a periodic module, use ModulePeriodic instead.


#ifndef Module_hpp
#define Module_hpp

#include <iostream>
#include <thread>
#include <chrono>
#include "BlackBoard.hpp"


class Module{
protected:
    std::thread t_main;
    bool isRunning;
    BlackBoard *monitor;
    virtual void mainThread();
    virtual void run();
    
public:
    Module(BlackBoard *monitor);
    ~Module();
    
    bool getRunningStatus();
    void start();
    virtual void stop();
    
};
#endif /* Module_hpp */
