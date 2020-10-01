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
#include <unistd.h>


class Module{
protected:
    std::thread t_main;         // Thread object
    bool isRunning;
    BlackBoard *monitor;
    virtual void mainThread();  // Configure the Tick
    virtual void run();         // Implementation
    
public:
    Module(BlackBoard *monitor);
    ~Module();
    
    bool getRunningStatus();    // Return isRunning variable
    void start();               // Start Thread
    virtual void stop();        // Pause Running Thread
    
};
#endif /* Module_hpp */
