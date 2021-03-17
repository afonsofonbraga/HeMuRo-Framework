//
//  Module.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Module.hpp"

Module::Module(Blackboard *monitor)
{
    this->monitor = monitor;
    this->isRunning = false;
    //usleep(100000);
    //this->start();
}

Module::~Module()
{
    this->stop();
    //this->monitor->conditional_task.notify_one();
    if (this->t_main.joinable())
        this->t_main.join();
}

void Module::mainThread()
{
    while(this->isRunning == true)
    {
        this->run();
    }
}

void Module::run()
{
    // Here comes the module code.
    // MUST NOT IMPLEMENT A LOOP
    std::cout << "Empty runnig thread!!!" <<std::endl;
}

bool Module::getRunningStatus()
{
    return this->isRunning;
}

void Module::start()
{
    this->isRunning = true;
    t_main = std::thread(&Module::mainThread,this);
}

void Module::stop()
{
    this->isRunning = false;
}
