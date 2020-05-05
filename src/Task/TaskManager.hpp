//
//  TaskManager.hpp
//  MRSMac
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TaskManager_hpp
#define TaskManager_hpp

#include <iostream>
#include <vector>
#include <string>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"
//#include "taskDescription.hpp"

class TaskManager: public Module
{
private:

    virtual void run();
    
    
    // Robot Functions
    bool performingTask = false;
    Task* vTask;
    
    void taskSwitch(Task& menu);
    
    virtual void charging();
    virtual void turnOn();
    
    virtual void goToPosition();
    s_pose* goal = nullptr;
    
    
public:
    TaskManager(BlackBoard* monitor);
    ~TaskManager();
};
#endif /* TaskManager_hpp */
