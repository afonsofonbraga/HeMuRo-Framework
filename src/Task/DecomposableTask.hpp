//
//  DecomposableTask.hpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef DecomposableTask_hpp
#define DecomposableTask_hpp

#include <iostream>
#include <vector>
#include <unordered_map>

#include "AtomicTask.hpp"
#include "dataTypes.hpp"
#include "BlackBoard.hpp"

#include "GoTo.hpp"
#include "ChargeBattery.hpp"
#include "TurnOn.hpp"


class DecomposableTask
{
protected:
    
    BlackBoard* monitor;
    
    
    std::vector<AtomicTask>* vTasks;
    enum_DecomposableTask taskToBeDecomposed;
    std::unordered_map<enum_DecomposableTask, std::vector<enum_AtomicTask>* > avaliableTasks;
    
    bool decomposable;
    float cost;
    
public:
    DecomposableTask(BlackBoard* vMonitor,  enum_DecomposableTask vtaskToBeDecomposed);
    DecomposableTask(BlackBoard* vMonitor,  enum_DecomposableTask vtaskToBeDecomposed, std::vector<AtomicTask>& taskVector);
    ~DecomposableTask();
    
    void setAllCost();
    float getAllCost();
    bool isDecomposable();
    
};


#endif /* DecomposableTask_hpp */
