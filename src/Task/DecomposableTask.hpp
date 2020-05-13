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

#include "AtomicTask.cpp"
#include "dataTypes.hpp"


class DecomposableTask
{
protected:
    
    std::vector<AtomicTask>* vTasks;
    std::unordered_map<enum_DecomposableTask, std::vector<enum_AtomicTask>* > avaliableTasks;
    
    float cost;
public:
    DecomposableTask(enum_DecomposableTask taskToBeDecomposed);
    ~DecomposableTask();
    
    void setAllCost();
    float getAllCost();
    bool isDecomposable();
    
};


#endif /* DecomposableTask_hpp */
