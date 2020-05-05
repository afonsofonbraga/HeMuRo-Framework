//
//  Task.hpp
//  MRSMac
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Task_hpp
#define Task_hpp

#include <iostream>
#include "dataTypes.hpp"

class Task{
public:
    taskDescription taskName;
    unsigned char atributes[100];
    
    Task(taskDescription vTask, unsigned char* vatributes);
    Task();
    ~Task();
    
};
#endif /* Task_hpp */
