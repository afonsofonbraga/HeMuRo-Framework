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
#include <string.h>
#include "dataTypes.hpp"

class Task{
public:
    TaskDescription taskName;
    unsigned char atributes[100];
    
    Task(TaskDescription vTask, unsigned char* vatributes, int size);
    Task();
    ~Task();
    
};
#endif /* Task_hpp */
