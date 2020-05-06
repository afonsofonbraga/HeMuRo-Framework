//
//  Task.cpp
//  MRSMac
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Task.hpp"

Task::Task(TaskDescription vTask, unsigned char* vatributes, int size)
{
    this->taskName = vTask;
    memcpy(this->atributes,vatributes,size);
}

Task::Task()
{
}


Task::~Task()
{
    
}
