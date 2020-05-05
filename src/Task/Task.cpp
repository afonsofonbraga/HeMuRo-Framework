//
//  Task.cpp
//  MRSMac
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Task.hpp"

Task::Task(taskDescription vTask, unsigned char* vatributes)
{
    this->taskName = vTask;
    memcpy(this->atributes,vatributes,sizeof(s_pose));
}

Task::Task()
{
}


Task::~Task()
{
    
}
