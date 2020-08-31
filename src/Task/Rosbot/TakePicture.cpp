//
//  TakePicture.cpp
//  MRSMac
//
//  Created by Afonso Braga on 27/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TakePicture.hpp"

TakePicture::TakePicture(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

TakePicture::~TakePicture() {}

void TakePicture::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            this->status = enum_AtomicTaskStatus::running;
            break;
            
        case enum_AtomicTaskStatus::running:
            std::cout << "Taking a Picture."<< std::endl;
            this->status = enum_AtomicTaskStatus::completed;
            break;
        case enum_AtomicTaskStatus::completed:
            break;
    }
}

void TakePicture::calculateCost()
{
    this->cost = this->costMeter;
}

