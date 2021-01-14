//
//  chargeBattery.cpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ChargeBattery.hpp"



ChargeBattery::ChargeBattery(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

ChargeBattery::~ChargeBattery(){}

void ChargeBattery::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
        {
            std::cout << "Carregando Bateria."<< std::endl;
            
            //Send a interrupt to inform that the robot has arrived at the station.
            s_BatteryMessage message;
            message.operation = enum_ChargingOperation::atomicTaskInterrupt;
            this->monitor->addBatteryMessage(message);
            t0 = std::chrono::system_clock::now();
            this->status = enum_AtomicTaskStatus::running;
            break;
        }

            
        case enum_AtomicTaskStatus::running:
            if(this->monitor->getBatteryLevel() < 100)
            {
                std::this_thread::sleep_until(t0 + this->tick);
                this->monitor->chargeBattery(1.0);
                t0 = t0 + this->tick;
                
            } else
            {
                std::cout << "Bateria Carregada!"<< std::endl;
                
                //Send a interrupt to inform that the charging is complete!"
                s_BatteryMessage message;
                message.operation = enum_ChargingOperation::atomicTaskInterrupt;
                this->monitor->addBatteryMessage(message);

                this->status = enum_AtomicTaskStatus::completed;
            }
            break;
        case enum_AtomicTaskStatus::completed:
            break;
        default:
            break;
    }
}

void ChargeBattery::calculateCost()
{
    this->cost = this->costMeter;
}

