//
//  Mission.cpp
//  MRSMac
//
//  Created by Afonso Braga on 12/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Mission.hpp"

Mission::Mission()
{
}


Mission::~Mission()
{
    //for(auto p:this->atomicTaskList)
    //{
        //delete p;
    //}
}

/*
Mission::Mission(const Mission& other)
{
}

Mission& Mission::operator=(const Mission &other)
{
    return *this;
}

void Mission::timer()
{
    cv = new std::condition_variable;
    cv_m = new std::mutex;
    
    std::unique_lock<std::mutex> lk(*this->cv_m);
    
    while(this->enum_request != enum_MissionRequest::missionComplete)
    {
        switch(this->enum_request)
        {
            case enum_MissionRequest::null:
                this->enum_request = enum_MissionRequest::waitingBids;
                break;
            case enum_MissionRequest::waitingBids:
            {
                auto now = std::chrono::system_clock::now();
                std::cout << "Waiting for Bids" << std::endl;
                cv->wait_until(lk, now + std::chrono::seconds(this->biddingTime));
                
                //Select the best bid;
                //Notify and wait for the answer
                
                this->enum_request = enum_MissionRequest::notifyingWinner;
                break;
            }
            case enum_MissionRequest::notifyingWinner:
            {
                auto now = std::chrono::system_clock::now();
                std::cout << "Notifying Winner" << std::endl;
                cv->wait_until(lk, now + std::chrono::seconds(this->communicationTime));
                this->enum_request = enum_MissionRequest::executingMission;
                break;
            }
            
            case enum_MissionRequest::executingMission:
            {
                auto now = std::chrono::system_clock::now();
                std::cout << "Executing Mission" << std::endl;
                cv->wait_until(lk, now + std::chrono::seconds(this->executionTime));
                this->enum_request = enum_MissionRequest::missionComplete;
                break;
            }
                
            case enum_MissionRequest::missionComplete:
                std::cout << "Mission Complete!" << std::endl;
                break;
        }
    }
    delete cv;
    delete cv_m;
}

void Mission::start(){
    std::cout<< "coida"<<std::endl;
    t5 = new std::thread(&Mission::timer, this);
}

*/
