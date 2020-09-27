//
//  Logger.cpp
//  MRSMac
//
//  Created by Afonso Braga on 24/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Logger.hpp"

Logger::Logger(BlackBoard* monitor): Module(monitor)
{
    
}

Logger::~Logger()
{
    this->stop();
    this->monitor->conditional_UDPMessageList.notify_one();
    if (this->t_main.joinable())
        this->t_main.join();
}

void Logger::run()
{
    vLoggerMessage = new s_LoggerMessage();
    this->monitor->getLoggerMessage(*vLoggerMessage);
    
    if (vLoggerMessage != nullptr && this->isRunning == true)
    {
        switch(vLoggerMessage->operation)
        {
            case enum_LoggerOperation::null:
            {
                break;
            }
            case enum_LoggerOperation::print:
            {
                printcmd(*vLoggerMessage);
                break;
            }
            case enum_LoggerOperation::save:
            {
                break;
            }
            case enum_LoggerOperation::printAndSave:
            {
                break;
            }
        }
    }
}

void Logger::printcmd(s_LoggerMessage &vMessage)
{
    std::cout << "[" << vMessage.robotName << "] " << vMessage.buffer << std::endl;
}
