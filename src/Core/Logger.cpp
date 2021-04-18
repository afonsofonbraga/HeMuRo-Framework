//
//  Logger.cpp
//  MRSMac
//
//  Created by Afonso Braga on 24/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Logger.hpp"
/*
Logger::Logger(Blackboard* monitor, int argc, char **argv) : Module(monitor)
{
    this->constructor();
    
    webmodule = new WebModule(monitor,argc, argv);
    webmodule->Module::start();
}*/

Logger::Logger(Blackboard* monitor): Module(monitor)
{
    this->constructor();
}

void Logger::constructor()
{
    path = std::string("$path_to_HeMuRo/logs/PathController.txt");
    int n;
    std::fstream s(path, s.in | s.out);
    if (!s.is_open()) {
        std::fstream s(path, s.trunc | s.in | s.out);
        n = 1;
        s << "Simulation " << n;
        
    } else {
        
        std::string str;
        if (s >> str >> n)
        {
            n++;
            s.seekg(0);
            s << "Simulation " << n;
        }else
        {
            n = 1;
            s.seekg(0);
            s << "Simulation " << n;
        }
        s.close();
    }
    path =  std::string("$path_to_HeMuRo/logs/Logger") + std::to_string(n);
    fs::create_directory(path);
    std::string name;
    this->monitor->getRobotsName(name);
    
    missionPath = path + "/Missions.txt";
    terminalPath = path + "/" + name + ".txt";
     
    this->start = std::chrono::steady_clock::now();
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    s.open(terminalPath, s.in | s.out | s.trunc);
    if (s.is_open())
    {
        s << "[0 s][SYSTEM] Log started at: " << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X") << "\n";
        s.close();
    }
    
    s.open(missionPath, s.in | s.out | s.trunc);
    if (s.is_open())
    {
        s << "[0 s][SYSTEM] Log started at: " << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X") << "\n";
        s << "[0 s][SYSTEM] Mission, Mission Owner, Mission Executioner, Relative Deadline , Estimated Execution Time, Execution Time, Status \n" ;
        s.close();
    }
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
            case enum_LoggerOperation::missionStatus:
            {
                missionStatus(*vLoggerMessage);
                break;
            }
            default:
                break;
        }
    }
}

void Logger::printcmd(s_LoggerMessage &vMessage)
{
    std::cout << "[" << vMessage.robotName << "] " << vMessage.buffer << std::endl;
    //webmodule ? webmodule->
    std::fstream s(terminalPath, s.ate | s.in | s.out );
    if (s.is_open())
    {
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end-start;
        s << "[" << diff.count() << " s][" << vMessage.robotName << "] " << vMessage.buffer <<" \n" ;
        s.close();
    }
    
}

void Logger::missionStatus(s_LoggerMessage &vMessage)
{
    s_MissionStatus missionStatusMessage = ((s_MissionStatus*) vMessage.buffer)[0];
    this->monitor->setMissionStatus(missionStatusMessage);
    this->monitor->getMissionStatus(missionStatusMessage);
    
    std::fstream s(missionPath, s.ate | s.in | s.out );
    if (s.is_open())
    {
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end-start;
        std::string status;
        switch(missionStatusMessage.status)
        {
            case enum_MissionStatus::null:
                status = "null";
                break;
            case enum_MissionStatus::allocating:
                status = "Allocating";
                break;
            case enum_MissionStatus::executing:
                status = "Executing";
                break;
            case enum_MissionStatus::complete:
                status = "Complete";
                break;
            case enum_MissionStatus::failure:
                status = "Failure";
                break;
            case enum_MissionStatus::aborted:
                status = "Redirected";
                break;
            case enum_MissionStatus::timeout:
                status = "Timeout";
                break;
            case enum_MissionStatus::lowBattery:
                status = "LowBattery";
                break;
            default:
                status = "null";
                break;
        }
        
        s << "[" << diff.count() << " s][" << vMessage.robotName << "] " << missionStatusMessage.missionCode << ", " << missionStatusMessage.missionOwner << ", "<< missionStatusMessage.missionExecutioner << ", " << missionStatusMessage.relativeDeadline.count() << ", " << missionStatusMessage.estimatedExecutionTime.count() << ", " << missionStatusMessage.executionTime.count() << ", " << status << " \n" ;
        s.close();
    }
    
}
