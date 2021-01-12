//
//  Logger.cpp
//  MRSMac
//
//  Created by Afonso Braga on 24/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Logger.hpp"
/*
Logger::Logger(BlackBoard* monitor, int argc, char **argv) : Module(monitor)
{
    this->constructor();
    
    webmodule = new WebModule(monitor,argc, argv);
    webmodule->Module::start();
}*/

Logger::Logger(BlackBoard* monitor): Module(monitor)
{
    this->constructor();
}

void Logger::constructor()
{
    path = getenv("HOME") + std::string("/Github/MRSFramework/logs/PathController.txt");
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
    path = getenv("HOME") + std::string("/Github/MRSFramework/logs/Logger") + std::to_string(n);
    fs::create_directory(path);
    std::string name;
    this->monitor->getRobotsName(name);
    path = path + "/" + name + ".txt";
    
    s.open(path, s.in | s.out | s.trunc);
    if (s.is_open())
    {
        this->start = std::chrono::steady_clock::now();
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        s << "[SYSTEM] Log started at: " << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X") << "\n";
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
            default:
                break;
        }
    }
}

void Logger::printcmd(s_LoggerMessage &vMessage)
{
    std::cout << "[" << vMessage.robotName << "] " << vMessage.buffer << std::endl;
    //webmodule ? webmodule->
    std::fstream s(path, s.ate | s.in | s.out );
    if (s.is_open())
    {
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end-start;
        s << "[" << std::setprecision(2) << diff.count() << " s][" << vMessage.robotName << "] " << vMessage.buffer <<" \n" ;
        s.close();
    }
    
}
