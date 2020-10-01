//
//  Logger.hpp
//  MRSMac
//
//  Created by Afonso Braga on 24/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Logger_hpp
#define Logger_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <chrono>

#include "Module.hpp"
#include "BlackBoard.hpp"
#include "dataTypes.hpp"

class Logger: public Module
{

private:
protected:
    std::string path;
    s_LoggerMessage* vLoggerMessage;
    void printcmd(s_LoggerMessage& vMessage);
    std::chrono::time_point<std::chrono::steady_clock> start;
    virtual void run();        
public:
    Logger(BlackBoard* monitor);
    ~Logger();
};

#endif /* Logger_hpp */



 
