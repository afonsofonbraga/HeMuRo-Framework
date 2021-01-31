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
#include <iomanip>
#include <fstream>
#include <string>
#include <chrono>

#include "Module.hpp"
#include "Blackboard.hpp"
#include "dataTypes.hpp"

#ifndef __has_include
  static_assert(false, "__has_include not supported");
#else
#  if __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif


class Logger: public Module
{
private:
    void constructor();
    
protected:
    std::string path;
    s_LoggerMessage* vLoggerMessage;
    void printcmd(s_LoggerMessage& vMessage);
    std::chrono::time_point<std::chrono::steady_clock> start;

    
    void run() override;
public:
    Logger(Blackboard* monitor);
    ~Logger();
};

#endif /* Logger_hpp */



 
