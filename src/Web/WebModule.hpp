//
//  webApp.hpp
//  MRSMac
//
//  Created by Afonso Braga on 28/10/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef webApp_hpp
#define webApp_hpp

#include <stdio.h>
#include <chrono>
#include <thread>

#include "Blackboard.hpp"
#include "dataTypes.hpp"
#include "Module.hpp"
#include "WebApp.hpp"

#include <Wt/WApplication.h>
#include <Wt/WServer.h>

class WebModule: public Module
{
protected:
    Blackboard*    monitor;
    WebApp* webapp;
    Wt::WServer* server;
    int argc;
    char** argv;
    void run() override;
public:
    WebModule(Blackboard* monitor, int argc, char **argv);
    ~WebModule();
};
#endif /* webApp_hpp */
