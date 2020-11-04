//
//  webApp.cpp
//  MRSMac
//
//  Created by Afonso Braga on 28/10/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
#include "WebModule.hpp"


WebModule::WebModule(BlackBoard* monitor, int argc, char **argv)
: Module(monitor)
{
    this->monitor = monitor;
    this->argc = argc;
    this->argv = argv;
}

WebModule::~WebModule()
{
    
}

void WebModule::run()
{
    BlackBoard* monitor2 = monitor;
    /*Wt::WRun(argc, argv, [monitor2](const Wt::WEnvironment &env) {
     return std::make_unique<WebApp>(env,monitor2);
     });*/
    
    try {
        Wt::WServer server(argc, argv, WTHTTP_CONFIGURATION);
        
        server.addEntryPoint(Wt::EntryPointType::Application, [monitor2](const Wt::WEnvironment &env) {
            
            auto app = std::make_unique<Wt::WApplication>(env);
            
            app->setTitle("Overwatch");
            
            //app->messageResourceBundle().use(app->appRoot() + "strings");
            //app->messageResourceBundle().use(app->appRoot() + "templates");
            
            app->useStyleSheet("MRSstyle.css");
            
            app->root()->addWidget(std::make_unique<WebApp>(monitor2));
            
            return app;
            //          return std::make_unique<WebApp>(env,monitor2);
        });
        
        //Session::configureAuth();
        
        server.run();
    } catch (Wt::WServer::Exception& e) {
        std::cerr << e.what() << std::endl;
    } catch (std::exception &e) {
        std::cerr << "exception: " << e.what() << std::endl;
    }
    
}
/*
 void WebModule::Module::stop()
 {
 //Wt::stop();
 }*/
