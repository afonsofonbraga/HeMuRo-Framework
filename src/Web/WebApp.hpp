//
//  WebApp.hpp
//  MRSMac
//
//  Created by Afonso Braga on 03/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef WebApp_hpp
#define WebApp_hpp

#include <stdio.h>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>

#include "Blackboard.hpp"
#include "dataTypes.hpp"
#include "Module.hpp"
#include "WebApp.hpp"

#include <Wt/WBorderLayout.h>
#include <Wt/WContainerWidget.h>
#include <Wt/WLineEdit.h>
#include <Wt/WTable.h>
#include <Wt/WTableCell.h>
#include <Wt/WText.h>
#include <Wt/WTextArea.h>
#include <Wt/WTimer.h>

#include <Wt/WImage.h>
#include <Wt/WLink.h>

#include <Wt/WProgressBar.h>


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


class WebApp : public Wt::WContainerWidget
{
public:
    WebApp(Blackboard* monitor);
    ~WebApp();
    void                          insertTerminalLine();
    
private:
    Blackboard*                   monitor;
    Wt::WTable*                   table;
    std::unique_ptr<Wt::WTimer>   timer_;
    Wt::WTextArea*                terminal;
    void                          updateAgentList();
protected:
    std::string                   path;
    s_LoggerMessage*              vLoggerMessage;
};


#endif /* WebApp_hpp */
