//
//  WebApp.cpp
//  MRSMac
//
//  Created by Afonso Braga on 03/11/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "WebApp.hpp"

WebApp::WebApp(Blackboard* monitor): WContainerWidget()
{
    this->monitor = monitor;
    
    path = std::string("/home/robot/github/HeMuRo-Framework/logs/PathController.txt");
    int n = 0;
    std::string str;
    std::fstream s(path, s.in);
    if (s.is_open()) {
        s >> str >> n;
        s.close();
    }
    path = std::string("/home/robot/github/HeMuRo-Framework/logs/Logger") + std::to_string(n) + "/Logger.txt";
    
    setHeight(720);
    setWidth(1024);
    setStyleClass("backgound-box");
    
    auto layout = setLayout(Wt::cpp14::make_unique<Wt::WBorderLayout>());
    auto container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    
    auto item = container->addWidget(Wt::cpp14::make_unique<Wt::WText>("<h1>HeMuRo Framework</h1>"));
    container->setStyleClass("title-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::North);
    /*
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    item = container->addWidget(Wt::cpp14::make_unique<Wt::WText>("<h3>Menu</h3>"));
    container->setStyleClass("sidebar-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::West);
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    item = container->addWidget(Wt::cpp14::make_unique<Wt::WText>("<h3>Information</h3> Here you can find all the <br /> information regarding <br />agents' status.<br />More information will be <br /> available in the future!<br /> Keep in touch!"));
    container->setStyleClass("sidebar-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::East); */
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    
    auto containerTable = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    agentsTable = containerTable->addWidget(Wt::cpp14::make_unique<Wt::WTable>());
    containerTable->setStyleClass("table-box");

    Wt::WTabWidget *tabW = container->addNew<Wt::WTabWidget>();
    
    tabW->addTab(std::move(containerTable),
                 "Agents List", Wt::ContentLoading::Eager);
    
    containerTable = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    missionsTable = containerTable->addWidget(Wt::cpp14::make_unique<Wt::WTable>());
    containerTable->setStyleClass("table-box");
    
    tabW->addTab(std::move(containerTable),
                 "Missions List", Wt::ContentLoading::Eager);

    tabW->setStyleClass("tab-box");
    
    
    layout->addWidget(std::move(container), Wt::LayoutPosition::Center);
    
    
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    container->addWidget(Wt::cpp14::make_unique<Wt::WText>("Terminal"));
    container->addWidget(Wt::cpp14::make_unique<Wt::WBreak>());
    terminal = container->addWidget(Wt::cpp14::make_unique<Wt::WTextArea>());
    
    terminal->setColumns(135);
    terminal->setRows(15);
    terminal->setReadOnly(true);
    terminal->setText("Starting Web Terminal...");
    
    container->setStyleClass("terminal-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::South);
    
    
    timer_ = Wt::cpp14::make_unique<Wt::WTimer>();
    timer_->setInterval(std::chrono::milliseconds{1000});
    timer_->timeout().connect(this, &WebApp::insertTerminalLine);
    timer_->timeout().connect(this, &WebApp::updateAgentList);
    timer_->timeout().connect(this, &WebApp::updateMissionList);
    timer_->start();
}

WebApp::~WebApp()
{
}

void WebApp::updateMainContainer()
{
    this->updateMissionList();
}

void WebApp::updateAgentList()
{
    agentsTable->clear();
    
    agentsTable->setHeaderCount(1);
    agentsTable->setWidth(Wt::WLength("100%"));
    agentsTable->elementAt(0, 0)->addNew<Wt::WText>("#");
    agentsTable->elementAt(0, 1)->addNew<Wt::WText>("Category");
    agentsTable->elementAt(0, 2)->addNew<Wt::WText>("Agent Name");
    agentsTable->elementAt(0, 3)->addNew<Wt::WText>("Position (x,y,z)");
    agentsTable->elementAt(0, 4)->addNew<Wt::WText>("Battery Level");
    agentsTable->elementAt(0, 5)->addNew<Wt::WText>("Status");
    
    
    int row = 0;
    std::unordered_map<std::string, s_BroadcastMessage> robots;
    this->monitor->getAllRobotsPosition(robots);
    //for (auto n : this->v_Blackboard.at(0)->mapRobotsPosition)
    std::string category;
    std::string img;
    std::string status;
    
    for (auto n : robots)
    {
        row++;
        switch(n.second.robotCategory)
        {
            case enum_RobotCategory::null:
                category = "null";
                img = "src/Web/images/agent.png";
                break;
            case enum_RobotCategory::uav:
                category = "UAV";
                img = "src/Web/images/uav.png";
                break;
            case enum_RobotCategory::ugv:
                category = "UGV";
                img = "src/Web/images/ugv.png";
                break;
            case enum_RobotCategory::usv:
                category = "USV";
                img = "src/Web/images/usv.png";
                break;
            case enum_RobotCategory::chargingStation:
                category = "charging station";
                img = "src/Web/images/charging_station.png";
                break;
            default:
                category = "null";
                img = "src/Web/images/null.png";
                break;
        }
        switch(n.second.robotStatus)
        {
            case enum_RobotStatus::null:
                status = "null";
                break;
            case enum_RobotStatus::available:
                status = "Available";
                break;
            case enum_RobotStatus::executing:
                status = "Executing";
                break;
            case enum_RobotStatus::failure:
                status = "Failure";
                break;
            case enum_RobotStatus::emergency:
                status = "Emergency";
                break;
            case enum_RobotStatus::lowBattery:
                status = "Low Battery";
                break;
            default:
                break;
        }
        auto container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
        Wt::WProgressBar *bar = container->addNew<Wt::WProgressBar>();
        container->setContentAlignment(Wt::AlignmentFlag::Center);
        
        auto container2 = Wt::cpp14::make_unique<Wt::WContainerWidget>();
        Wt::WImage *image = container2->addNew<Wt::WImage>(Wt::WLink(img));
        container2->setContentAlignment(Wt::AlignmentFlag::Center);
        image->setAlternateText(category);
        image->resize(25,25);
        
        bar->setRange(0, 100);
        bar->setValue(n.second.batteryLevel);
        agentsTable->elementAt(row, 0)
        ->addNew<Wt::WText>(Wt::WString("{1}").arg(row));
        agentsTable->elementAt(row, 1)
        ->addWidget(std::move(container2));
        agentsTable->elementAt(row, 2)
        ->addNew<Wt::WText>(n.first);
        agentsTable->elementAt(row, 3)
        ->addNew<Wt::WText>(Wt::WString("("+std::to_string(n.second.robotsPosition.x)+","+ std::to_string(n.second.robotsPosition.y)+ "," + std::to_string(n.second.robotsPosition.z) +")"));
        agentsTable->elementAt(row, 4)
        ->addWidget(std::move(container));
        agentsTable->elementAt(row, 5)
        ->addNew<Wt::WText>(status);
    }
}

void WebApp::updateMissionList()
{
    missionsTable->clear();
    
    missionsTable->setHeaderCount(1);
    missionsTable->setWidth(Wt::WLength("100%"));
    missionsTable->elementAt(0, 0)->addNew<Wt::WText>("#");
    missionsTable->elementAt(0, 1)->addNew<Wt::WText>("Mission ID");
    missionsTable->elementAt(0, 2)->addNew<Wt::WText>("Mission Owner");
    missionsTable->elementAt(0, 3)->addNew<Wt::WText>("Mission Executioner*");
    missionsTable->elementAt(0, 4)->addNew<Wt::WText>("Relative Deadline [s]");
    missionsTable->elementAt(0, 5)->addNew<Wt::WText>("Estimated Execution Time [s]");
    missionsTable->elementAt(0, 6)->addNew<Wt::WText>("Execution Time [s]");
    missionsTable->elementAt(0, 7)->addNew<Wt::WText>("Status");
    
    int row = 0;
    std::unordered_map<std::string, s_MissionStatus> missionList;
    this->monitor->getAllMissionStatus(missionList);
    
    std::string status;
    
    for (auto n : missionList)
    {
        row++;
    
        switch(n.second.status)
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
            default:
                break;
        }
        
        missionsTable->elementAt(row, 0)
        ->addNew<Wt::WText>(Wt::WString("{1}").arg(row));
        missionsTable->elementAt(row, 1)
        ->addNew<Wt::WText>(n.first);
        missionsTable->elementAt(row, 2)
        ->addNew<Wt::WText>(n.second.missionOwner);
        missionsTable->elementAt(row, 3)
        ->addNew<Wt::WText>(n.second.missionExecutioner);
        missionsTable->elementAt(row, 4)
        ->addNew<Wt::WText>(Wt::WString("{1}").arg(n.second.relativeDeadline.count()/1000));
        missionsTable->elementAt(row, 5)
        ->addNew<Wt::WText>(Wt::WString("{1}").arg(n.second.estimatedExecutionTime.count()/1000));
        missionsTable->elementAt(row, 6)
        ->addNew<Wt::WText>(Wt::WString("{1}").arg(n.second.executionTime.count()/1000));
        missionsTable->elementAt(row, 7)
        ->addNew<Wt::WText>(status);
    }
}

void WebApp::insertTerminalLine()
{
    std::fstream s(path, s.in);
    std::string str;
    terminal->setText("");
    if (s.is_open())
    {
        while ( std::getline(s, str) )
            terminal->setText(terminal->text() + "\n"+ str);
        s.close();
    }
}
