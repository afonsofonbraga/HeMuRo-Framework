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
    
    path = getenv("HOME") + std::string("/Github/MRSFramework/logs/PathController.txt");
    int n = 0;
    std::string str;
    std::fstream s(path, s.in);
    if (s.is_open()) {
        s >> str >> n;
        s.close();
    }
    path = getenv("HOME") + std::string("/Github/MRSFramework/logs/Logger") + std::to_string(n) + "/Logger.txt";
    
    setHeight(720);
    setWidth(1280);
    setStyleClass("backgound-box");
    
    auto layout = setLayout(Wt::cpp14::make_unique<Wt::WBorderLayout>());
    auto container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    
    auto item = container->addWidget(Wt::cpp14::make_unique<Wt::WText>("<h1>HeMuRo Framework</h1>"));
    container->setStyleClass("title-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::North);
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    item = container->addWidget(Wt::cpp14::make_unique<Wt::WText>("<h3>Menu</h3>"));
    container->setStyleClass("sidebar-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::West);
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    item = container->addWidget(Wt::cpp14::make_unique<Wt::WText>("<h3>Information</h3> Here you can find all the <br /> information regarding <br />agents' status.<br />More information will be <br /> available in the future!<br /> Keep in touch!"));
    container->setStyleClass("sidebar-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::East);
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    table = container->addWidget(Wt::cpp14::make_unique<Wt::WTable>());
    container->setStyleClass("table-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::Center);
    
    container = Wt::cpp14::make_unique<Wt::WContainerWidget>();
    container->addWidget(Wt::cpp14::make_unique<Wt::WText>("Terminal"));
    container->addWidget(Wt::cpp14::make_unique<Wt::WBreak>());
    terminal = container->addWidget(Wt::cpp14::make_unique<Wt::WTextArea>());
    
    terminal->setColumns(150);
    terminal->setRows(15);
    terminal->setReadOnly(true);
    terminal->setText("Starting Web Terminal...");
    
    container->setStyleClass("terminal-box");
    layout->addWidget(std::move(container), Wt::LayoutPosition::South);
    
    
    timer_ = Wt::cpp14::make_unique<Wt::WTimer>();
    timer_->setInterval(std::chrono::milliseconds{1000});
    timer_->timeout().connect(this, &WebApp::insertTerminalLine);
    timer_->timeout().connect(this, &WebApp::updateAgentList);
    timer_->start();
}

WebApp::~WebApp()
{
}

void WebApp::updateAgentList()
{
    table->clear();
    
    table->setHeaderCount(1);
    table->setWidth(Wt::WLength("100%"));
    table->elementAt(0, 0)->addNew<Wt::WText>("#");
    table->elementAt(0, 1)->addNew<Wt::WText>("Agent Name");
    table->elementAt(0, 2)->addNew<Wt::WText>("Category");
    table->elementAt(0, 3)->addNew<Wt::WText>("Position (x,y,z)");
    table->elementAt(0, 4)->addNew<Wt::WText>("Battery Level");
    table->elementAt(0, 5)->addNew<Wt::WText>("Status");
    
    
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
                img = "001-vr-glasses.png";
                break;
            case enum_RobotCategory::uav:
                category = "UAV";
                img = "023-drone-2.png";
                break;
            case enum_RobotCategory::ugv:
                category = "UGV";
                img = "007-robot.png";
                break;
            case enum_RobotCategory::usv:
                category = "USV";
                img = "submarine.png";
                break;
            case enum_RobotCategory::chargingStation:
                category = "Charging Station";
                img = "charging-station.png";
                break;
            default:
                category = "null";
                img = "question-mark.png";
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
        table->elementAt(row, 0)
        ->addNew<Wt::WText>(Wt::WString("{1}").arg(row));
        table->elementAt(row, 1)
        ->addNew<Wt::WText>(n.first);
        table->elementAt(row, 2)
        ->addWidget(std::move(container2));
        table->elementAt(row, 3)
        ->addNew<Wt::WText>(Wt::WString("("+std::to_string(n.second.robotsPosition.x)+","+ std::to_string(n.second.robotsPosition.y)+ "," + std::to_string(n.second.robotsPosition.z) +")"));
        table->elementAt(row, 4)
        ->addWidget(std::move(container));
        table->elementAt(row, 5)
        ->addNew<Wt::WText>(status);
    }
}

// <Wt::WText>(category);

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
