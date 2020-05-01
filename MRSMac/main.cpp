//
//  main.cpp
//  MRSMac
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include "BlackBoard.hpp"
#include <unordered_map>


int main(){
    BlackBoard teste;
    s_pose* la = new s_pose;
    s_pose* la2 = new s_pose;
    s_robotsPose* robo = new s_robotsPose;
    std::unordered_map<std::string, s_pose>* mapRobotsPosition;
    mapRobotsPosition = new std::unordered_map<std::string, s_pose>;
    
    la->x = 1.0;
    la->y = 2.9;
    la->theta = 2.0;
    teste.setPosition(*la);
    
    robo->robotName = "Vulkan";
    robo->position = la;
    teste.setAllRobotsPosition(*robo);
    
    robo->robotName = "Thor";
    robo->position = la;
    teste.setAllRobotsPosition(*robo);
    
    robo->robotName = "Chronos";
    robo->position = la;
    teste.setAllRobotsPosition(*robo);
    
    robo->robotName = "Afrodite";
    robo->position = la;
    teste.setAllRobotsPosition(*robo);
    
    la->x = 99.0;
    la->y = 1.9;
    la->theta = 3.14;
    teste.setPosition(*la);
    
    robo->robotName = "Vulkan";
    robo->position = la;
    teste.setAllRobotsPosition(*robo);
    
    //delete la;
    teste.getPosition(*la2);
    std::cout << "Memcpy X : " << la2->x << " Y: " << la2->y << " Theta: " << la2->theta << std::endl;
    //delete la2;
    
    teste.getAllRobotsPosition(*mapRobotsPosition);
    
    for (const auto& n: *mapRobotsPosition){
        std::cout << "Name: [ " << n.first << " ] Position: [ " << n.second.x << "," << n.second.y << "," << n.second.theta << " ]" << std::endl;
    }
    
    teste.removeAllRobotsPosition(*robo);
    
    std::cout << "\n\n limpando" << std::endl;
    teste.getAllRobotsPosition(*mapRobotsPosition);
    
    for (const auto& n: *mapRobotsPosition){
        std::cout << "Name: [ " << n.first << " ] Position: [ " << n.second.x << "," << n.second.y << "," << n.second.theta << " ]" << std::endl;
    }
    
    //delete mapRobotsPosition;
    return 0;
}
